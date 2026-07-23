# -*- coding: UTF-8 -*-
"""
Hanger tagging functionality for Revit MCP
Tags hangers (or any point-locatable category) in a view with independent
tags placed so tag heads never overlap each other, leader tails never cross,
and (optionally) tags avoid landing on top of nearby elements such as
piping or insulation.
"""
import json
import logging
import math

# NOTE: Only import DB here. `pyrevit.routes` must NOT be imported at module
# level: it instantiates a .NET HTTP handler interface that fails under the
# CPython engine ("interface takes exactly one argument"), which would break
# the ribbon button. `routes` is imported lazily inside register_hanger_routes,
# which only ever runs under IronPython at extension startup.
from pyrevit import DB

logger = logging.getLogger(__name__)

DEFAULT_AVOID_CATEGORIES = [
    "MEP Fabrication Pipework",
    "Insulation",
    "MEP Fabrication Hangers",
]
DEFAULT_RADII = [
    1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0, 6.0,
    8.0, 10.0, 13.0, 16.0, 20.0, 25.0, 30.0,
]
# 16 directions (every 22.5 deg), tried from the shortest radius outward.
# Ordered by preference: straight up/down and sideways first, then the
# diagonals, so tags favour clean orthogonal leaders before angled ones.
DEFAULT_ANGLES_DEG = [
    90, 270, 0, 180,          # up, down, right, left
    45, 135, 315, 225,        # 45-deg diagonals
    68, 112, 292, 248,        # near-vertical
    23, 157, 337, 203,        # near-horizontal
]


def _box_of(bb):
    return (bb.Min.X, bb.Min.Y, bb.Max.X, bb.Max.Y)


def _box_overlap(a, b, margin):
    return not (
        a[2] + margin < b[0]
        or b[2] + margin < a[0]
        or a[3] + margin < b[1]
        or b[3] + margin < a[1]
    )


def _seg_intersect(p1, p2, p3, p4):
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    d1 = cross(p3, p4, p1)
    d2 = cross(p3, p4, p2)
    d3 = cross(p1, p2, p3)
    d4 = cross(p1, p2, p4)
    return ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and (
        (d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)
    )


def _nearest_on_box(px, py, box):
    """Closest point to (px, py) on/inside the axis-aligned box."""
    minx, miny, maxx, maxy = box
    nx = min(max(px, minx), maxx)
    ny = min(max(py, miny), maxy)
    return (nx, ny)


def _seg_intersects_box(p1, p2, box):
    """True if segment p1->p2 crosses or lies inside the axis-aligned box."""
    minx, miny, maxx, maxy = box
    # Quick reject if the segment's bounding box misses the box entirely.
    if (max(p1[0], p2[0]) < minx or min(p1[0], p2[0]) > maxx
            or max(p1[1], p2[1]) < miny or min(p1[1], p2[1]) > maxy):
        return False
    corners = [(minx, miny), (maxx, miny), (maxx, maxy), (minx, maxy)]
    for i in range(4):
        if _seg_intersect(p1, p2, corners[i], corners[(i + 1) % 4]):
            return True
    # Segment fully inside the box (an endpoint inside implies intersection).
    if minx <= p1[0] <= maxx and miny <= p1[1] <= maxy:
        return True
    return False


# Penalty weights for scoring a candidate position (higher = worse). Used to
# pick the least-bad spot when no fully clear position exists.
_PEN_TAG_OVERLAP = 100.0
_PEN_LEADER_CROSS = 60.0
_PEN_OBSTACLE = 45.0
_PEN_LEADER_THRU_TEXT = 25.0


def _segments(pts):
    """Consecutive segments of a polyline given as a list of points."""
    return [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]


def _candidate_penalty(cand_box, leader_pts, placed, obstacles,
                       tag_margin, obstacle_margin):
    """Return a penalty score for a candidate position; 0.0 means fully clear.

    `leader_pts` is the leader polyline (2 points for a straight leader, 3 for
    an elbow). Every placed tag stores its own leader polyline in p["leader"].
    """
    pen = 0.0
    my_segs = _segments(leader_pts)
    for p in placed:
        if _box_overlap(cand_box, p["box"], tag_margin):
            pen += _PEN_TAG_OVERLAP
        p_segs = _segments(p["leader"])
        for a1, a2 in my_segs:
            for b1, b2 in p_segs:
                if _seg_intersect(a1, a2, b1, b2):
                    pen += _PEN_LEADER_CROSS
            if _seg_intersects_box(a1, a2, p["box"]):
                pen += _PEN_LEADER_THRU_TEXT
        for b1, b2 in p_segs:
            if _seg_intersects_box(b1, b2, cand_box):
                pen += _PEN_LEADER_THRU_TEXT
    for ob in obstacles:
        if _box_overlap(cand_box, ob, obstacle_margin):
            pen += _PEN_OBSTACLE
    return pen


def _leader_routes(hx, hy, cx, cy, ebox):
    """Candidate leader polylines from the target to head (hx, hy).

    Straight first (shortest), then two L-shaped detours (vertical-then-
    horizontal and horizontal-then-vertical) whose elbow lets the leader
    route around obstacles such as other tags' text.
    """
    straight = [_nearest_on_box(hx, hy, ebox), (hx, hy)]
    # Elbow at (hx, cy): leader runs from the target out to x=hx then up to head.
    elbow_v = (hx, cy)
    route_v = [_nearest_on_box(elbow_v[0], elbow_v[1], ebox), elbow_v, (hx, hy)]
    # Elbow at (cx, hy): leader runs from the target across to y=hy then to head.
    elbow_h = (cx, hy)
    route_h = [_nearest_on_box(elbow_h[0], elbow_h[1], ebox), elbow_h, (hx, hy)]
    return [straight, route_v, route_h]


def _cluster_2d(recs, dist):
    """Greedily group records whose targets are within `dist` of the cluster."""
    unassigned = list(recs)
    clusters = []
    while unassigned:
        cluster = [unassigned.pop(0)]
        changed = True
        while changed:
            changed = False
            for r in unassigned[:]:
                if any((r["cx"] - c["cx"]) ** 2 + (r["cy"] - c["cy"]) ** 2
                       <= dist * dist for c in cluster):
                    cluster.append(r)
                    unassigned.remove(r)
                    changed = True
        clusters.append(cluster)
    return clusters


def _run_clusters(hangers, x_gap):
    """Group targets into runs separated by horizontal gaps wider than x_gap."""
    ts = sorted(hangers, key=lambda t: t[1])
    runs = []
    cur = [ts[0]]
    for t in ts[1:]:
        if t[1] - cur[-1][1] > x_gap:
            runs.append(cur)
            cur = [t]
        else:
            cur.append(t)
    runs.append(cur)
    return runs


def _tag_ganged_columns(doc, view, tag_symbol, hangers, obstacle_boxes,
                        to_tag, tags_to_remove, tag_margin):
    """Place tags in vertical columns flanking each run (the manual method).

    Tags stack at a fixed X beside the pipe run, ordered top-to-bottom to match
    their targets so the short inward leaders form a non-crossing fan. Big runs
    are split across left and right columns; columns are pushed into clear
    space off the obstacles.
    """
    tagged_ids = []
    lengths = []
    placed_boxes = []

    t = DB.Transaction(doc, "Tag - Ganged Columns")
    t.Start()
    try:
        for tid in tags_to_remove:
            doc.Delete(tid)
        if not tag_symbol.IsActive:
            tag_symbol.Activate()
            doc.Regenerate()

        # Pass 1: create each tag, establish its leader, measure its head box.
        recs = []
        for el, cx, cy, z, ebox in to_tag:
            ref = DB.Reference(el)
            tag = DB.IndependentTag.Create(
                doc, tag_symbol.Id, view.Id, ref, True,
                DB.TagOrientation.Horizontal, DB.XYZ(cx, cy + 2.0, z))
            has_free = tag.CanLeaderEndConditionBeAssigned(
                DB.LeaderEndCondition.Free)
            if has_free:
                tag.HasLeader = True
                tag.LeaderEndCondition = DB.LeaderEndCondition.Free
                tag.SetLeaderEnd(ref, DB.XYZ(cx, cy, z))
            doc.Regenerate()
            tag.HasLeader = False
            doc.Regenerate()
            bb = tag.get_BoundingBox(view)
            tag.HasLeader = True
            doc.Regenerate()
            w = bb.Max.X - bb.Min.X
            h = bb.Max.Y - bb.Min.Y
            recs.append({"el": el, "cx": cx, "cy": cy, "z": z, "ebox": ebox,
                         "tag": tag, "ref": ref, "has_free": has_free,
                         "w": w, "h": h})

        if not recs:
            t.Commit()
            return tagged_ids, lengths

        row_h = max(r["h"] for r in recs) + 0.35   # consistent row spacing
        clear0 = 2.0                                 # gap from cluster to stack

        # Cluster targets into small LOCAL groups (nearby in both X and Y): a
        # dense hanger location becomes a compact stack right beside it, rather
        # than one giant full-height column with long fanned leaders.
        clusters = _cluster_2d(recs, dist=6.0)

        for cluster in clusters:
            cminx = min(r["cx"] for r in cluster)
            cmaxx = max(r["cx"] for r in cluster)
            # Split the cluster's tags left/right of the local group by x, so
            # each side's leaders stay short and the two fans don't tangle.
            cluster.sort(key=lambda r: r["cx"])
            mid = (len(cluster) + 1) // 2
            sides = [(-1, cluster[:mid]), (1, cluster[mid:])]

            for side, col in sides:
                if not col:
                    continue
                # Order the stack by target Y (top first). Tie-break by X so
                # targets at the same height fan to the column without their
                # leaders swapping and crossing.
                col.sort(key=lambda r: (-r["cy"], -side * r["cx"]))
                n = len(col)
                colw = max(r["w"] for r in col)
                ymean = sum(r["cy"] for r in col) / n
                y_top = ymean + (n - 1) * row_h / 2.0

                # Offset the stack just clear of the local cluster; nudge out
                # further only if it lands on an already-placed stack (which
                # would tangle their leaders). Pipework is not avoided here so
                # stacks stay close and leaders short.
                clear = clear0
                for _ in range(40):
                    if side < 0:
                        edge = cminx - clear
                        head_x = edge - (colw / 2.0)
                        col_box = (head_x - colw / 2.0, y_top - (n - 1) * row_h,
                                   edge, y_top)
                    else:
                        edge = cmaxx + clear
                        head_x = edge + (colw / 2.0)
                        col_box = (edge, y_top - (n - 1) * row_h,
                                   head_x + colw / 2.0, y_top)
                    if not any(_box_overlap(col_box, pb, 0.2)
                               for pb in placed_boxes):
                        break
                    clear += 1.0

                # Each leader gets a horizontal shoulder at its own row: it runs
                # flat from the head out to an elbow on the cluster edge, then
                # angles into the cluster to the target. The flat shoulder sits
                # at a unique row height so it only touches its own text box (no
                # through-text within the stack), and the angled part heads into
                # the pipe cluster where there is no text. Elbows stack in row
                # order and targets in Y order, so the angled fan cannot cross.
                edge_x = cminx if side < 0 else cmaxx
                for i, r in enumerate(col):
                    hy = y_top - i * row_h
                    hx = head_x
                    tag = r["tag"]
                    ref = r["ref"]
                    tag.TagHeadPosition = DB.XYZ(hx, hy, r["z"])
                    elbow = (edge_x, hy)                 # shoulder end, at row Y
                    tgt = _nearest_on_box(edge_x, r["cy"], r["ebox"])
                    if r["has_free"]:
                        tag.SetLeaderEnd(ref, DB.XYZ(tgt[0], tgt[1], r["z"]))
                        try:
                            tag.SetLeaderElbow(ref, DB.XYZ(elbow[0], elbow[1], r["z"]))
                        except Exception:
                            pass
                    hw = r["w"] / 2.0
                    hh = r["h"] / 2.0
                    placed_boxes.append((hx - hw, hy - hh, hx + hw, hy + hh))
                    tagged_ids.append(r["el"].Id.IntegerValue)
                    lengths.append(math.hypot(hx - elbow[0], hy - elbow[1])
                                   + math.hypot(elbow[0] - tgt[0],
                                                elbow[1] - tgt[1]))

        doc.Regenerate()
        t.Commit()
    except Exception:
        t.RollBack()
        raise
    return tagged_ids, lengths


def tag_hangers_no_overlap(
    doc,
    view_name=None,
    hanger_category="MEP Fabrication Hangers",
    tag_family_category="MEP Fabrication Hanger Tags",
    avoid_categories=None,
    retag_existing=False,
    tag_margin=0.3,
    obstacle_margin=0.15,
    radii=None,
    angles_deg=None,
    max_leader=None,
    layout="radial",
):
    """
    Tag every element of `hanger_category` visible in a view with an
    IndependentTag, guaranteeing tag heads never overlap and leader tails
    never cross. Each tag uses a free-end leader and searches increasing
    radii in multiple directions to keep the leader as short as possible
    while clearing other tags and (optionally) obstacle categories.

    Args:
        doc: Revit document
        view_name (str): Name of the view to tag in. Defaults to the active view.
        hanger_category (str): Category name of elements to tag.
        tag_family_category (str): Category name of the tag family to use.
        avoid_categories (list): Category names whose elements tags must not
            overlap (e.g. piping, insulation). Pass [] to disable.
        retag_existing (bool): If True, remove existing tags of this type on
            these hangers in the view first. If False, hangers that already
            have a tag in the view are left alone.
        tag_margin (float): Minimum clearance (ft) required between tag heads.
        obstacle_margin (float): Minimum clearance (ft) required between a
            tag head and an obstacle element.
        radii (list): Candidate leader lengths (ft) to try, shortest first.
        angles_deg (list): Candidate leader directions (degrees), in the
            order they should be preferred.

    Returns:
        dict: Summary of the tagging operation.
    """
    try:
        if view_name:
            view = next(
                (
                    v
                    for v in DB.FilteredElementCollector(doc).OfClass(DB.View)
                    if v.Name == view_name and not v.IsTemplate
                ),
                None,
            )
            if view is None:
                return {
                    "status": "error",
                    "message": "View '{}' not found".format(view_name),
                }
        else:
            view = doc.ActiveView

        if avoid_categories is None:
            avoid_categories = DEFAULT_AVOID_CATEGORIES
        radii = radii or DEFAULT_RADII
        angles_deg = angles_deg or DEFAULT_ANGLES_DEG
        # Leader cap is applied after the targets are collected (below), because
        # "auto" needs to know how many there are.

        tag_symbol = None
        for s in DB.FilteredElementCollector(doc).OfClass(DB.FamilySymbol):
            try:
                if s.Category.Name == tag_family_category:
                    tag_symbol = s
                    break
            except Exception:
                pass

        if tag_symbol is None:
            return {
                "status": "error",
                "message": "No tag family found for category '{}'".format(
                    tag_family_category
                ),
            }

        collector = DB.FilteredElementCollector(doc, view.Id).WhereElementIsNotElementType()

        hangers = []
        obstacle_boxes = []
        avoid_set = set(avoid_categories)
        for el in collector:
            try:
                cat = el.Category.Name if el.Category else None
            except Exception:
                cat = None
            if cat == hanger_category:
                bbox = el.get_BoundingBox(view)
                if bbox:
                    cx = (bbox.Min.X + bbox.Max.X) / 2.0
                    cy = (bbox.Min.Y + bbox.Max.Y) / 2.0
                    z = bbox.Max.Z
                    # Keep the element's plan box: the leader may attach to the
                    # nearest point on it (not just the centre), which matters
                    # for extended targets like spools.
                    hangers.append((el, cx, cy, z, _box_of(bbox)))
                    # Hangers are tag targets, but if the category is also in
                    # the avoid list, treat every hanger box as an obstacle so
                    # tag heads never sit on top of any hanger (own or others).
                    if cat in avoid_set:
                        obstacle_boxes.append(_box_of(bbox))
            elif cat in avoid_set:
                bbox = el.get_BoundingBox(view)
                if bbox:
                    obstacle_boxes.append(_box_of(bbox))

        if not hangers:
            return {
                "status": "error",
                "message": "No elements of category '{}' found in view '{}'".format(
                    hanger_category, view.Name
                ),
            }

        # Place the most crowded hangers first: while the field is still empty
        # they can claim the few clear spots, leaving the roomier hangers to
        # adapt around them. Crowdedness = number of other hangers nearby.
        NEIGHBOR_R = 8.0
        NEIGHBOR_R2 = NEIGHBOR_R * NEIGHBOR_R

        def _crowd(r):
            cx, cy = r[1], r[2]
            n = 0
            for o in hangers:
                if o is r:
                    continue
                if (o[1] - cx) ** 2 + (o[2] - cy) ** 2 <= NEIGHBOR_R2:
                    n += 1
            return n

        hangers.sort(key=lambda r: (-_crowd(r), r[1], r[2]))

        # Apply the leader-length cap now that the target count is known.
        # "auto" scales the cap with sqrt(count): to pack N tags without
        # overlap they need an area ~ N, so the reach they require grows like
        # sqrt(N). Sparse views stay tight; dense views get room to spread.
        if max_leader == "auto":
            eff_cap = max(6.0, 1.3 * math.sqrt(len(hangers)))
        else:
            eff_cap = max_leader
        if eff_cap is not None:
            radii = [r for r in radii if r <= eff_cap] or [radii[0]]

        existing_tags = DB.FilteredElementCollector(doc, view.Id).OfClass(DB.IndependentTag)
        already_tagged_ids = set()
        tags_to_remove = []
        for tg in existing_tags:
            try:
                tagged_ids = list(tg.GetTaggedLocalElementIds())
            except Exception:
                tagged_ids = []
            for tid in tagged_ids:
                el = doc.GetElement(tid)
                try:
                    cat = el.Category.Name if el and el.Category else None
                except Exception:
                    cat = None
                if cat == hanger_category:
                    already_tagged_ids.add(tid.IntegerValue)
                    if retag_existing:
                        tags_to_remove.append(tg.Id)

        # Ganged-column layout: stack tags in vertical columns beside each run.
        if layout == "ganged":
            to_tag = [h for h in hangers
                      if retag_existing
                      or h[0].Id.IntegerValue not in already_tagged_ids]
            skipped_ids = [h[0].Id.IntegerValue for h in hangers
                           if not retag_existing
                           and h[0].Id.IntegerValue in already_tagged_ids]
            g_tagged, g_lengths = _tag_ganged_columns(
                doc, view, tag_symbol, hangers, obstacle_boxes,
                to_tag, tags_to_remove, tag_margin)
            result = {
                "status": "success",
                "view": view.Name,
                "hanger_category": hanger_category,
                "tagged_count": len(g_tagged),
                "skipped_already_tagged": skipped_ids,
                "could_not_clear": [],
                "layout": "ganged",
            }
            if g_lengths:
                result["leader_length_ft"] = {
                    "min": round(min(g_lengths), 2),
                    "max": round(max(g_lengths), 2),
                    "avg": round(sum(g_lengths) / len(g_lengths), 2),
                }
            return result

        placed = []
        tagged_ids = []
        skipped_ids = []
        failed_ids = []
        lengths = []

        t = DB.Transaction(doc, "Tag Hangers - No Overlap No Crossing")
        t.Start()
        try:
            for tid in tags_to_remove:
                doc.Delete(tid)

            if not tag_symbol.IsActive:
                tag_symbol.Activate()
                doc.Regenerate()

            for el, cx, cy, z, ebox in hangers:
                if el.Id.IntegerValue in already_tagged_ids and not retag_existing:
                    skipped_ids.append(el.Id.IntegerValue)
                    continue

                ref = DB.Reference(el)
                init_hx, init_hy = cx, cy + radii[0]
                tag = DB.IndependentTag.Create(
                    doc, tag_symbol.Id, view.Id, ref, False,
                    DB.TagOrientation.Horizontal, DB.XYZ(init_hx, init_hy, z),
                )
                # Establish the free-end leader (anchored at the element) BEFORE
                # measuring. Some tag types (e.g. Assembly Tags) render the head
                # at the element's own location until a leader exists, so the
                # box must be measured with the leader in place to reflect where
                # the head will actually sit.
                tag.TagHeadPosition = DB.XYZ(init_hx, init_hy, z)
                has_free = tag.CanLeaderEndConditionBeAssigned(
                    DB.LeaderEndCondition.Free)
                if has_free:
                    tag.HasLeader = True
                    tag.LeaderEndCondition = DB.LeaderEndCondition.Free
                    tag.SetLeaderEnd(ref, DB.XYZ(cx, cy, z))
                doc.Regenerate()

                # Measure the head-only box (leader line excluded) and record
                # its extents relative to the head position. The box translates
                # rigidly with the head, so candidates are evaluated purely
                # geometrically with no further regeneration.
                tag.HasLeader = False
                doc.Regenerate()
                bb = tag.get_BoundingBox(view)
                tag.HasLeader = True
                doc.Regenerate()

                dxmin = bb.Min.X - init_hx
                dymin = bb.Min.Y - init_hy
                dxmax = bb.Max.X - init_hx
                dymax = bb.Max.Y - init_hy

                placed_ok = False
                chosen = None
                # Track the least-bad candidate in case none is fully clear.
                best_score = None
                best_chosen = None
                for radius in radii:
                    for ang in angles_deg:
                        rad = math.radians(ang)
                        hx = cx + radius * math.cos(rad)
                        hy = cy + radius * math.sin(rad)
                        cand_box = (hx + dxmin, hy + dymin, hx + dxmax, hy + dymax)
                        # Try a straight leader first, then elbow detours; the
                        # elbow lets the leader route around other tags' text.
                        for ri, route in enumerate(
                                _leader_routes(hx, hy, cx, cy, ebox)):
                            pen = _candidate_penalty(
                                cand_box, route, placed,
                                obstacle_boxes, tag_margin, obstacle_margin,
                            )
                            if pen == 0.0:
                                chosen = (cand_box, route)
                                placed_ok = True
                                break
                            # Prefer low penalty, then short leader, then a
                            # straight leader over an elbow (ri tiebreak).
                            score = pen + radius * 0.01 + ri * 0.001
                            if best_score is None or score < best_score:
                                best_score = score
                                best_chosen = (cand_box, route)
                        if placed_ok:
                            break
                    if placed_ok:
                        break

                if not placed_ok:
                    # No fully clear spot: take the least-bad candidate so the
                    # unavoidable compromise is minimal instead of defaulting
                    # to a fixed position that may overlap several neighbours.
                    chosen = best_chosen

                cand_box, route = chosen
                leader_start = route[0]
                leader_end = route[-1]
                elbow = route[1] if len(route) == 3 else None
                # Move the head, anchor the free-end leader at the element, and
                # add the elbow if the chosen route bends.
                tag.TagHeadPosition = DB.XYZ(leader_end[0], leader_end[1], z)
                if has_free:
                    tag.SetLeaderEnd(ref, DB.XYZ(leader_start[0], leader_start[1], z))
                    if elbow is not None:
                        try:
                            tag.SetLeaderElbow(ref, DB.XYZ(elbow[0], elbow[1], z))
                        except Exception:
                            pass
                doc.Regenerate()

                placed.append({
                    "id": tag.Id.IntegerValue, "box": cand_box,
                    "leader": route,
                })
                if placed_ok:
                    tagged_ids.append(el.Id.IntegerValue)
                    seglen = sum(
                        math.hypot(route[k + 1][0] - route[k][0],
                                   route[k + 1][1] - route[k][1])
                        for k in range(len(route) - 1))
                    lengths.append(seglen)
                else:
                    failed_ids.append(el.Id.IntegerValue)

            t.Commit()
        except Exception:
            t.RollBack()
            raise

        result = {
            "status": "success",
            "view": view.Name,
            "hanger_category": hanger_category,
            "tagged_count": len(tagged_ids),
            "skipped_already_tagged": skipped_ids,
            "could_not_clear": failed_ids,
        }
        if lengths:
            result["leader_length_ft"] = {
                "min": round(min(lengths), 2),
                "max": round(max(lengths), 2),
                "avg": round(sum(lengths) / len(lengths), 2),
            }
        return result

    except Exception as e:
        logger.error("Error in tag_hangers_no_overlap: %s", e)
        return {"status": "error", "message": "Failed to tag hangers: {}".format(str(e))}


# Category / tag-family defaults for tagging spools (fabrication assemblies).
SPOOL_CATEGORY = "Assemblies"
SPOOL_TAG_CATEGORY = "Assembly Tags"


# Spools sit inside dense pipework, so cap the leader length. "auto" scales
# the cap with the spool count: a sparse view keeps leaders tight (~6 ft),
# while a dense view lets them reach out far enough to spread apart.
SPOOL_MAX_LEADER = "auto"


# Wider tag-to-tag clearance for spools so labels spread out rather than
# stacking tightly.
SPOOL_TAG_MARGIN = 0.8


def tag_spools(doc, view_name=None, retag_existing=False):
    """Tag every spool (fabrication Assembly) in a view with the same
    no-overlap / no-crossing / no-leader-through-text rules as the hanger
    tool. Tag heads avoid piping, insulation and hangers and are spaced out;
    the leader-length cap scales automatically with how many spools there are.
    """
    return tag_hangers_no_overlap(
        doc,
        view_name=view_name,
        hanger_category=SPOOL_CATEGORY,
        tag_family_category=SPOOL_TAG_CATEGORY,
        avoid_categories=[
            "MEP Fabrication Pipework",
            "Insulation",
            "MEP Fabrication Hangers",
        ],
        retag_existing=retag_existing,
        layout="ganged",
    )


def register_hanger_routes(api):
    """Register hanger-tagging routes with the API"""
    from pyrevit import routes

    @api.route("/tag_hangers/", methods=["POST"])
    def tag_hangers_route(doc, request):
        """
        Tag hangers in a view with non-overlapping tags and non-crossing leaders.

        Expected JSON payload (all fields optional):
        {
            "view_name": "PLUMBING LEVEL 02 ... HANGERS",
            "hanger_category": "MEP Fabrication Hangers",
            "tag_family_category": "MEP Fabrication Hanger Tags",
            "avoid_categories": ["MEP Fabrication Pipework", "Insulation"],
            "retag_existing": false,
            "tag_margin": 0.3,
            "obstacle_margin": 0.15
        }
        """
        try:
            data = (
                json.loads(request.data) if isinstance(request.data, str) else request.data
            ) or {}

            result = tag_hangers_no_overlap(
                doc,
                view_name=data.get("view_name"),
                hanger_category=data.get("hanger_category", "MEP Fabrication Hangers"),
                tag_family_category=data.get(
                    "tag_family_category", "MEP Fabrication Hanger Tags"
                ),
                avoid_categories=data.get("avoid_categories"),
                retag_existing=data.get("retag_existing", False),
                tag_margin=data.get("tag_margin", 0.3),
                obstacle_margin=data.get("obstacle_margin", 0.15),
            )

            status_code = 200 if result.get("status") == "success" else 500
            return routes.make_response(data=result, status=status_code)

        except Exception as e:
            logger.error("Error in tag_hangers route: %s", e)
            return routes.make_response(data={"error": str(e)}, status=500)
