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


def _box_point_dist(box, px, py):
    """Distance from (px, py) to the nearest point on/inside the box (0 if
    the point is inside)."""
    nx, ny = _nearest_on_box(px, py, box)
    return math.hypot(px - nx, py - ny)


# How far inward (ft) to pull a leader end off the bbox edge toward the element
# centre, so the end lands on the tagged geometry rather than floating at a
# bounding-box corner.
_LEADER_TOUCH_INSET = 0.25


def _leader_touch(px, py, box):
    """A leader-end point that touches the tagged element.

    Starts from the point on the element's box nearest the approach (px, py) --
    which keeps the leader short by attaching at the near edge -- then nudges it
    inward toward the box centre by `_LEADER_TOUCH_INSET`. That inward bias
    lands the end on the object's footprint instead of at an empty bbox corner
    (the axis-aligned box can be larger than the geometry inside it). For a tag
    smaller than the inset the centre itself is returned.
    """
    nx, ny = _nearest_on_box(px, py, box)
    cx = (box[0] + box[2]) / 2.0
    cy = (box[1] + box[3]) / 2.0
    dx, dy = cx - nx, cy - ny
    d = math.hypot(dx, dy)
    if d <= _LEADER_TOUCH_INSET:
        return (cx, cy)
    f = _LEADER_TOUCH_INSET / d
    return (nx + dx * f, ny + dy * f)


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
_PEN_LEADER_THRU_OBSTACLE = 40.0
_PEN_LEADER_THRU_TEXT = 25.0

# A leader legitimately terminates on the pipework/assembly it tags, so ignore
# obstacle crossings within this distance (ft) of the leader's target end.
# Only genuine crossings further out along the leader count against it.
_LEADER_OBSTACLE_NEAR = 1.5


def _any_overlap(box, boxes, margin):
    """True if `box` overlaps any box in `boxes` (explicit loop).

    Used instead of `any(... for ...)` in the deeply-nested layout code: under
    IronPython, generator expressions that close over the layout's shared cell
    variables intermittently raise "Sequence contains no elements", whereas a
    plain loop is reliable.
    """
    for b in boxes:
        if _box_overlap(box, b, margin):
            return True
    return False


def _count_overlap(box, boxes, margin=0.0):
    """Number of boxes in `boxes` that overlap `box` (explicit loop)."""
    c = 0
    for b in boxes:
        if _box_overlap(box, b, margin):
            c += 1
    return c


def _spread_y(desired, gap):
    """Declutter labels along Y with minimum displacement.

    Given each label's desired Y, return final Ys that keep the labels in
    descending-desired order and at least `gap` apart, moving them as little as
    possible. This is the manual spool-tagging behaviour: each head sits at its
    own spool's height, nudged apart only where spools crowd. Standard
    pool-adjacent-violators (isotonic) algorithm; input order is preserved in
    the returned list (positions align to input indices).
    """
    n = len(desired)
    if n == 0:
        return []
    order = sorted(range(n), key=lambda i: -desired[i])
    # Each block: (members top-down, s) where the block's top position
    # T = s / len(members) minimises squared displacement, and member k sits at
    # T - k*gap. s = sum(desired[member_k] + k*gap).
    blocks = []
    for i in order:
        members = [i]
        s = desired[i]
        while blocks:
            pm, ps = blocks[-1]
            t_prev = ps / len(pm)
            t_new = s / len(members)
            # Previous block is above; overlap if this block's top is higher
            # than the previous block's bottom minus one gap.
            if t_new > t_prev - len(pm) * gap:
                members = pm + members
                s = sum(desired[members[k]] + k * gap
                        for k in range(len(members)))
                blocks.pop()
            else:
                break
        blocks.append((members, s))
    pos = [0.0] * n
    for members, s in blocks:
        top = s / len(members)
        for k, idx in enumerate(members):
            pos[idx] = top - k * gap
    return pos


def _segments(pts):
    """Consecutive segments of a polyline given as a list of points."""
    return [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]


def _leader_hits_obstacle(leader_pts, obstacles, tgt_pt,
                          near=_LEADER_OBSTACLE_NEAR):
    """True if the leader crosses an obstacle box away from its target end.

    `leader_pts[0]` is the end that attaches to the tagged element. Obstacle
    boxes within `near` of `tgt_pt` are ignored -- every leader legitimately
    ends on the pipe/assembly it tags, so those crossings are expected. A
    crossing further along the leader (over unrelated pipework or insulation)
    is a genuine one and returns True.
    """
    segs = _segments(leader_pts)
    for ob in obstacles:
        if near > 0 and _box_point_dist(ob, tgt_pt[0], tgt_pt[1]) <= near:
            continue
        for a, b in segs:
            if _seg_intersects_box(a, b, ob):
                return True
    return False


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
    # Penalise a leader that runs across an obstacle away from its own target
    # (crossings near the target end are expected and ignored).
    if leader_pts and _leader_hits_obstacle(leader_pts, obstacles, leader_pts[0]):
        pen += _PEN_LEADER_THRU_OBSTACLE
    return pen


def _leader_routes(hx, hy, cx, cy, ebox):
    """Candidate leader polylines from the target to head (hx, hy).

    Straight first (shortest), then two L-shaped detours (vertical-then-
    horizontal and horizontal-then-vertical) whose elbow lets the leader
    route around obstacles such as other tags' text.
    """
    straight = [_leader_touch(hx, hy, ebox), (hx, hy)]
    # Elbow at (hx, cy): leader runs from the target out to x=hx then up to head.
    elbow_v = (hx, cy)
    route_v = [_leader_touch(elbow_v[0], elbow_v[1], ebox), elbow_v, (hx, hy)]
    # Elbow at (cx, hy): leader runs from the target across to y=hy then to head.
    elbow_h = (cx, hy)
    route_h = [_leader_touch(elbow_h[0], elbow_h[1], ebox), elbow_h, (hx, hy)]
    return [straight, route_v, route_h]


def _callout_regions(doc, view):
    """Model-XY rectangles (minx, miny, maxx, maxy) of the callouts drawn on a
    view. Elements inside a callout are detailed in the callout view, so the
    parent view should not tag them."""
    regions = []
    ref_names = set()
    for el in DB.FilteredElementCollector(doc, view.Id).WhereElementIsNotElementType():
        try:
            if el.Category and el.Category.Name == "Views":
                p = el.get_Parameter(DB.BuiltInParameter.VIEW_NAME)
                if p and p.AsString():
                    ref_names.add(p.AsString())
        except Exception:
            pass
    if not ref_names:
        return regions
    for vw in DB.FilteredElementCollector(doc).OfClass(DB.View):
        try:
            if vw.Name not in ref_names:
                continue
            cb = vw.CropBox
            tr = cb.Transform
            xs = []
            ys = []
            for xx in (cb.Min.X, cb.Max.X):
                for yy in (cb.Min.Y, cb.Max.Y):
                    for zz in (cb.Min.Z, cb.Max.Z):
                        pt = tr.OfPoint(DB.XYZ(xx, yy, zz))
                        xs.append(pt.X)
                        ys.append(pt.Y)
            regions.append((min(xs), min(ys), max(xs), max(ys)))
        except Exception:
            pass
    return regions


def _point_in_regions(px, py, regions):
    for (minx, miny, maxx, maxy) in regions:
        if minx <= px <= maxx and miny <= py <= maxy:
            return True
    return False


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


# --- Shared column-engine helpers (used by both the hanger and spool engines;
# these are strategy-neutral, so sharing them does not couple the layouts). ---

def _cur_segs(info):
    """Leader polyline segments for a placed tag (straight, or dogleg)."""
    if info["straight"] is not None:
        return [info["straight"]]
    return [(info["tgt"], info["elbow"]), (info["elbow"], info["head"])]


def _tag_is_unnamed(tag):
    """True if the tag renders blank or as '?' -- i.e. the tagged element has no
    name value. Unnamed hangers (wall brackets, Klo-Shure hangers) come through
    this way and should be skipped rather than tagged with a '?'."""
    try:
        txt = tag.TagText
    except Exception:
        # TagText unavailable (older API): don't skip anything.
        return False
    if txt is None:
        return True
    s = txt.strip()
    # Blank, or made up solely of question marks / whitespace.
    return s == "" or s.replace("?", "").strip() == ""


def _measure_tags(doc, view, tag_symbol, to_tag, skip_unnamed=False):
    """Create a tag per target and measure its head box; return rec dicts.

    Each tag gets a free-end leader (so the head renders where it will finally
    sit), then the head-only box is measured with the leader hidden. When
    `skip_unnamed` is set, a tag that would render blank/'?' is deleted and its
    target dropped (the hanger tool excludes unnamed wall brackets / Klo-Shure
    hangers this way).
    """
    recs = []
    last_wh = [4.5, 0.85]   # fallback size if a tag can't be measured
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
        if skip_unnamed and _tag_is_unnamed(tag):
            doc.Delete(tag.Id)
            continue
        tag.HasLeader = False
        doc.Regenerate()
        bb = tag.get_BoundingBox(view)
        tag.HasLeader = True
        doc.Regenerate()
        if bb is not None:
            w = bb.Max.X - bb.Min.X
            h = bb.Max.Y - bb.Min.Y
            last_wh[0], last_wh[1] = w, h
        else:
            # Tag fell outside the view crop (tight callout views); reuse the
            # last measured size.
            w, h = last_wh[0], last_wh[1]
        recs.append({"el": el, "cx": cx, "cy": cy, "z": z, "ebox": ebox,
                     "tag": tag, "ref": ref, "has_free": has_free,
                     "w": w, "h": h})
    return recs


def _leader_lengths(placed_info):
    """Reported leader length per placed tag (straight or dogleg)."""
    lengths = []
    for info in placed_info:
        if info["straight"] is not None:
            s = info["straight"]
            lengths.append(math.hypot(s[1][0] - s[0][0], s[1][1] - s[0][1]))
        else:
            h = info["head"]
            e = info["elbow"]
            tt = info["tgt"]
            lengths.append(math.hypot(h[0] - e[0], h[1] - e[1])
                           + math.hypot(e[0] - tt[0], e[1] - tt[1]))
    return lengths


def _nudge_off_obstacles(doc, placed_info, obstacle_boxes, tag_margin,
                         max_move=5.0):
    """Slide any tag head that sits on pipework/insulation to the nearest clear
    spot: off every obstacle, off other tags, with a straight leader that
    crosses no other leader or text. The move is bounded (`max_move`) so leaders
    stay short; a genuinely buried tag with no clear spot in reach is left where
    it is rather than exiled on a long leader. Runs a few passes so tags in a
    tight cluster (whose leaders block each other) can clear in sequence once a
    neighbour has moved."""
    for _ in range(4):
        moved = False
        for info in placed_info:
            if not _any_overlap(info["box"], obstacle_boxes, 0.0):
                continue                               # already clear of pipe
            box = info["box"]
            hw = (box[2] - box[0]) / 2.0
            hh = (box[3] - box[1]) / 2.0
            hx0, hy0 = info["head"]
            other_boxes = [o["box"] for o in placed_info if o is not info]
            best = None
            for d in (0.75, 1.25, 1.75, 2.5, 3.25, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0):
                if d > max_move:
                    break
                for ang in range(0, 360, 20):
                    nx = hx0 + d * math.cos(math.radians(ang))
                    ny = hy0 + d * math.sin(math.radians(ang))
                    nb = (nx - hw, ny - hh, nx + hw, ny + hh)
                    if _any_overlap(nb, obstacle_boxes, 0.0):
                        continue
                    if _any_overlap(nb, other_boxes, tag_margin):
                        continue
                    st = _leader_touch(nx, ny, info["ebox"])
                    sseg = (st, (nx, ny))
                    if _leader_hits_obstacle([st, (nx, ny)], obstacle_boxes, st):
                        continue
                    bad = False
                    for o in placed_info:
                        if o is info:
                            continue
                        if _seg_intersects_box(sseg[0], sseg[1], o["box"]):
                            bad = True
                            break
                        for os in _cur_segs(o):
                            if _seg_intersect(sseg[0], sseg[1], os[0], os[1]):
                                bad = True
                                break
                        if bad:
                            break
                    if bad:
                        continue
                    ln = math.hypot(nx - st[0], ny - st[1])
                    if best is None or ln < best[0]:
                        best = (ln, nx, ny, st, nb, sseg)
                if best is not None:
                    break                   # nearest ring with a clear spot wins
            if best is None:
                continue
            ln, nx, ny, st, nb, sseg = best
            tag = info["tag"]
            ref = info["ref"]
            z = info["z"]
            try:
                tag.TagHeadPosition = DB.XYZ(nx, ny, z)
                tag.HasLeader = False
                doc.Regenerate()
                tag.HasLeader = True
                tag.LeaderEndCondition = DB.LeaderEndCondition.Free
                tag.SetLeaderEnd(ref, DB.XYZ(st[0], st[1], z))
                doc.Regenerate()
                info["head"] = (nx, ny)
                info["box"] = nb
                info["tgt"] = st
                info["straight"] = sseg
                moved = True
            except Exception:
                pass
        if not moved:
            break


def _finish_columns(doc, placed_info, obstacle_boxes, tag_margin,
                    obstacle_margin, row_h, avoid_obstacles=True):
    """Post-passes run after a column engine places its tags: migration,
    straight-leader conversion, and crossing-repair by head swap. Mutates
    `placed_info` and returns the recomputed leader lengths."""
    def _leader_len(info):
        total = 0.0
        for s in _cur_segs(info):
            total += math.hypot(s[1][0] - s[0][0], s[1][1] - s[0][1])
        return total

    def _is_bad(idx):
        info = placed_info[idx]
        if _leader_len(info) > 8.0:
            return True
        if (avoid_obstacles and info["straight"] is not None
                and _leader_hits_obstacle(
                    [info["tgt"], info["head"]], obstacle_boxes,
                    info["tgt"])):
            return True
        a_segs = _cur_segs(info)
        for j, other in enumerate(placed_info):
            if j == idx:
                continue
            for s1 in a_segs:
                if _seg_intersects_box(s1[0], s1[1], other["box"]):
                    return True
                for s2 in _cur_segs(other):
                    if _seg_intersect(s1[0], s1[1], s2[0], s2[1]):
                        return True
        return False

    col_xs = sorted(set([round(info["head"][0], 1) for info in placed_info]))
    for i, info in enumerate(placed_info):
        if not _is_bad(i):
            continue
        cx, cy = info["cx"], info["cy"]
        hw = (info["box"][2] - info["box"][0]) / 2.0
        hh = (info["box"][3] - info["box"][1]) / 2.0
        cur_len = _leader_len(info)
        best = None
        cands = sorted(set(col_xs + [round(cx, 1)]), key=lambda x: abs(x - cx))
        for col_x in cands:
            for k in range(0, 9):
                for sgn in ((1, -1) if k else (1,)):
                    hy = cy + sgn * k * row_h
                    nb = (col_x - hw, hy - hh, col_x + hw, hy + hh)
                    if _any_overlap(nb, [o["box"] for o in placed_info
                                         if o is not info], tag_margin):
                        continue
                    st = _leader_touch(col_x, hy, info["ebox"])
                    ln = math.hypot(col_x - st[0], hy - st[1])
                    if ln > max(cur_len, 6.0):
                        continue
                    sseg = (st, (col_x, hy))
                    clean = True
                    for o in placed_info:
                        if o is info:
                            continue
                        if _seg_intersects_box(sseg[0], sseg[1], o["box"]):
                            clean = False
                            break
                        for os in _cur_segs(o):
                            if _seg_intersect(sseg[0], sseg[1], os[0], os[1]):
                                clean = False
                                break
                        if not clean:
                            break
                    if not clean:
                        continue
                    obs = 0.0
                    if avoid_obstacles:
                        if _any_overlap(nb, obstacle_boxes, obstacle_margin):
                            obs += 10.0
                        if _leader_hits_obstacle([st, (col_x, hy)],
                                                 obstacle_boxes, st):
                            obs += 10.0
                    score = obs + ln
                    if best is None or score < best[3]:
                        best = (col_x, hy, st, score, nb, sseg)
            if best is not None:
                break
        if best is not None:
            col_x, hy, st, score, nb, sseg = best
            tag = info["tag"]
            ref = info["ref"]
            z = info["z"]
            try:
                tag.TagHeadPosition = DB.XYZ(col_x, hy, z)
                tag.HasLeader = False
                doc.Regenerate()
                tag.HasLeader = True
                tag.LeaderEndCondition = DB.LeaderEndCondition.Free
                tag.SetLeaderEnd(ref, DB.XYZ(st[0], st[1], z))
                doc.Regenerate()
                info["head"] = (col_x, hy)
                info["box"] = nb
                info["tgt"] = st
                info["straight"] = sseg
            except Exception:
                pass

    for i, info in enumerate(placed_info):
        head = info["head"]
        st = _leader_touch(head[0], head[1], info["ebox"])
        sseg = (st, head)
        if avoid_obstacles and _leader_hits_obstacle(
                [st, head], obstacle_boxes, st):
            continue
        ok = True
        for j, other in enumerate(placed_info):
            if j == i:
                continue
            if _seg_intersects_box(sseg[0], sseg[1], other["box"]):
                ok = False
                break
            crossed = False
            for oseg in _cur_segs(other):
                if _seg_intersect(sseg[0], sseg[1], oseg[0], oseg[1]):
                    crossed = True
                    break
            if crossed:
                ok = False
                break
        if ok:
            tag = info["tag"]
            ref = info["ref"]
            z = info["z"]
            try:
                tag.HasLeader = False
                doc.Regenerate()
                tag.HasLeader = True
                tag.LeaderEndCondition = DB.LeaderEndCondition.Free
                tag.SetLeaderEnd(ref, DB.XYZ(st[0], st[1], z))
                doc.Regenerate()
                info["straight"] = sseg
            except Exception:
                pass

    def _pair_crosses(a, b):
        for s1 in _cur_segs(a):
            for s2 in _cur_segs(b):
                if _seg_intersect(s1[0], s1[1], s2[0], s2[1]):
                    return True
        return False

    def _try_head_swap(i, j):
        A = placed_info[i]
        B = placed_info[j]
        if A["straight"] is None or B["straight"] is None:
            return False

        def _rebox(info, head):
            hw = (info["box"][2] - info["box"][0]) / 2.0
            hh = (info["box"][3] - info["box"][1]) / 2.0
            return (head[0] - hw, head[1] - hh, head[0] + hw, head[1] + hh)

        hA, hB = B["head"], A["head"]
        boxA, boxB = _rebox(A, hA), _rebox(B, hB)
        stA = _leader_touch(hA[0], hA[1], A["ebox"])
        stB = _leader_touch(hB[0], hB[1], B["ebox"])
        segA, segB = (stA, hA), (stB, hB)
        if _box_overlap(boxA, boxB, tag_margin):
            return False
        if _seg_intersect(segA[0], segA[1], segB[0], segB[1]):
            return False
        if (_seg_intersects_box(segA[0], segA[1], boxB)
                or _seg_intersects_box(segB[0], segB[1], boxA)):
            return False
        for k, o in enumerate(placed_info):
            if k == i or k == j:
                continue
            if (_box_overlap(boxA, o["box"], tag_margin)
                    or _box_overlap(boxB, o["box"], tag_margin)):
                return False
            for seg in (segA, segB):
                if _seg_intersects_box(seg[0], seg[1], o["box"]):
                    return False
                for os in _cur_segs(o):
                    if _seg_intersect(seg[0], seg[1], os[0], os[1]):
                        return False
        try:
            for info, head, st, box in ((A, hA, stA, boxA),
                                        (B, hB, stB, boxB)):
                info["tag"].TagHeadPosition = DB.XYZ(head[0], head[1], info["z"])
                if info["ref"] is not None:
                    info["tag"].SetLeaderEnd(
                        info["ref"], DB.XYZ(st[0], st[1], info["z"]))
                info["head"] = head
                info["tgt"] = st
                info["box"] = box
                info["straight"] = (st, head)
            doc.Regenerate()
            return True
        except Exception:
            return False

    for _ in range(4):
        changed = False
        for a in range(len(placed_info)):
            for b in range(a + 1, len(placed_info)):
                if _pair_crosses(placed_info[a], placed_info[b]):
                    if _try_head_swap(a, b):
                        changed = True
        if not changed:
            break

    return _leader_lengths(placed_info)


def _tag_hanger_columns(doc, view, tag_symbol, hangers, obstacle_boxes,
                        to_tag, tags_to_remove, tag_margin,
                        obstacle_margin=0.15):
    """Hanger layout (independent of the spool engine): give each hanger a
    standalone tag with a short leader where there is room, and fall back to
    grouped dogleg columns only in crowded stretches. Shared post-passes then
    shorten long leaders, straighten where clean, and repair crossings."""
    tagged_ids = []
    lengths = []
    placed_boxes = []
    avoid_obstacles = True

    t = DB.Transaction(doc, "Tag Hangers - Columns")
    t.Start()
    try:
        for tid in tags_to_remove:
            doc.Delete(tid)
        if not tag_symbol.IsActive:
            tag_symbol.Activate()
            doc.Regenerate()

        # Skip unnamed hangers (wall brackets, Klo-Shure hangers): they render
        # as a '?' tag, so leave them untagged.
        recs = _measure_tags(doc, view, tag_symbol, to_tag, skip_unnamed=True)
        if not recs:
            t.Commit()
            return tagged_ids, lengths

        row_h = max(r["h"] for r in recs) + 0.35   # consistent row spacing
        clear0 = 2.0                                 # gap from cluster to stack
        placed_info = []                             # for the straight-leader pass

        # --- Standalone-first phase ---
        # Most targets on a run have room for their own tag right beside them.
        # Detect the run direction and try to give each target a standalone tag
        # (short straight leader, offset perpendicular to the run, alternating
        # sides). Only targets that would collide fall through to the grouped
        # column logic below, so the result is a natural mix of spread-out
        # standalone tags with tight groups only where the run is crowded.
        xspread = max(r["cx"] for r in recs) - min(r["cx"] for r in recs)
        yspread = max(r["cy"] for r in recs) - min(r["cy"] for r in recs)
        horizontal = xspread >= yspread

        def _placed_segs(p):
            if p["straight"] is not None:
                return [p["straight"]]
            return [(p["tgt"], p["elbow"]), (p["elbow"], p["head"])]

        def _standalone_try(r, allow_obstacle=False):
            """Find a short standalone spot beside the target.

            Never sits on another tag and never crosses another leader -- those
            are hard rules. Pipework is avoided too on the strict pass; the
            relaxed pass (`allow_obstacle=True`) tolerates a tag over a pipe so
            a buried target still gets a short leader instead of being exiled to
            a far column. Shortest offset wins (k ascending, both sides).
            """
            cx, cy, ebox = r["cx"], r["cy"], r["ebox"]
            hw = r["w"] / 2.0
            hh = r["h"] / 2.0
            base = hh + 0.5
            check_obstacle = avoid_obstacles and not allow_obstacle
            for k in range(1, 6):                    # keep standalone leaders short
                for sgn in (1, -1):
                    off = base + (k - 1) * row_h
                    if horizontal:
                        hx, hy = cx, cy + sgn * off
                    else:
                        hx, hy = cx + sgn * off, cy
                    box = (hx - hw, hy - hh, hx + hw, hy + hh)
                    if _any_overlap(box, [p["box"] for p in placed_info],
                                    tag_margin):
                        continue
                    st = _leader_touch(hx, hy, ebox)
                    sseg = (st, (hx, hy))
                    bad = False
                    for p in placed_info:
                        if _seg_intersects_box(sseg[0], sseg[1], p["box"]):
                            bad = True
                            break
                        for ps in _placed_segs(p):
                            if _seg_intersect(sseg[0], sseg[1], ps[0], ps[1]):
                                bad = True
                                break
                        if bad:
                            break
                    if bad:
                        continue
                    # Strict pass only: also keep the tag and its leader off
                    # unrelated pipework/insulation.
                    if check_obstacle:
                        if _any_overlap(box, obstacle_boxes, obstacle_margin):
                            continue
                        if _leader_hits_obstacle([st, (hx, hy)],
                                                 obstacle_boxes, st):
                            continue
                    return (hx, hy, st, box, sseg)
            return None

        def _place_standalone(r, got):
            hx, hy, st, box, sseg = got
            tag = r["tag"]
            ref = r["ref"]
            z = r["z"]
            tag.TagHeadPosition = DB.XYZ(hx, hy, z)
            if r["has_free"]:
                tag.SetLeaderEnd(ref, DB.XYZ(st[0], st[1], z))
            doc.Regenerate()
            placed_boxes.append(box)
            tagged_ids.append(r["el"].Id.IntegerValue)
            lengths.append(math.hypot(hx - st[0], hy - st[1]))
            placed_info.append({
                "tag": tag, "ref": ref, "z": z, "ebox": r["ebox"],
                "cx": r["cx"], "cy": r["cy"],
                "head": (hx, hy), "elbow": None, "tgt": st, "box": box,
                "straight": sseg,
            })

        # --- Standalone-first phase (hangers) ---
        # Give each target a standalone tag where there is room; only the
        # colliding ones fall through to grouped columns below.
        ordered = sorted(recs,
                         key=lambda r: r["cx"] if horizontal else r["cy"])
        # Pass 1: fully-clear standalone tags.
        leftovers = []
        for r in ordered:
            got = _standalone_try(r, allow_obstacle=False)
            if got is None:
                leftovers.append(r)
            else:
                _place_standalone(r, got)
        # Pass 2: a buried target with no clear spot gets the shortest standalone
        # leader that still avoids other tags and crossings, tolerating a tag
        # over pipework rather than a long, crossing leader.
        if leftovers:
            still = []
            for r in leftovers:
                got = _standalone_try(r, allow_obstacle=True)
                if got is None:
                    still.append(r)
                else:
                    _place_standalone(r, got)
            leftovers = still

        # Cluster the crowded leftovers and stack each in a dogleg column.
        clusters = _cluster_2d(leftovers, dist=6.0) if leftovers else []

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
                # Order the stack by target Y (top first) as a starting point.
                col.sort(key=lambda r: (-r["cy"], -side * r["cx"]))
                n = len(col)
                colw = max(r["w"] for r in col)

                ymean = sum(r["cy"] for r in col) / n
                y_top = ymean + (n - 1) * row_h / 2.0

                # Offset the stack just clear of the local cluster; nudge out
                # further if it lands on an already-placed stack (which would
                # tangle their leaders) or on pipework/insulation. The push is
                # capped: a column must never flee far enough to create a long,
                # crossing leader, so past the cap take the least-bad offset --
                # overlapping another tag stack is weighed far heavier than
                # overlapping pipework, which is tolerable.
                best_off = None
                clear = clear0
                max_clear = clear0 + 6.0
                while clear <= max_clear:
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
                    tag_hit = _any_overlap(col_box, placed_boxes, 0.2)
                    obs_hit = bool(avoid_obstacles
                                   and _any_overlap(col_box, obstacle_boxes,
                                                    obstacle_margin))
                    pen = (2.0 if tag_hit else 0.0) + (1.0 if obs_hit else 0.0)
                    if best_off is None or pen < best_off[0]:
                        best_off = (pen, head_x)
                    if pen == 0.0:
                        break
                    clear += 1.0
                head_x = best_off[1]

                # Each leader gets a horizontal shoulder at its own row: it runs
                # flat from the head out to an elbow on the cluster edge, then
                # angles into the cluster to the target. The flat shoulder sits
                # at a unique row height so it only touches its own text box (no
                # through-text within the stack), and the angled part heads into
                # the pipe cluster where there is no text. Elbows stack in row
                # order and targets in Y order, so the angled fan cannot cross.
                edge_x = cminx if side < 0 else cmaxx
                slot_ys = [y_top - i * row_h for i in range(n)]

                # The tags in a column are interchangeable labels, so permute
                # which spool occupies which row to minimise leader crossings.
                # A valid non-crossing order exists even for tightly bundled
                # spools; the plain Y-sort misses it, so refine by swapping
                # adjacent rows whenever that removes crossings.
                tgts = {id(r): _leader_touch(edge_x, r["cy"], r["ebox"])
                        for r in col}

                def _total_cross(order):
                    total = 0
                    for a in range(len(order)):
                        sa = (tgts[id(order[a])], (edge_x, slot_ys[a]))
                        for b in range(a + 1, len(order)):
                            sb = (tgts[id(order[b])], (edge_x, slot_ys[b]))
                            if _seg_intersect(sa[0], sa[1], sb[0], sb[1]):
                                total += 1
                    return total

                # Reassign spools to rows to minimise the column's total leader
                # crossings. A non-crossing order exists even for tight bundles,
                # so try every pairwise swap (not just adjacent) until no swap
                # lowers the count -- this escapes the local minima that
                # adjacent-only bubbling gets stuck in.
                base = _total_cross(col)
                improved = True
                passes = 0
                while improved and base > 0 and passes < n + 4:
                    improved = False
                    passes += 1
                    for a in range(n - 1):
                        for b in range(a + 1, n):
                            col[a], col[b] = col[b], col[a]
                            new = _total_cross(col)
                            if new < base:
                                base = new
                                improved = True
                            else:
                                col[a], col[b] = col[b], col[a]

                for i, r in enumerate(col):
                    hy = y_top - i * row_h
                    hx = head_x
                    tag = r["tag"]
                    ref = r["ref"]
                    tag.TagHeadPosition = DB.XYZ(hx, hy, r["z"])
                    elbow = (edge_x, hy)                 # shoulder end, at row Y
                    tgt = _leader_touch(edge_x, r["cy"], r["ebox"])
                    if r["has_free"]:
                        tag.SetLeaderEnd(ref, DB.XYZ(tgt[0], tgt[1], r["z"]))
                        try:
                            tag.SetLeaderElbow(ref, DB.XYZ(elbow[0], elbow[1], r["z"]))
                        except Exception:
                            pass
                    hw = r["w"] / 2.0
                    hh = r["h"] / 2.0
                    box = (hx - hw, hy - hh, hx + hw, hy + hh)
                    placed_boxes.append(box)
                    tagged_ids.append(r["el"].Id.IntegerValue)
                    lengths.append(math.hypot(hx - elbow[0], hy - elbow[1])
                                   + math.hypot(elbow[0] - tgt[0], elbow[1] - tgt[1]))
                    # Record for the straight-leader / migration passes below.
                    placed_info.append({
                        "tag": tag, "ref": ref, "z": r["z"], "ebox": r["ebox"],
                        "cx": r["cx"], "cy": r["cy"],
                        "head": (hx, hy), "elbow": elbow, "tgt": tgt, "box": box,
                        "straight": None,
                    })

        _finish_columns(doc, placed_info, obstacle_boxes,
                        tag_margin, obstacle_margin, row_h)
        # Final pass: slide any head still sitting on pipework to the nearest
        # clear spot. Allow a longer reach (~9 ft) so tags stuck in a tight
        # cluster can route out to a non-crossing spot.
        _nudge_off_obstacles(doc, placed_info, obstacle_boxes, tag_margin,
                             max_move=9.0)
        lengths[:] = _leader_lengths(placed_info)

        doc.Regenerate()
        t.Commit()
    except Exception:
        t.RollBack()
        raise
    return tagged_ids, lengths


def _tag_spool_columns(doc, view, tag_symbol, hangers, obstacle_boxes,
                       to_tag, tags_to_remove, tag_margin,
                       obstacle_margin=0.15):
    """Spool layout (independent of the hanger engine): tight columns beside
    each run, each head at its own spool's decluttered Y with a straight,
    nearly horizontal fanned leader -- the hand-tagging style. Crowded runs are
    split left/right; the shared post-passes then repair any crossings."""
    avoid_obstacles = True
    tagged_ids = []
    lengths = []
    placed_boxes = []

    t = DB.Transaction(doc, "Tag Spools - Columns")
    t.Start()
    try:
        for tid in tags_to_remove:
            doc.Delete(tid)
        if not tag_symbol.IsActive:
            tag_symbol.Activate()
            doc.Regenerate()

        recs = _measure_tags(doc, view, tag_symbol, to_tag)
        if not recs:
            t.Commit()
            return tagged_ids, lengths

        row_h = max(r["h"] for r in recs) + 0.35
        placed_info = []

        # Cluster spools into local groups; each side of a group gets a tight
        # straight-fan column.
        clusters = _cluster_2d(recs, dist=3.5)
        for cluster in clusters:
            cluster.sort(key=lambda r: r["cx"])
            mid = (len(cluster) + 1) // 2
            sides = [(-1, cluster[:mid]), (1, cluster[mid:])]
            for side, col in sides:
                if not col:
                    continue
                col.sort(key=lambda r: (-r["cy"], -side * r["cx"]))
                colw = max(r["w"] for r in col)
                # Manual spool style: fixed column X ~1 ft off the run's near
                # edge; each head at its own spool's Y (decluttered, order
                # preserved); straight near-horizontal leaders.
                mrow = max(r["h"] for r in col) + 0.2
                head_ys = _spread_y([r["cy"] for r in col], mrow)
                ytop, ybot = max(head_ys), min(head_ys)
                cl = min(r["ebox"][0] for r in col)
                cr = max(r["ebox"][2] for r in col)
                gap = 1.0
                best = None
                push = 0.0
                while push <= 6.0:
                    if side < 0:
                        hx = cl - gap - colw / 2.0 - push
                    else:
                        hx = cr + gap + colw / 2.0 + push
                    cbox = (hx - colw / 2.0, ybot - mrow,
                            hx + colw / 2.0, ytop + mrow)
                    tag_hit = _any_overlap(cbox, placed_boxes, 0.2)
                    obs_hit = bool(avoid_obstacles and _any_overlap(
                        cbox, obstacle_boxes, obstacle_margin))
                    pen = (2.0 if tag_hit else 0.0) + (1.0 if obs_hit else 0.0)
                    if best is None or pen < best[0]:
                        best = (pen, hx)
                    if pen == 0.0:
                        break
                    push += 1.0
                head_x = best[1]
                for i, r in enumerate(col):
                    hy = head_ys[i]
                    tag = r["tag"]
                    ref = r["ref"]
                    tgt = _leader_touch(head_x, r["cy"], r["ebox"])
                    tag.TagHeadPosition = DB.XYZ(head_x, hy, r["z"])
                    if r["has_free"]:
                        tag.SetLeaderEnd(ref, DB.XYZ(tgt[0], tgt[1], r["z"]))
                    hw = r["w"] / 2.0
                    hh = r["h"] / 2.0
                    box = (head_x - hw, hy - hh, head_x + hw, hy + hh)
                    placed_boxes.append(box)
                    tagged_ids.append(r["el"].Id.IntegerValue)
                    lengths.append(math.hypot(head_x - tgt[0], hy - tgt[1]))
                    placed_info.append({
                        "tag": tag, "ref": ref, "z": r["z"],
                        "ebox": r["ebox"], "cx": r["cx"], "cy": r["cy"],
                        "head": (head_x, hy), "elbow": None, "tgt": tgt,
                        "box": box, "straight": (tgt, (head_x, hy)),
                    })

        lengths[:] = _finish_columns(doc, placed_info, obstacle_boxes,
                                     tag_margin, obstacle_margin, row_h)

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
    exclude_callouts=True,
    avoid_obstacles=False,
    kind="hanger",
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

        # Callout regions on this view: their contents are detailed (and tagged)
        # in the callout view, so skip any target whose centre falls inside one.
        callout_regions = _callout_regions(doc, view) if exclude_callouts else []
        in_callout_count = 0

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
                    if callout_regions and _point_in_regions(cx, cy, callout_regions):
                        in_callout_count += 1
                        continue
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

        # Ganged-column layout: dispatch to the independent per-tool engine.
        if layout == "ganged":
            to_tag = [h for h in hangers
                      if retag_existing
                      or h[0].Id.IntegerValue not in already_tagged_ids]
            skipped_ids = [h[0].Id.IntegerValue for h in hangers
                           if not retag_existing
                           and h[0].Id.IntegerValue in already_tagged_ids]
            engine = (_tag_spool_columns if kind == "spool"
                      else _tag_hanger_columns)
            g_tagged, g_lengths = engine(
                doc, view, tag_symbol, hangers, obstacle_boxes,
                to_tag, tags_to_remove, tag_margin, obstacle_margin)
            result = {
                "status": "success",
                "view": view.Name,
                "hanger_category": hanger_category,
                "tagged_count": len(g_tagged),
                "skipped_already_tagged": skipped_ids,
                "could_not_clear": [],
                "excluded_in_callout": in_callout_count,
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

                if bb is None:
                    # Tag outside the view crop (e.g. tight callout view); use a
                    # nominal box so placement can still proceed.
                    dxmin, dymin, dxmax, dymax = -2.25, -0.42, 2.25, 0.42
                else:
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
            "excluded_in_callout": in_callout_count,
        }
        if lengths:
            result["leader_length_ft"] = {
                "min": round(min(lengths), 2),
                "max": round(max(lengths), 2),
                "avg": round(sum(lengths) / len(lengths), 2),
            }
        return result

    except Exception as e:
        import traceback
        logger.error("Error in tag_hangers_no_overlap: %s", e)
        return {"status": "error", "message": "Failed to tag hangers: {}".format(str(e)),
                "traceback": traceback.format_exc()}


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
        # Route to the independent spool engine (straight-fan columns).
        kind="spool",
    )


def tag_hangers(doc, view_name=None, retag_existing=False):
    """Tag every hanger in a view using the ganged-column layout (the manual
    drafting style): tags stacked in columns flanking the run, horizontal
    shoulder leaders that touch each hanger, with crossings minimised."""
    return tag_hangers_no_overlap(
        doc,
        view_name=view_name,
        hanger_category="MEP Fabrication Hangers",
        tag_family_category="MEP Fabrication Hanger Tags",
        # Keep hanger tags off the pipe runs and insulation.
        avoid_categories=["MEP Fabrication Pipework", "Insulation"],
        retag_existing=retag_existing,
        layout="ganged",
        # Route to the independent hanger engine (standalone-first + doglegs).
        kind="hanger",
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
                layout=data.get("layout", "ganged"),
            )

            status_code = 200 if result.get("status") == "success" else 500
            return routes.make_response(data=result, status=status_code)

        except Exception as e:
            logger.error("Error in tag_hangers route: %s", e)
            return routes.make_response(data={"error": str(e)}, status=500)
