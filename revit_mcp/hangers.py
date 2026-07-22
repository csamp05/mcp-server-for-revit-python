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


def _candidate_ok(cand_box, leader_start, leader_end, placed, obstacles,
                  tag_margin, obstacle_margin):
    for p in placed:
        if _box_overlap(cand_box, p["box"], tag_margin):
            return False
        # No two leaders may cross each other.
        if _seg_intersect(leader_start, leader_end, p["hanger_pt"], p["head_pt"]):
            return False
        # This leader must not run through an already-placed tag's text box,
        # and no placed leader may run through this candidate's text box.
        if _seg_intersects_box(leader_start, leader_end, p["box"]):
            return False
        if _seg_intersects_box(p["hanger_pt"], p["head_pt"], cand_box):
            return False
    for ob in obstacles:
        if _box_overlap(cand_box, ob, obstacle_margin):
            return False
    return True


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
                    hangers.append((el, cx, cy, z))
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

        hangers.sort(key=lambda r: (r[1], r[2]))

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

            for el, cx, cy, z in hangers:
                if el.Id.IntegerValue in already_tagged_ids and not retag_existing:
                    skipped_ids.append(el.Id.IntegerValue)
                    continue

                ref = DB.Reference(el)
                # Create the tag WITHOUT a leader so its bounding box is the
                # head only. Position it a default distance up to start.
                init_hx, init_hy = cx, cy + radii[0]
                tag = DB.IndependentTag.Create(
                    doc, tag_symbol.Id, view.Id, ref, False,
                    DB.TagOrientation.Horizontal, DB.XYZ(init_hx, init_hy, z),
                )
                # A single regenerate so the head box is available to measure.
                doc.Regenerate()
                bb = tag.get_BoundingBox(view)

                # Record the head box extents relative to the head position.
                # The box translates rigidly with the head, so we can evaluate
                # every candidate position with pure geometry (no regenerate).
                dxmin = bb.Min.X - init_hx
                dymin = bb.Min.Y - init_hy
                dxmax = bb.Max.X - init_hx
                dymax = bb.Max.Y - init_hy

                leader_start = (cx, cy)
                placed_ok = False
                chosen = None
                for radius in radii:
                    for ang in angles_deg:
                        rad = math.radians(ang)
                        hx = cx + radius * math.cos(rad)
                        hy = cy + radius * math.sin(rad)
                        cand_box = (hx + dxmin, hy + dymin, hx + dxmax, hy + dymax)
                        leader_end = (hx, hy)
                        if _candidate_ok(
                            cand_box, leader_start, leader_end, placed,
                            obstacle_boxes, tag_margin, obstacle_margin,
                        ):
                            chosen = (cand_box, leader_end)
                            placed_ok = True
                            break
                    if placed_ok:
                        break

                if not placed_ok:
                    # Fall back to the last (farthest) candidate tried.
                    cand_box = (init_hx + dxmin, init_hy + dymin,
                                init_hx + dxmax, init_hy + dymax)
                    chosen = (cand_box, (init_hx, init_hy))

                cand_box, leader_end = chosen
                # Apply the chosen head position and a free-end leader once.
                tag.TagHeadPosition = DB.XYZ(leader_end[0], leader_end[1], z)
                if tag.CanLeaderEndConditionBeAssigned(DB.LeaderEndCondition.Free):
                    tag.HasLeader = True
                    tag.LeaderEndCondition = DB.LeaderEndCondition.Free
                    tag.SetLeaderEnd(ref, DB.XYZ(cx, cy, z))
                doc.Regenerate()

                placed.append({
                    "id": tag.Id.IntegerValue, "box": cand_box,
                    "hanger_pt": leader_start, "head_pt": leader_end,
                })
                if placed_ok:
                    tagged_ids.append(el.Id.IntegerValue)
                    lengths.append(math.hypot(
                        leader_end[0] - cx, leader_end[1] - cy
                    ))
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
