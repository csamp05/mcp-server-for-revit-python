# -*- coding: utf-8 -*-
"""Hanger tagging tools"""

from mcp.server.fastmcp import Context
from typing import List, Optional
from .utils import format_response


def register_hanger_tools(mcp, revit_get, revit_post, revit_image=None):
    """Register hanger tools with the MCP server."""

    @mcp.tool()
    async def tag_hangers(
        view_name: Optional[str] = None,
        hanger_category: str = "MEP Fabrication Hangers",
        tag_family_category: str = "MEP Fabrication Hanger Tags",
        avoid_categories: Optional[List[str]] = None,
        retag_existing: bool = False,
        tag_margin: float = 0.3,
        obstacle_margin: float = 0.15,
        ctx: Context = None,
    ) -> str:
        """
        Tag hangers in a Revit view with non-overlapping tags and non-crossing leaders.

        Each hanger gets an IndependentTag with a short, free-end leader. Tags are
        placed by searching increasing leader lengths in multiple directions
        (up, diagonals, sides, down) and picking the shortest one that clears:
        - every other tag's head (no touching/overlapping tag boxes)
        - every other tag's leader (no crossing leader tails)
        - any element in `avoid_categories` (e.g. piping, insulation), if given

        Args:
            view_name: View to tag in. Defaults to the currently active view.
            hanger_category: Category name of the elements to tag (default: "MEP Fabrication Hangers").
            tag_family_category: Category name of the tag family to use (default: "MEP Fabrication Hanger Tags").
            avoid_categories: Category names tags must steer clear of, e.g.
                ["MEP Fabrication Pipework", "Insulation"]. Pass an empty list
                to allow tags to land anywhere (e.g. on top of equipment).
                Defaults to piping and insulation if omitted.
            retag_existing: If True, remove and replace any existing tags on
                these hangers in the view. If False (default), hangers that
                already have a tag in the view are left untouched.
            tag_margin: Minimum clearance in feet required between tag heads (default: 0.3).
            obstacle_margin: Minimum clearance in feet required between a tag
                head and an avoided element (default: 0.15).
            ctx: MCP context for logging

        Returns:
            Summary of the tagging operation: how many were tagged, skipped,
            or could not be cleanly placed, plus leader length statistics.
        """
        try:
            data = {
                "hanger_category": hanger_category,
                "tag_family_category": tag_family_category,
                "retag_existing": retag_existing,
                "tag_margin": tag_margin,
                "obstacle_margin": obstacle_margin,
            }
            if view_name:
                data["view_name"] = view_name
            if avoid_categories is not None:
                data["avoid_categories"] = avoid_categories

            if ctx:
                await ctx.info(
                    "Tagging {} in view {}".format(
                        hanger_category, view_name or "(active view)"
                    )
                )
            response = await revit_post("/tag_hangers/", data, ctx, timeout=120.0)
            return format_response(response)

        except Exception as e:
            error_msg = "Error tagging hangers: {}".format(str(e))
            if ctx:
                await ctx.error(error_msg)
            return error_msg
