#! python3
# -*- coding: UTF-8 -*-
"""Tag all hangers in the active view.

Places an IndependentTag on every hanger, keeping tag heads clear of each
other, leader tails from crossing, and tags off of piping/insulation.
Hangers that already have a tag in this view are left alone.
"""

from pyrevit import revit, script
from revit_mcp.hangers import tag_hangers_no_overlap

output = script.get_output()
doc = revit.doc

result = tag_hangers_no_overlap(doc, view_name=doc.ActiveView.Name)

if result.get("status") == "success":
    output.print_md(
        "**Tagged {} hangers** in view `{}`".format(
            result.get("tagged_count", 0), result.get("view")
        )
    )
    skipped = result.get("skipped_already_tagged")
    if skipped:
        output.print_md("Skipped {} already-tagged hangers.".format(len(skipped)))
    failed = result.get("could_not_clear")
    if failed:
        output.print_md(
            "**Warning:** could not fully clear overlap for {} hangers: {}".format(
                len(failed), failed
            )
        )
    lengths = result.get("leader_length_ft")
    if lengths:
        output.print_md(
            "Leader length (ft): min {min}, max {max}, avg {avg}".format(**lengths)
        )
else:
    output.print_md("**Error:** {}".format(result.get("message", "Unknown error")))
