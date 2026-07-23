#! python3
# -*- coding: UTF-8 -*-
"""Tag all hangers in the active view.

Places an IndependentTag on every hanger, keeping tag heads clear of each
other, leader tails from crossing, and tags off of piping/insulation.
Hangers that already have a tag in this view are left alone.
"""

import os
import sys
import traceback

from pyrevit import revit, script

output = script.get_output()
# Force the output window to appear so results/errors are always visible.
output.show()
output.print_md("### Tag Hangers")

try:
    # The revit_mcp package lives at the extension root. When this button runs
    # under the CPython engine, that root is not on sys.path (unlike
    # startup.py), so add it: script -> pushbutton -> panel -> tab -> root.
    _ext_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    )
    if _ext_root not in sys.path:
        sys.path.append(_ext_root)
    output.print_md("Extension root: `{}`".format(_ext_root))

    # pyRevit's CPython engine is persistent across clicks, so drop any cached
    # copy of the module to guarantee the latest code runs every time.
    for _m in list(sys.modules.keys()):
        if _m == "revit_mcp.hangers" or _m.startswith("revit_mcp.hangers."):
            del sys.modules[_m]

    from revit_mcp.hangers import tag_hangers

    doc = revit.doc
    view = doc.ActiveView
    output.print_md("Active view: `{}`".format(view.Name))

    result = tag_hangers(doc, view_name=view.Name)

    if result.get("status") == "success":
        output.print_md(
            "**Tagged {} hangers** in view `{}`".format(
                result.get("tagged_count", 0), result.get("view")
            )
        )
        skipped = result.get("skipped_already_tagged")
        if skipped:
            output.print_md(
                "Skipped {} already-tagged hangers.".format(len(skipped))
            )
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

except Exception:
    output.print_md("**Button crashed — full traceback below:**")
    output.print_md("```\n{}\n```".format(traceback.format_exc()))
