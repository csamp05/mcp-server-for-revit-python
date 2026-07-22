#! python3
# -*- coding: UTF-8 -*-
"""Tag all spools (fabrication assemblies) in the active view.

Places an Assembly Tag on every spool using the same rules as the hanger
tool: tag heads never overlap, leaders never cross or run through another
tag's text, and tags stay off piping, insulation and hangers. Spools that
already have a tag in this view are left alone.
"""

import os
import sys
import traceback

from pyrevit import revit, script

output = script.get_output()
output.show()
output.print_md("### Tag Spools")

try:
    # script -> pushbutton -> panel -> tab -> extension root (holds revit_mcp).
    _ext_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    )
    if _ext_root not in sys.path:
        sys.path.append(_ext_root)

    # pyRevit's CPython engine persists across clicks; drop any cached copy so
    # the latest code always runs.
    for _m in list(sys.modules.keys()):
        if _m == "revit_mcp.hangers" or _m.startswith("revit_mcp.hangers."):
            del sys.modules[_m]

    from revit_mcp.hangers import tag_spools

    doc = revit.doc
    view = doc.ActiveView
    output.print_md("Active view: `{}`".format(view.Name))

    result = tag_spools(doc, view_name=view.Name)

    if result.get("status") == "success":
        output.print_md(
            "**Tagged {} spools** in view `{}`".format(
                result.get("tagged_count", 0), result.get("view")
            )
        )
        skipped = result.get("skipped_already_tagged")
        if skipped:
            output.print_md(
                "Skipped {} already-tagged spools.".format(len(skipped))
            )
        failed = result.get("could_not_clear")
        if failed:
            output.print_md(
                "**Note:** {} spools placed with an unavoidable compromise "
                "(tight cluster): {}".format(len(failed), failed)
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
