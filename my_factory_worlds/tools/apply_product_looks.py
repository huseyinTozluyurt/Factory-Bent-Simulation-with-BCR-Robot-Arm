#!/usr/bin/env python3
"""
Give the products that are written into the world files (product_milk_carton, pickup_box, ...)
the realistic look of factory_bringup/product_models.py. Only their <visual> elements change;
pose, plugin, inertia and the collision stay as they are. Safe to run again after a look changed.

  python3 apply_product_looks.py                 # every world in ../worlds
  python3 apply_product_looks.py ../worlds/factory_conveyor.sdf
"""

import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "factory_bringup"))

from factory_bringup.product_catalog import product_visuals, type_of  # noqa: E402

MODEL = re.compile(r'(<model name="([^"]+)">)(.*?)(</model>)', re.S)


def apply(path: str) -> int:
    text = open(path).read()
    changed = 0

    def fix(m):
        nonlocal changed
        name, body = m.group(2), m.group(3)
        ptype = type_of(name)
        if not ptype or "<visual" not in body:
            return m.group(0)
        start, end = body.index("<visual"), body.rindex("</visual>") + len("</visual>")
        indent = re.search(r"\n([ \t]*)<visual", body)
        indent = indent.group(1) if indent else "        "
        visuals = product_visuals(ptype).replace("<visual ", f"\n{indent}<visual ").lstrip()
        changed += 1
        return m.group(1) + body[:start] + visuals + body[end:] + m.group(4)

    text = MODEL.sub(fix, text)
    if changed:
        open(path, "w").write(text)
    return changed


if __name__ == "__main__":
    files = sys.argv[1:] or sorted(os.path.join(HERE, "..", "worlds", f) for f in os.listdir(os.path.join(HERE, "..", "worlds"))
                                   if f.endswith(".sdf"))
    for f in files:
        print(f"{os.path.basename(f)}: {apply(f)} products")
