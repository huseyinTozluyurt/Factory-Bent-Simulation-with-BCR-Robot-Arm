"""
Lab files (no ROS imports): every experiment of the vision lab is one Python file in a labs folder.

    labs/lab_<name>.py
        LAB_TITLE = "..."            shown by `vision_lab list`
        WORLD = None                 world it is meant for (None = any / whatever runs)
        DETECTOR = "opencv_contours" detector name, "file.py:Class", or a Detector class in the file
        DETECTOR_OPTIONS = {}
        ARM_PARAMS = {}              vision_pick_place parameters for this lab
        def main(lab): ...           the experiment: detection + arm calls

Create one from a template, list them, and run one by name:

    vision_lab new color_sorting --template sort --world dual
    vision_lab list
    vision_lab run color_sorting               (finds labs/lab_color_sorting.py)
    vision_lab offline --lab color_sorting --synthetic 10   (its detector, no simulation)

The labs folder is $VISION_LAB_DIR (run_vision_lab.sh sets it to <project>/labs), else ./labs.
"""

import ast
import datetime
import os
import re
from typing import Dict, List, Optional, Tuple

TEMPLATE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "templates")
LAB_DIR_ENV = "VISION_LAB_DIR"
PREFIX = "lab_"


def templates() -> Dict[str, str]:
    """template name -> its description (the '# TEMPLATE: ...' first line of templates/<name>.py)."""
    out = {}
    for f in sorted(os.listdir(TEMPLATE_DIR)):
        if f.endswith(".py") and not f.startswith("_"):
            with open(os.path.join(TEMPLATE_DIR, f)) as fh:
                first = fh.readline()
            out[f[:-3]] = first.split("TEMPLATE:", 1)[1].strip() if "TEMPLATE:" in first else ""
    return out


def lab_dir(path: Optional[str] = None) -> str:
    return os.path.abspath(os.path.expanduser(path or os.environ.get(LAB_DIR_ENV) or "labs"))


def _meta(path: str) -> Dict:
    """LAB_TITLE / WORLD / DETECTOR of a lab file, read without running it."""
    meta = {"title": "", "world": None, "detector": None}
    try:
        tree = ast.parse(open(path).read())
    except (OSError, SyntaxError) as exc:
        meta["title"] = f"(cannot read: {exc})"
        return meta
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            key = {"LAB_TITLE": "title", "WORLD": "world", "DETECTOR": "detector"}.get(node.targets[0].id)
            if key:
                try:
                    meta[key] = ast.literal_eval(node.value)
                except ValueError:
                    meta[key] = ast.unparse(node.value)
    if not meta["title"]:
        meta["title"] = (ast.get_docstring(tree) or "").strip().split("\n")[0]
    return meta


def list_labs(path: Optional[str] = None) -> List[Tuple[str, str, Dict]]:
    """[(name, file, meta)] of the lab files in the labs folder."""
    d = lab_dir(path)
    if not os.path.isdir(d):
        return []
    out = []
    for f in sorted(os.listdir(d)):
        if f.startswith(PREFIX) and f.endswith(".py"):
            out.append((f[len(PREFIX):-3], os.path.join(d, f), _meta(os.path.join(d, f))))
    return out


def find_lab(name: str, path: Optional[str] = None) -> Optional[str]:
    """A lab by file path, name ('color_sorting'), file name ('lab_color_sorting') or number prefix ('03')."""
    p = os.path.expanduser(name)
    if os.path.isfile(p):
        return os.path.abspath(p)
    d = lab_dir(path)
    stem = name[:-3] if name.endswith(".py") else name
    for cand in (stem, PREFIX + stem):
        f = os.path.join(d, cand + ".py")
        if os.path.isfile(f):
            return f
    hits = [f for n, f, _ in list_labs(path) if n.startswith(stem)]
    return hits[0] if len(hits) == 1 else None


def slug(name: str) -> str:
    s = re.sub(r"[^0-9a-zA-Z_]+", "_", name.strip()).strip("_").lower()
    if s.startswith(PREFIX):
        s = s[len(PREFIX):]
    if not s:
        raise ValueError("empty lab name")
    return s


def create_lab(name: str, template: str = "transfer", path: Optional[str] = None, world: Optional[str] = None,
               detector: Optional[str] = None, title: Optional[str] = None, force: bool = False) -> str:
    """Write labs/lab_<name>.py from templates/<template>.py; returns the file path."""
    tpls = templates()
    if template not in tpls:
        raise ValueError(f"unknown template '{template}'; templates: {', '.join(tpls)}")
    s = slug(name)
    d = lab_dir(path)
    os.makedirs(d, exist_ok=True)
    out = os.path.join(d, f"{PREFIX}{s}.py")
    if os.path.exists(out) and not force:
        raise FileExistsError(f"{out} exists (use --force to overwrite)")
    text = open(os.path.join(TEMPLATE_DIR, template + ".py")).read()
    subs = {
        "{{NAME}}": s,
        "{{TITLE}}": title or s.replace("_", " ").capitalize(),
        "{{DATE}}": datetime.date.today().isoformat(),
        "{{TEMPLATE}}": template,
    }
    if text.startswith("# TEMPLATE:"):
        text = text.split("\n", 1)[1]
    text = text.replace('"{{TITLE}}"', repr(subs["{{TITLE}}"]))
    for k, v in subs.items():
        text = text.replace(k, v)
    if world is not None:
        text = re.sub(r"^WORLD = .*$", f"WORLD = {world!r}", text, count=1, flags=re.M)
    if detector is not None:
        text = re.sub(r"^DETECTOR = .*$", f"DETECTOR = {detector!r}", text, count=1, flags=re.M)
    with open(out, "w") as f:
        f.write(text)
    return out
