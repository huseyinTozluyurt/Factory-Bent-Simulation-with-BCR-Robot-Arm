"""
Starter lab scripts for a world (no Qt, no ROS imports).

    from factory_bringup.lab_ide.generator import STARTERS, generate
    text = generate("pipeline-tree", name="my_sorting", starter="world")

Starters:
    world             generated from the world itself: every station, arm, bent, bin and dock of the world
                      written into the script, with a plan per arm you edit (default)
    template:<T>      one of the vision_lab templates (templates/<T>.py), with WORLD set to the world
    lab:<file>        a copy of an existing lab file (labs/lab_*.py), with WORLD set to the world

Every generated file is a normal lab file (LAB_TITLE, WORLD, DETECTOR, DETECTOR_OPTIONS, ARM_PARAMS,
main(lab)): run it with  ./run_vision_lab.sh run <file> --world <world>.
"""

import datetime
import os
import re
from typing import Dict, List, Optional, Tuple

from factory_bringup.lab_ide import world_info

TEMPLATE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "vision_lab", "templates")

ITEMS = ["cereal_box", "glass_jar", "sneaker_pair", "coffee_can", "shoe_box", "basketball", "pizza_box",
         "packet_pouch", "tool_case", "toy_box", "water_jug", "milk_carton", "paint_can", "shipping_box_large"]
TRASH = ["sneaker_pair", "packet_pouch", "unknown"]


def slug(name: str) -> str:
    s = re.sub(r"[^0-9a-zA-Z_]+", "_", name.strip()).strip("_").lower()
    if s.startswith("lab_"):
        s = s[4:]
    if not s:
        raise ValueError("empty lab name")
    return s


def lab_file_name(name: str) -> str:
    """lab_<slug>.py: the lab finds it by name, and its results go to <folder>/results/<slug>/."""
    return f"lab_{slug(name)}.py"


def templates() -> Dict[str, str]:
    """vision_lab template name -> its one-line description."""
    out = {}
    if os.path.isdir(TEMPLATE_DIR):
        for f in sorted(os.listdir(TEMPLATE_DIR)):
            if f.endswith(".py") and not f.startswith("_"):
                with open(os.path.join(TEMPLATE_DIR, f)) as fh:
                    first = fh.readline()
                out[f[:-3]] = first.split("TEMPLATE:", 1)[1].strip() if "TEMPLATE:" in first else ""
    return out


def example_labs(labs_dir: Optional[str]) -> List[Tuple[str, str]]:
    """[(file name, title)] of the lab files in a labs folder."""
    out = []
    if labs_dir and os.path.isdir(labs_dir):
        for f in sorted(os.listdir(labs_dir)):
            if f.startswith("lab_") and f.endswith(".py"):
                title = ""
                try:
                    with open(os.path.join(labs_dir, f)) as fh:
                        for line in fh:
                            if line.startswith("LAB_TITLE"):
                                title = line.split("=", 1)[1].strip().strip("'\"")
                                break
                except OSError:
                    pass
                out.append((f, title))
    return out


def starters(labs_dir: Optional[str] = None) -> List[Tuple[str, str]]:
    """[(starter id, label)] for the IDE's starter list."""
    out = [("world", "Generated for this world: every station, arm, bin and dock, with a plan per arm")]
    out += [(f"template:{k}", f"Template '{k}': {v}") for k, v in templates().items()]
    out += [(f"lab:{f}", f"Copy of {f}: {t}") for f, t in example_labs(labs_dir)]
    return out


def _set_world(text: str, key: str) -> str:
    if re.search(r"^WORLD = .*$", text, flags=re.M):
        return re.sub(r"^WORLD = .*$", f"WORLD = {key!r}", text, count=1, flags=re.M)
    return text


def _fill(text: str, name: str, title: str, template: str) -> str:
    if text.startswith("# TEMPLATE:"):
        text = text.split("\n", 1)[1]
    text = text.replace('"{{TITLE}}"', repr(title))
    subs = {"{{NAME}}": name, "{{TITLE}}": title, "{{DATE}}": datetime.date.today().isoformat(),
            "{{TEMPLATE}}": template}
    for k, v in subs.items():
        text = text.replace(k, v)
    return text


def _world_comment(info: Dict) -> str:
    lines = [f"# ---- World {info['key']}: {info['title']}",
             f"# Gazebo world {info['gazebo_world']} ({info['sdf']})"]
    if info["kind"] == "pipeline":
        lines += ["#"] + ["#   " + ln for ln in info["flow"].splitlines()] + ["#"]
        for a in info["arms"]:
            lines.append(f"#   {a['name']:3s} picks on {a['pick_conv']:3s} -> places on {a['place_conv']:3s} "
                         f"rule {a['rule']:10s} bins {', '.join(a['bins'])}")
        if info["docks"]:
            lines.append("#   docks: " + ", ".join(f"{d['name']} ({d['title']})" for d in info["docks"]))
    else:
        lines.append("#   " + info["flow"])
        lines.append("#   destinations: " + ", ".join(info["destinations"]))
    return "\n".join(lines) + "\n"


def _insert_after_docstring(text: str, block: str) -> str:
    m = re.match(r'\s*(""".*?"""|\'\'\'.*?\'\'\')\n', text, flags=re.S)
    if not m:
        return block + "\n" + text
    return text[:m.end()] + block + text[m.end():]


# ------------------------------------------------------------------ the generated starters
def _run_lines(key: str, file_name: str, pipeline: bool) -> str:
    sim = f"./run_vision_lab.sh sim --world {key}" + (" --drawn-cameras" if pipeline else "")
    out = (f"Run (Lab IDE: Run, or in two terminals):\n"
           f"      {sim}          (terminal 1)\n"
           f"      ./run_vision_lab.sh run {file_name} --world {key}   (terminal 2)\n")
    if pipeline:
        out += f"Without Gazebo:  ./run_vision_lab.sh run {file_name} --world {key} --fake\n"
    return out


def _pipeline_starter(info: Dict, name: str, title: str, file_name: str) -> str:
    key = info["key"]
    tree = info.get("layout") == "tree"
    share = [a["name"] for a in info["arms"] if a["rule"] == "share"]
    st_lines, plan_lines = [], []
    for a in info["arms"]:
        bins = list(a["bins"])
        st_lines.append(f"    {a['name']!r}: dict(pick={a['pick_conv']!r}, place={a['place_conv']!r}, "
                        f"rule={a['rule']!r}, bins={bins!r}),")
        plan_lines.append(f"    {(repr(a['name']) + ':'):6s} \"rules\",    # rule {a['rule']}: {a['pick_conv']} -> {a['place_conv']}")
    docks = ", ".join(f"{d['name']!r}: {d['title']!r}" for d in info["docks"])
    flow = "\n".join("    " + ln for ln in info["flow"].splitlines())
    return f'''"""
LAB {name}: {title}
Created {datetime.date.today().isoformat()} by the Lab IDE for the world '{key}'.

{info['title']}  (Gazebo world {info['gazebo_world']}, {info['sdf']})

{flow}

Products enter on {info['inbound']} (FEED below). Every arm station has a pick bent, a place bent (bent 2),
a trash bin and a recycling bin; the output lines end over a dock box. decide(ctx) runs for every
product that reaches an arm (ctx.station = which arm, ctx.label = what YOUR detector saw,
ctx.product_type = the simulation's truth) and returns where it goes:
    "bent2"      onto the arm's place bent (towards the next arm / a dock)
    "trash"      into the arm's trash bin        "recycling"  into the arm's recycling bin
    "skip"       let it pass (the next arm, or the next lap of a ring, gets it)

PLAN says what each arm does: "rules" = its rule from the layout ({", ".join(sorted({a["rule"] for a in info["arms"]}))}),
or a list of steps done in turn, e.g. ["trash", "recycling", "bent2"] (a step advances when the arm
delivered a product).

{_run_lines(key, file_name, True)}The log (lab.log) and report.txt go to results/{name}/<time>/ next to this file.
"""
import os
import threading
import time

from factory_bringup.vision_lab.pipeline_testing import S1Feeder, product_mix

LAB_TITLE = {title!r}
WORLD = {key!r}
DETECTOR = "opencv_contours"            # opencv_contours, opencv_color, opencv_bgsub, yolo, file.py:Class
DETECTOR_OPTIONS = {{"classify": True}}
ARM_PARAMS = {{}}                         # e.g. {{"locate": "vision"}} = camera only (default "assist")

COUNT = 20                              # products fed on {info['inbound']}
TIMEOUT = 1800                          # seconds
FEED = ITEMS = {ITEMS!r}
TRASH = {{{', '.join(repr(t) for t in TRASH)}}}   # labels every arm throws into its own trash bin
SCHEDULE = "classic"                    # "rendezvous": belts never stop (needs the rendezvous scheduler, lab 13)

# ---- the stations of this world (from the layout; for reference)
STATIONS = {{
{chr(10).join(st_lines)}
}}
DOCKS = {{{docks}}}

# ---- what each arm does: "rules" or a list of steps (bent2, trash, recycling, skip)
PLAN = {{
{chr(10).join(plan_lines)}
}}
# arms whose step list takes turns with the other arms of their pick bent (lab.share_turn)
SHARE = {share!r}
ACTIVE = "all"                          # or e.g. ["A1", "A2", "A3"]


class Plan:
    """decide(ctx) from PLAN; a step advances when that arm delivered its product."""

    def __init__(self):
        self.step, self.picks, self.lock = {{}}, {{}}, threading.Lock()

    def steps(self, arm):
        p = PLAN.get(arm, "rules")
        return p if isinstance(p, (list, tuple)) else None

    def __call__(self, ctx):
        if ctx.label in TRASH:
            return "trash"
        steps = self.steps(ctx.station)
        if steps is None:
            return ctx.lab.layout_rule(ctx)              # the arm's rule from the layout
        with self.lock:
            if ctx.station in SHARE and not ctx.lab.share_turn(ctx.lab, ctx.station, self.picks):
                return "skip"
            return steps[self.step.get(ctx.station, 0) % len(steps)]

    def on_result(self, r):
        if not r.station or r.status != "delivered":
            return
        with self.lock:
            self.picks[r.station] = self.picks.get(r.station, 0) + 1
            if self.steps(r.station) is not None:
                self.step[r.station] = self.step.get(r.station, 0) + 1


def main(lab):
    if not hasattr(lab, "all_stations"):
        raise SystemExit("This lab is for the {key} world: run it with --world {key} (or --fake)")
    if SCHEDULE != "classic" and hasattr(lab, "set_scheduler"):
        lab.set_scheduler(SCHEDULE)
    plan = Plan()
    lab.on_result(plan.on_result)
    lab.on_result(lambda r: r.status != "skipped" and print(
        f"    {{r.station:3s}} #{{r.number}}: {{r.label or '?':18s}} -> {{r.destination:9s}} {{r.status:9s}} "
        f"(truth {{r.product_type}})", flush=True))
    stations = lab.all_stations if ACTIVE == "all" else [s.upper() for s in ACTIVE]
    for a in stations:
        p = PLAN.get(a, "rules")
        print(f"  {{a:3s}} {{STATIONS[a]['pick']}} -> {{STATIONS[a]['place']:3s}} "
              f"{{p if isinstance(p, str) else ' -> '.join(p)}}", flush=True)

    lab.feed = False                                     # products come from FEED, not the automatic feeder
    feeder = S1Feeder(lab, product_mix(COUNT, FEED)).start()
    sim = lab.sim
    t0 = time.time()

    def finished():
        with sim.lock:
            busy = [n for n in feeder.names if n in sim.products
                    and sim.products[n].state in ("belt", "carried", "falling")]
        return feeder.done and not busy
    try:
        lab.run(plan, stations=stations, timeout=TIMEOUT, until=finished)
    finally:
        feeder.stop()

    counts = sim.summary()
    lines = [f"{{len(feeder.names)}} products fed in {{time.time() - t0:.0f}} s",
             "where they are: " + ", ".join(f"{{k}} {{v}}" for k, v in counts.items()),
             "docks: " + ", ".join(f"{{DOCKS[d]}} {{counts.get(d, 0)}}" for d in DOCKS)]
    for a in stations:
        lines.append(f"  {{a:3s}} picks {{plan.picks.get(a, 0):3d}}  "
                     + ", ".join(f"{{b}} {{counts.get(b, 0)}}" for b in STATIONS[a]["bins"]))
    text = "\\n".join(lines) + "\\n\\n" + lab.report()
    print("\\n" + "=" * 78 + "\\n" + text, flush=True)
    os.makedirs(lab.output_dir, exist_ok=True)
    with open(os.path.join(lab.output_dir, "report.txt"), "w") as f:
        f.write(text + "\\n")
    for a in stations[:3]:
        lab.snapshot(f"{{a}}.png", station=a)
    print(f"report: {{os.path.join(lab.output_dir, 'report.txt')}}")
'''


def _single_starter(info: Dict, name: str, title: str, file_name: str) -> str:
    key = info["key"]
    has2 = info["has_bent2"]
    seq = (["bent2", "recycling", "trash"] if has2 else ["recycling", "trash"])
    by_label = ('{"sneaker_pair": "trash", "packet_pouch": "trash", "cereal_box": "recycling", '
                f'"*": "{"bent2" if has2 else "recycling"}"}}')
    bins = ", ".join(f"{k} at ({v[0]:.2f}, {v[1]:.2f})" for k, v in info["bins"].items())
    bents = ", ".join(f"{b['name']} {b['shape']}" for b in info["bents"])
    return f'''"""
LAB {name}: {title}
Created {datetime.date.today().isoformat()} by the Lab IDE for the world '{key}'.

{info['title']}  (Gazebo world {info['gazebo_world']}, {info['sdf']})
    bents: {bents}
    one BCR arm; it catches products at the catch station on bent 1 ({info['arms'][0]['catch_station'][0]:.2f}, 0)
    bins: {bins}
    {info['flow']}

MODE "sequence": the arm takes COUNT products and puts them where SEQUENCE says, in turn.
MODE "labels":   the arm sorts by what YOUR detector sees (BY_LABEL: label -> destination, "*" = the rest).
Destinations in this world: {", ".join(info["destinations"])}{"" if has2 else " (no bent 2 here)"}.

{_run_lines(key, file_name, False)}The log and the snapshot go to results/{name}/<time>/ next to this file.
"""
LAB_TITLE = {title!r}
WORLD = {key!r}
DETECTOR = "opencv_contours"            # opencv_contours, opencv_color, opencv_bgsub, yolo, file.py:Class
DETECTOR_OPTIONS = {{}}
ARM_PARAMS = {{"motion_type": "auto"}}    # auto, cycle, random, standard, mirror, high_carry, low_fast ...

MODE = "sequence"                       # "sequence" or "labels"
COUNT = 6
SEQUENCE = {seq!r}
BY_LABEL = {by_label}
TIMEOUT = 900


def main(lab):
    print(f"World {{lab.world.key}}: destinations {{lab.world.destinations}}", flush=True)
    lab.on_result(lambda r: print(f"  #{{r.number}}: {{r.label or '?'}} -> {{r.destination}} ({{r.status}}, "
                                  f"truth {{r.product_type}})", flush=True))
    if MODE == "labels":
        lab.run(BY_LABEL, count=COUNT, timeout=TIMEOUT)
    else:
        for i in range(COUNT):
            where = SEQUENCE[i % len(SEQUENCE)]
            if where == "bent2" and not lab.world.has_bent2:
                where = "recycling"
            result = lab.transfer(where)
            print(f"{{i + 1}}/{{COUNT}} -> {{where}}: {{result}}", flush=True)
    lab.snapshot()
    print(lab.report())
'''


def generate(world: str, name: str, starter: str = "world", title: Optional[str] = None,
             labs_dir: Optional[str] = None, info: Optional[Dict] = None) -> str:
    """The text of a new lab file for `world` (see the module doc for `starter`)."""
    s = slug(name)
    title = title or s.replace("_", " ").capitalize()
    info = info or world_info.describe(world)
    file_name = lab_file_name(s)
    if starter == "world":
        if info["kind"] == "pipeline":
            return _pipeline_starter(info, s, title, file_name)
        return _single_starter(info, s, title, file_name)
    if starter.startswith("template:"):
        t = starter.split(":", 1)[1]
        path = os.path.join(TEMPLATE_DIR, t + ".py")
        if not os.path.isfile(path):
            raise ValueError(f"unknown template '{t}'")
        text = _fill(open(path).read(), s, title, t)
    elif starter.startswith("lab:"):
        f = starter.split(":", 1)[1]
        path = os.path.join(labs_dir or "", f)
        if not os.path.isfile(path):
            raise ValueError(f"lab file not found: {path}")
        text = open(path).read()
        text = re.sub(r"^LAB_TITLE = .*$", f"LAB_TITLE = {title!r}", text, count=1, flags=re.M)
        text = f"# Copied from {f} by the Lab IDE on {datetime.date.today().isoformat()}.\n" + text
    else:
        raise ValueError(f"unknown starter '{starter}'")
    return _insert_after_docstring(_set_world(text, world), "\n" + _world_comment(info))
