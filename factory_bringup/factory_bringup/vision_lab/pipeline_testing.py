"""
Test cases for the pipeline world (no ROS imports; runs on a PipelineLab, in Gazebo or with --fake).

A test case puts known products on a conveyor, lets some arm stations work with a routing table, and
checks where every product really ends up against where it should end up:

    PipelineCase(
        name="chain_to_large_dock",
        items=["cereal_box", "shoe_box"],                 # spawned on `start` (default S1)
        routing={"A1": "bent2", "A3": "bent2", "A5": "bent2", "A9": "bent2"},
    )

routing: station -> what that arm does with each product it sees, by the detector's label:
    "bent2" / "trash" / "recycling" / "skip"          the same for every product
    {"glass_jar": "recycling", "*": "bent2"}          a table: label -> destination, "*" = the rest
    "rules"                                           the station's rule from pipeline_layout.py
    a function f(ctx) -> destination
Stations not in routing do nothing (products pass them or wait at the end of their belt).

The expected route and final place of every product are worked out from the same routing, applied
to the product's TRUE type (and true size for size rules): conveyor by conveyor, the stations in
the order a product meets them. A product passes when it ends there through exactly those stations.
When it fails, the report says whether the detector's label was wrong or the arm / belt failed.

Final places: "trash_bin_A5", "recycling_bin_A1", "dock_S7" (the dock past an output line), or a
conveyor name ("C1" = still riding ring C1 because no working arm takes it, "S2" = waiting at the
end of S2).
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple, Union

from factory_bringup import pipeline_layout as L
from factory_bringup.product_catalog import PRODUCT_TYPES
from factory_bringup.vision_lab import pipeline_world as P
from factory_bringup.vision_lab.common import PickContext, normalize_destination
from factory_bringup.vision_lab.detectors.base import ObjectDetection

Routing = Union[str, Dict[str, str], Callable[[PickContext], str]]


@dataclass
class PipelineCase:
    name: str
    items: List[str]
    routing: Dict[str, Routing]
    description: str = ""
    start: str = "S1"                      # conveyor the products are put on
    spacing: float = 1.4
    timeout: float = 300.0                 # seconds for the whole case
    belt_speed: Optional[float] = None     # m/s for this case (None = the lab's)
    expect: Dict[str, str] = field(default_factory=dict)   # optional fixed final places by product type


@dataclass
class ProductCheck:
    name: str
    product_type: str
    expected_place: str
    expected_route: List[str]
    place: str
    route: List[str]
    labels: List[str]                      # what the detector called it at each station that decided
    ok: bool
    why: str = ""


@dataclass
class CaseResult:
    case: PipelineCase
    checks: List[ProductCheck]
    seconds: float
    lost: List[str]

    @property
    def passed(self) -> bool:
        return not self.lost and all(c.ok for c in self.checks)


# ------------------------------------------------------------------------------------------ layout
def pickers(conv: str) -> List[str]:
    """The stations that pick from a conveyor, in the order a product on it meets them."""
    arms = [a for a in P.STATIONS if P.station(a)["pick_conv"] == conv]
    if not P.is_ring(conv) or len(arms) < 2:
        return arms
    cx, cy, _ = L.CIRCLES[conv]
    ang = lambda p: math.atan2(p[1] - cy, p[0] - cx) % (2 * math.pi)  # noqa: E731
    feed = [P.station(a)["place"] for a in P.STATIONS if P.station(a)["place_conv"] == conv]
    f = ang(feed[0]) if feed else 0.0
    d = P.ring_dir(conv)
    return sorted(arms, key=lambda a: ((ang(P.station(a)["pick"]) - f) * d) % (2 * math.pi))


def end_place(conv: str) -> str:
    """Where a product ends on a conveyor no working arm takes it from."""
    if not P.is_ring(conv) and L.STRAIGHTS[conv][3]:
        return f"dock_{conv}"
    return conv


def truth_detection(ptype: str, x: float = 0.0, y: float = 0.0) -> ObjectDetection:
    """What a perfect detector would report for a product type (label and footprint)."""
    sx, sy, _ = PRODUCT_TYPES[ptype]["size"]
    return ObjectDetection(u=0.0, v=0.0, label=ptype, score=1.0, x=x, y=y, size_x=sx, size_y=sy, bent=1)


def make_decider(rule: Routing, rule_fn: Callable[[PickContext], str]) -> Callable[[PickContext], str]:
    if callable(rule):
        return rule
    if isinstance(rule, dict):
        table = {k: normalize_destination(v) for k, v in rule.items()}
        return lambda ctx: table.get(ctx.label, table.get("*", "skip"))
    if rule in ("rules", "rule", "layout"):
        return rule_fn
    fixed = normalize_destination(rule)
    return lambda ctx: fixed


def expected_route(ptype: str, case: PipelineCase, deciders: Dict[str, Callable], lab=None) -> Tuple[List[str], str]:
    """(stations that deliver it, final place) for a product of this TRUE type."""
    if ptype in case.expect:
        return [], case.expect[ptype]
    conv, route = case.start, []
    for _ in range(20):
        moved = False
        for arm in pickers(conv):
            if arm not in deciders:
                continue
            st = P.station(arm)
            ctx = PickContext(number=0, detection=truth_detection(ptype, *st["pick"]), model=None,
                              product_type=ptype, world=P.station_profile(arm), lab=lab, station=arm)
            dest = normalize_destination(deciders[arm](ctx))
            if dest == "skip":
                continue
            route.append(arm)
            if dest in ("trash", "recycling"):
                return route, P.bin_name(arm, dest)
            conv, moved = st["place_conv"], True
            break
        if not moved:
            return route, end_place(conv)
    return route, conv


# ------------------------------------------------------------------------------------------ runner
def product_place(p) -> str:
    if p.state in ("bin", "falling"):
        return p.where
    if p.state == "belt":
        return p.conv
    return p.state                                    # carried


def run_case(lab, case: PipelineCase, decide_by: str = "label") -> CaseResult:
    """Run one case on a PipelineLab. decide_by="truth" routes on the true type (tests only the arms
    and belts), "label" (default) on what the detector sees (tests vision and motion together)."""
    sim = lab.sim
    lab._stop.clear()                                 # (a previous run may have stopped the arms)
    sim.clear()
    with sim.lock:
        sim.counts.clear()
        for a in sim.arms.values():
            a.carrying = None
    for conv in list(sim.running_belts):
        sim.running_belts[conv] = True
    for arm in case.routing:
        lab.home(arm, duration=1.0)

    deciders = {a.upper(): make_decider(r, lab.layout_rule) for a, r in case.routing.items()}
    lab.log(f"case {case.name}: {len(case.items)} products on {case.start}, stations "
            f"{', '.join(deciders)}")
    lab.feed = False
    names = sim.spawn_row(list(case.items), case.start, case.spacing)
    expected = {}
    for n in names:
        ptype = sim.products[n].ptype
        expected[n] = expected_route(ptype, case, deciders, lab)

    def decide(ctx: PickContext) -> str:
        if decide_by == "truth":
            ctx = PickContext(number=ctx.number, detection=truth_detection(ctx.product_type, ctx.detection.x,
                              ctx.detection.y), model=ctx.model, product_type=ctx.product_type,
                              world=ctx.world, lab=ctx.lab, station=ctx.station)
        return deciders[ctx.station](ctx)

    def finished() -> bool:
        """Every product is where it should be, or can no longer move (in a bin or a dock)."""
        with sim.lock:
            for n in names:
                p = sim.products.get(n)
                if p is None:
                    continue
                if p.state == "falling" or p.state == "carried":
                    return False
                if p.state != "bin" and product_place(p) != expected[n][1]:
                    return False
            return all(a.carrying is None for a in sim.arms.values())

    t0 = time.time()
    start = len(lab.results)
    speed = lab.belt_speed
    if case.belt_speed:
        lab.belt_speed = sim.speed = case.belt_speed
    try:
        lab.run(decide, stations=list(deciders), timeout=case.timeout, until=finished)
    finally:
        lab.belt_speed = sim.speed = speed
    time.sleep(1.0)                                   # let the last drop land
    results = [r for r in lab.results[start:] if r.model in expected]

    checks, lost = [], []
    for n in names:
        p = sim.products.get(n)
        if p is None:
            lost.append(n)
            continue
        route = [r.station for r in results if r.model == n and r.status == "delivered"]
        labels = [f"{r.station}:{r.label}" for r in results if r.model == n and r.status in ("delivered", "skipped")]
        exp_route, exp_place = expected[n]
        place = product_place(p)
        ok = place == exp_place and (not exp_route or route == exp_route)
        why = ""
        if not ok:
            wrong = [lbl for lbl in labels if lbl.split(":", 1)[1] != p.ptype]
            missed = [r.station for r in results if r.model == n and r.status == "missed"]
            if wrong:
                why = f"detector called it {', '.join(wrong)}"
            elif missed:
                why = f"missed grasp at {', '.join(missed)}"
            elif place == "floor":
                why = "dropped on the floor"
            elif route != exp_route:
                i = next((k for k in range(max(len(route), len(exp_route)))
                          if k >= len(route) or k >= len(exp_route) or route[k] != exp_route[k]), 0)
                if i < len(exp_route):
                    why = (f"{exp_route[i]} did not take it" + (f" ({route[i]} did)" if i < len(route) else "")
                           + ": its camera did not catch it in time, or decide() said skip")
                else:
                    why = f"{route[i]} took it, but no arm should have"
            else:
                why = "did not arrive in time (timeout)"
        checks.append(ProductCheck(n, p.ptype, exp_place, exp_route, place, route, labels, ok, why))
    return CaseResult(case, checks, time.time() - t0, lost)


def format_result(res: CaseResult) -> str:
    c = res.case
    lines = [f"[{'PASS' if res.passed else 'FAIL'}] {c.name}  ({res.seconds:.0f} s)  {c.description}"]
    for k in res.checks:
        route = " -> ".join(k.route) or "-"
        exp = " -> ".join(k.expected_route) or "-"
        lines.append(f"   {'ok  ' if k.ok else 'FAIL'} {k.product_type:20s} route {route:24s} at {k.place:18s}"
                     + ("" if k.ok else f" expected {exp} at {k.expected_place}: {k.why}"))
    for n in res.lost:
        lines.append(f"   FAIL {n} disappeared")
    return "\n".join(lines)


def run_cases(lab, cases: List[PipelineCase], only: Optional[List[str]] = None, decide_by: str = "label",
              stop_on_fail: bool = False) -> List[CaseResult]:
    picked = [c for c in cases if not only or c.name in only or any(c.name.startswith(o) for o in only)]
    out = []
    for i, c in enumerate(picked, 1):
        print(f"\n--- case {i}/{len(picked)}: {c.name}", flush=True)
        if c.description:
            print(f"    {c.description}", flush=True)
        res = run_case(lab, c, decide_by)
        print(format_result(res), flush=True)
        out.append(res)
        if stop_on_fail and not res.passed:
            break
    return out


def summary(results: List[CaseResult]) -> str:
    n = sum(r.passed for r in results)
    lines = [f"{n}/{len(results)} cases passed"]
    for r in results:
        bad = sum(not c.ok for c in r.checks) + len(r.lost)
        lines.append(f"  {'PASS' if r.passed else 'FAIL'}  {r.case.name:28s} {len(r.checks) - bad}/{len(r.checks) + len(r.lost)} "
                     f"products ok  {r.seconds:.0f} s")
    return "\n".join(lines)
