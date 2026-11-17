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


# ------------------------------------------------------------------------------ every box at least once
"""
The "all boxes" test: a fixed number of products (default 70) enters on S1 one after another, every
belt and all 12 arms work at once, and the test PASSES when
  - each of the 29 boxes of the factory has at least one product: the trash bin and the recycling bin
    of every station (24) and the 5 dock boxes, and
  - every product has ended in a box; once all boxes are filled the remaining products all go to the
    docks (Dock 1 ... Dock 5), none is left on a belt, in a gripper or on the floor.

Every station decides with AllBoxesPolicy (need-based), so the result does not depend on the product
mix and a missed product is simply taken on the next try.
"""

DOCKS: List[str] = [f"dock_{c}" for c, (_s, _e, _l, dock) in L.STRAIGHTS.items() if dock]
DOCK_TITLES: Dict[str, str] = {f"dock_{c}": dock.title() for c, (_s, _e, _l, dock) in L.STRAIGHTS.items() if dock}


def all_boxes() -> List[str]:
    """The 29 boxes: trash and recycling bin of every station (A1 ... A12), then the 5 docks."""
    return [P.bin_name(a, k) for a in P.STATIONS for k in ("trash", "recycling")] + DOCKS


def downstream_convs(conv: str) -> List[str]:
    """This conveyor and every conveyor a product on it can still reach."""
    out, todo = [], [conv]
    while todo:
        c = todo.pop()
        if c in out:
            continue
        out.append(c)
        todo += [P.station(a)["place_conv"] for a in P.STATIONS if P.station(a)["pick_conv"] == c]
    return out


def downstream_boxes(conv: str) -> List[str]:
    """The boxes a product on this conveyor can still end in."""
    convs = downstream_convs(conv)
    out = [P.bin_name(a, k) for a in P.STATIONS if P.station(a)["pick_conv"] in convs
           for k in ("trash", "recycling")]
    return out + downstream_docks(conv)


def downstream_docks(conv: str) -> List[str]:
    return [f"dock_{c}" for c in downstream_convs(conv) if f"dock_{c}" in DOCKS]


class AllBoxesPolicy:
    """decide(ctx) for every station of the all-boxes test.

    Phase 1, while some box is still empty, at a station in this order:
      1. the product's label suits one of the station's own empty bins (recyclable -> recycling bin,
         a TRASH label -> trash bin): into that bin          (so the bins are filled by vision)
      2. the boxes further down the line still need products: bent2
      3. one of the station's own bins is still empty: into that bin
      4. otherwise skip (on a straight it waits and is judged again; on a ring it rides on to the
         next arm or comes round again)
      "Need" = empty boxes downstream minus the products already on their way there (on those belts or
      in those arms' grippers), so the line is not flooded and the branches share the products.
    Phase 2, every box has a product: no more bins, every product goes on towards the docks. Where a
    ring has two arms, the first one takes a product only while its docks have no more products per
    dock (counting the ones on their way) than the other arm's; the last arm takes all the others, so
    the remaining products are spread over Dock 1 ... Dock 5 and nothing circles forever.
    """

    def __init__(self, sim, trash_labels=("sneaker_pair", "packet_pouch", "unknown"), by: str = "label"):
        from factory_bringup.product_catalog import RECYCLABLE
        self.sim = sim
        self.trash_labels = set(trash_labels)
        self.recyclable = set(RECYCLABLE)
        self.by = by                                    # "label" (detector) or "truth" (simulation type)
        convs = list(L.STRAIGHTS) + list(L.CIRCLES)
        self._convs = {c: set(downstream_convs(c)) for c in convs}
        self._boxes = {c: downstream_boxes(c) for c in convs}
        self._docks = {c: downstream_docks(c) for c in convs}
        self._order = {c: pickers(c) for c in L.CIRCLES}
        self._lock = threading.Lock()

    def filled(self, box: str) -> bool:
        """The box has a product (or one is falling into it right now)."""
        with self.sim.lock:
            if self.sim.counts.get(box, 0) > 0:
                return True
            return any(p.state == "falling" and p.where == box for p in self.sim.products.values())

    def all_filled(self) -> bool:
        return all(self.filled(b) for b in all_boxes())

    def coming(self, conv: str) -> int:
        """Products on their way into the part of the line after `conv` (on its belts or in its grippers)."""
        convs = self._convs[conv]
        with self.sim.lock:
            n = sum(1 for p in self.sim.products.values() if p.state == "belt" and p.conv in convs)
            n += sum(1 for a in self.sim.arms.values() if a.carrying and P.station(a.name)["pick_conv"] in convs)
        return n

    def need(self, conv: str) -> int:
        """Empty boxes downstream of `conv` minus the products already heading there."""
        return sum(1 for b in self._boxes[conv] if not self.filled(b)) - self.coming(conv)

    def dock_load(self, conv: str) -> float:
        """Products per dock after `conv`: already in its docks plus on their way."""
        docks = self._docks[conv]
        with self.sim.lock:
            n = sum(self.sim.counts.get(d, 0) for d in docks)
        return (n + self.coming(conv)) / max(1, len(docks))

    def _phase2(self, arm: str) -> str:
        st = P.station(arm)
        order = self._order.get(st["pick_conv"])
        if not order or len(order) < 2 or arm == order[-1]:
            return "bent2"                              # straight, or the last arm of a ring: take it
        mine = self.dock_load(st["place_conv"])
        others = [self.dock_load(P.station(a)["place_conv"]) for a in order if a != arm]
        return "bent2" if mine <= min(others) else "skip"

    def __call__(self, ctx: PickContext) -> str:
        arm = ctx.station
        label = ctx.product_type if self.by == "truth" else ctx.label
        own = {k: P.bin_name(arm, k) for k in ("trash", "recycling")}
        with self._lock:
            if self.all_filled():
                return self._phase2(arm)
            if label in self.trash_labels and not self.filled(own["trash"]):
                return "trash"
            if label in self.recyclable and not self.filled(own["recycling"]):
                return "recycling"
            if self.need(P.station(arm)["place_conv"]) > 0:
                return "bent2"
            for kind in ("recycling", "trash"):
                if not self.filled(own[kind]):
                    return kind
            return "skip"


class S1Feeder:
    """Puts `items` on S1 one after another, the next one as soon as the start of the belt is free
    (min_gap m). A spawn Gazebo refuses is logged and tried again; it never stops the test."""

    def __init__(self, lab, items: List[str], conv: str = "S1", min_gap: float = 1.2):
        self.lab, self.items, self.conv, self.min_gap = lab, list(items), conv, min_gap
        self.names: List[str] = []
        self.failures = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="all_boxes_feeder", daemon=True)

    @property
    def done(self) -> bool:
        return len(self.names) >= len(self.items)

    def start(self):
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()

    def _loop(self):
        sim, s0 = self.lab.sim, 0.4
        while not self._stop.is_set() and not self.done:
            free = all(abs(p.s - s0) > self.min_gap for p in sim.on_conveyor(self.conv))
            if not free or not sim.running_belts.get(self.conv, True):
                self._stop.wait(0.3)
                continue
            ptype = self.items[len(self.names)]
            try:
                self.names += sim.spawn_row([ptype], self.conv, 1.0, s0)
                self.failures = 0
                if len(self.names) % 10 == 0 or self.done:
                    self.lab.log(f"feeder: {len(self.names)}/{len(self.items)} products on {self.conv}")
            except Exception as exc:
                self.failures += 1
                self.lab.log(f"feeder: spawn {len(self.names) + 1} ({ptype}) failed: {exc}")
                if self.failures == 3:
                    self.lab.log("feeder: Gazebo keeps refusing spawns; check with  ./run_vision_lab.sh check")
                self._stop.wait(min(10.0, 2.0 * self.failures))


def product_mix(n: int, items: Optional[List[str]] = None, seed: Optional[int] = 7) -> List[str]:
    """n product types: `items` repeated in a shuffled order (seeded: the same run every time)."""
    import random
    pool = list(items or PRODUCT_TYPES)
    rng = random.Random(seed)
    out: List[str] = []
    while len(out) < n:
        rng.shuffle(pool)
        out += pool
    return out[:n]


@dataclass
class AllBoxesResult:
    boxes: Dict[str, int]                  # box -> products in it at the end
    deliveries: Dict[str, Dict[str, int]]  # station -> destination -> delivered products
    missed: Dict[str, int]                 # station -> missed grasps
    conveyors: Dict[str, int]              # conveyor -> products that rode it
    spawned: int
    wanted: int
    left: Dict[str, int]                   # products not in a box at the end: where they are -> how many
    seconds: float
    timeout: float
    covered_after: Optional[float] = None  # seconds until every box had a product

    @property
    def empty(self) -> List[str]:
        return [b for b, n in self.boxes.items() if n == 0]

    @property
    def idle_arms(self) -> List[str]:
        return [a for a in P.STATIONS if not sum(self.deliveries.get(a, {}).values())]

    @property
    def idle_conveyors(self) -> List[str]:
        return [c for c, n in self.conveyors.items() if n == 0]

    @property
    def in_docks(self) -> int:
        return sum(self.boxes[d] for d in DOCKS)

    @property
    def passed(self) -> bool:
        return (not self.empty and not self.idle_arms and not self.idle_conveyors
                and self.spawned == self.wanted and not self.left)


def run_all_boxes(lab, count: int = 70, items: Optional[List[str]] = None, timeout: float = 3600.0,
                  decide_by: str = "label", belt_speed: Optional[float] = None, seed: Optional[int] = 7,
                  trash_labels=("sneaker_pair", "packet_pouch", "unknown")) -> AllBoxesResult:
    """`count` products enter on S1 one after another (types: `items` shuffled, default every catalog
    type); all 13 belts and all 12 arms work until every box has a product and every product has
    arrived in a box (the rest in the docks), or until timeout."""
    sim = lab.sim
    mix = product_mix(count, items, seed)
    bad = sorted({i for i in mix if i not in PRODUCT_TYPES})
    if bad:
        raise ValueError(f"unknown product types {bad}")
    lab._stop.clear()
    lab.feed = False                                # this test feeds S1 itself, a fixed number of products
    sim.clear()
    with sim.lock:
        sim.counts.clear()
        for a in sim.arms.values():
            a.carrying = None
    for conv in list(sim.running_belts):
        sim.running_belts[conv] = True              # every bent runs
    for arm in P.STATIONS:
        lab.home(arm, duration=1.0)

    policy = AllBoxesPolicy(sim, trash_labels, decide_by)
    boxes = all_boxes()
    feeder = S1Feeder(lab, mix)
    t0 = time.time()
    state = {"empty": None, "covered": None, "boxed": -1}

    def until() -> bool:
        empty = [b for b in boxes if not policy.filled(b)]
        if len(empty) != state["empty"]:
            state["empty"] = len(empty)
            lab.log(f"all boxes: {len(boxes) - len(empty)}/{len(boxes)} filled after {time.time() - t0:.0f} s"
                    + (f"; still empty: {', '.join(empty)}" if empty and len(empty) <= 8 else ""))
            if not empty and state["covered"] is None:
                state["covered"] = time.time() - t0
                lab.log("all boxes have a product: the remaining products go to the docks")
        with sim.lock:
            names = list(feeder.names)
            boxed = sum(1 for n in names if n in sim.products and sim.products[n].state == "bin"
                        and sim.products[n].where != "floor")
            lost = sum(1 for n in names if n in sim.products and sim.products[n].where == "floor")
            busy = any(a.carrying for a in sim.arms.values())
        if boxed != state["boxed"] and boxed % 10 == 0:
            state["boxed"] = boxed
            lab.log(f"all boxes: {boxed}/{len(mix)} products in boxes ({sum(sim.counts.get(d, 0) for d in DOCKS)} in docks)")
        return not empty and feeder.done and not busy and boxed + lost >= len(mix)

    speed = lab.belt_speed
    if belt_speed:
        lab.belt_speed = sim.speed = belt_speed
    feeder.start()
    try:
        lab.run(policy, stations="all", timeout=timeout, until=until)
    finally:
        feeder.stop()
        lab.belt_speed = sim.speed = speed
    time.sleep(1.0)                                 # let the last drop land

    deliveries: Dict[str, Dict[str, int]] = {a: {} for a in P.STATIONS}
    missed: Dict[str, int] = {}
    rode: Dict[str, set] = {c: set() for c in list(L.STRAIGHTS) + list(L.CIRCLES)}
    names = set(feeder.names)
    for r in lab.results:
        if r.model not in names:
            continue
        if r.status == "delivered":
            d = deliveries[r.station]
            d[r.destination] = d.get(r.destination, 0) + 1
            st = P.station(r.station)
            rode[st["pick_conv"]].add(r.model)
            if r.destination == "bent2":
                rode[st["place_conv"]].add(r.model)
        elif r.status == "missed":
            missed[r.station] = missed.get(r.station, 0) + 1
    left: Dict[str, int] = {}
    with sim.lock:
        counts = {b: sim.counts.get(b, 0) for b in boxes}
        for n in names:
            p = sim.products.get(n)
            if p is None:
                where = "removed"
            elif p.state == "bin" and p.where in counts:
                continue
            elif p.state == "belt":
                rode[p.conv].add(n)
                where = f"on {p.conv}"
            else:
                where = p.where or p.state
            left[where] = left.get(where, 0) + 1
    return AllBoxesResult(counts, deliveries, missed, {c: len(s) for c, s in rode.items()}, len(feeder.names),
                          len(mix), left, time.time() - t0, timeout, state["covered"])


def format_all_boxes(res: AllBoxesResult) -> str:
    n_full = len(res.boxes) - len(res.empty)
    covered = f", all boxes filled after {res.covered_after:.0f} s" if res.covered_after is not None else ""
    lines = [f"[{'PASS' if res.passed else 'FAIL'}] all_boxes  ({res.seconds:.0f} s{covered})",
             f"   products: {res.spawned}/{res.wanted} fed on S1, {sum(res.boxes.values())} in boxes "
             f"({res.in_docks} in the docks), {sum(res.left.values())} elsewhere",
             f"   {n_full}/{len(res.boxes)} boxes got a product, "
             f"{len(P.STATIONS) - len(res.idle_arms)}/{len(P.STATIONS)} arms delivered, "
             f"{len(res.conveyors) - len(res.idle_conveyors)}/{len(res.conveyors)} bents carried products",
             "", "   station  trash bin  recycling bin  bent2  missed   picks from -> places on"]
    for a in P.STATIONS:
        st = P.station(a)
        d = res.deliveries.get(a, {})
        t, r = res.boxes[P.bin_name(a, "trash")], res.boxes[P.bin_name(a, "recycling")]
        lines.append(f"   {'ok  ' if t and r else 'FAIL'} {a:4s} {t:6d}     {r:9d}      {d.get('bent2', 0):5d}  "
                     f"{res.missed.get(a, 0):6d}   {st['pick_conv']} -> {st['place_conv']}")
    lines.append("")
    for b in DOCKS:
        lines.append(f"   {'ok  ' if res.boxes[b] else 'FAIL'} {DOCK_TITLES[b]:20s} ({b}) {res.boxes[b]:4d} products")
    lines += ["", "   bents (products that rode them): "
              + ", ".join(f"{c} {n}{'' if n else ' FAIL'}" for c, n in res.conveyors.items())]
    if res.empty:
        lines.append(f"   FAIL empty boxes: {', '.join(res.empty)}")
    if res.left:
        lines.append("   FAIL products not in a box: " + ", ".join(f"{k} {v}" for k, v in sorted(res.left.items())))
    if res.spawned < res.wanted:
        lines.append(f"   FAIL only {res.spawned} of {res.wanted} products could be spawned")
    if res.idle_arms:
        lines.append(f"   FAIL arms that delivered nothing: {', '.join(res.idle_arms)}")
    if not res.passed and res.seconds >= res.timeout:
        lines.append(f"   (time limit {res.timeout:.0f} s reached)")
    return "\n".join(lines)
