"""
Rendezvous pick-and-place for the pipeline worlds (pipeline and pipeline-tree): the belts never stop
for an arm. Instead of holding a product and pausing its belt while the arm comes down, every arm

  1. predicts where a product on its pick belt will be at every moment (its Track: path position,
     speed along the belt measured by the camera or the simulation, sideways offset);
  2. finds the RENDEZVOUS: the earliest moment at which the product is inside the arm's reach window
     on the belt AND the arm can be there (its approach time, the time its IK takes, and when it is
     free again after its current delivery);
  3. starts moving just in time and stretches its approach so it hovers over the product as the
     product arrives, then comes down along with the moving product and grasps it at the planned
     contact point (no waiting over the belt, no belt stop).

A RendezvousScheduler sits above the arms. On a conveyor that several interchangeable arms pick from
(a "pool": the three arms on S1 of the tree world; the "all" arms of the rings) it assigns every
product coming down the belt to one arm, from the arms' rendezvous windows and when each arm is free:
  assign=balanced  (default) the feasible arm with the fewest products so far, then the earliest contact
  assign=earliest  the feasible arm that meets the product first (most throughput)
Assignments are re-planned every tick until an arm commits to a product; a commit is exclusive, so two
arms never go for the same product. A product no arm can meet in time passes on (a ring brings it round
again; the last arm of a straight takes the queue at its end). Hand-offs between bents need no extra
step: a product an arm puts on its place belt is planned by the next pool at once, and an arm places
only into a gap of the moving belt that stays free while it comes down.

Planning reads the products' positions from the simulation engine (as the feeder, the lazy cameras and
the S1 share rule already do); the grasp itself follows lab.timing["locate"]: the camera's view
(vision), the camera with the simulation's position as a counted fallback (assist), or truth only.

Select it with   --arm schedule=rendezvous   (or PIPELINE_SCHEDULE=rendezvous, or
lab.set_scheduler("rendezvous")); the lab default stays schedule=classic (hold + pause the belt).
"""

import math
import os
import threading
import time
from dataclasses import dataclass, field
from functools import lru_cache
from typing import Dict, List, Optional, Tuple

from factory_bringup import pipeline_layout as L
from factory_bringup.vision_lab import pipeline_world as P

SCHEDULES = ("classic", "rendezvous")
ASSIGN_MODES = ("balanced", "earliest")
POOL_MODES = ("rules", "conveyor", "none")
END_MARGIN = 0.15                    # as pipeline_sim: products stop this far before the end of a straight


def _reach(height: float) -> float:
    """Horizontal reach (m from the arm base) of the gripper hovering 0.25 m over a product `height` m
    tall on a belt (measured with the BCR arm's IK: 2.91 m at 0.08 m, 2.75 m at 0.40 m), minus a margin."""
    margin = float(os.environ.get("RENDEZVOUS_REACH_MARGIN", "0.08"))
    h = min(0.6, max(0.0, height))
    return 2.95 - 0.5 * h - margin


# ------------------------------------------------------------------------------------ belt geometry
def travel_sign(conv: str) -> int:
    return P.ring_dir(conv) if P.is_ring(conv) else 1


def offset(arm: str, conv: str, s: float) -> float:
    """Signed distance along the travel direction from the arm's pick point to path position s:
    < 0 still coming, > 0 already past. On a ring a product less than half a lap past counts as past."""
    s_pick = P.path_s(conv, *P.station(arm)["pick"])
    d = (s - s_pick) * travel_sign(conv)
    if P.is_ring(conv):
        n = P.length(conv)
        d = (d + n / 2) % n - n / 2
    return d


def window(arm: str, height: float, limit: float = 0.8) -> Tuple[float, float]:
    return _window(arm, round(height, 2), round(limit, 2))


@lru_cache(maxsize=None)
def _window(arm: str, height: float, limit: float) -> Tuple[float, float]:
    """(lo, hi): the offsets around the pick point where the arm can hover over and grasp a product of
    this height (geometric reach, at most `limit` m either way: the camera sees that stretch best)."""
    st = P.station(arm)
    conv = st["pick_conv"]
    bx, by = st["base"]
    r = _reach(height)
    s_pick = P.path_s(conv, *st["pick"])
    sign = travel_sign(conv)
    lo = hi = 0.0
    step = 0.02
    for direction in (-1, 1):
        o = 0.0
        while abs(o) < limit:
            nxt = o + direction * step
            x, y, _ = P.path_xy(conv, s_pick + sign * nxt)
            if math.hypot(x - bx, y - by) > r or not _on_path(conv, s_pick + sign * nxt):
                break
            o = nxt
        if direction < 0:
            lo = o
        else:
            hi = o
    return lo, hi


def _on_path(conv: str, s: float) -> bool:
    return P.is_ring(conv) or 0.0 <= s <= P.length(conv)


@dataclass
class Track:
    """A product's motion along its belt: at time t it was at path position s and moves at v m/s
    (along the belt's travel direction); lat = its sideways offset from the belt's centre line."""
    conv: str
    s: float
    t: float
    v: float
    lat: Tuple[float, float] = (0.0, 0.0)
    half_len: float = 0.15
    source: str = "sim"                       # camera, sim (assist / truth)

    def s_at(self, t: float) -> float:
        s = self.s + travel_sign(self.conv) * self.v * max(0.0, t - self.t)
        if P.is_ring(self.conv):
            return s % P.length(self.conv)
        return min(s, max(self.s, P.length(self.conv) - END_MARGIN - self.half_len))

    def stops_at(self) -> float:
        """When the product comes to a stand (at the end of a straight), inf if it keeps moving."""
        if self.v <= 1e-3:
            return self.t
        if P.is_ring(self.conv):
            return math.inf
        s_stop = max(self.s, P.length(self.conv) - END_MARGIN - self.half_len)
        return self.t + (s_stop - self.s) / self.v

    def xy_at(self, t: float) -> Tuple[float, float]:
        x, y, _ = P.path_xy(self.conv, self.s_at(t))
        return x + self.lat[0], y + self.lat[1]

    def offset_at(self, arm: str, t: float) -> float:
        return offset(arm, self.conv, self.s_at(t))

    def time_at_offset(self, arm: str, o: float, now: float) -> Optional[float]:
        """When the product reaches offset o of this arm (now if it is already there or past; None if it
        never gets there: standing still before it, or a straight ending first)."""
        o_now = self.offset_at(arm, now)
        if o_now >= o:
            return now
        if self.v <= 1e-3:                        # standing: there already, give or take a grasp
            return now if o_now >= o - 0.06 else None
        t = now + (o - o_now) / self.v
        if not P.is_ring(self.conv) and self.offset_at(arm, t) < o - 1e-3:
            # it stops at the end of the line first: close enough to o (within a grasp), it meets o there
            s_stop = max(self.s, P.length(self.conv) - END_MARGIN - self.half_len)
            if offset(arm, self.conv, s_stop) < o - 0.06:
                return None
            return max(now, self.t + (s_stop - self.s) / self.v)
        return t


def track_from_product(p, now: Optional[float] = None) -> Track:
    return Track(p.conv, p.s, now or time.time(), max(0.0, getattr(p, "v", 0.0)), (0.0, 0.0),
                 p.size[0] / 2, "sim")


def track_from_detection(conv: str, x: float, y: float, t: float, v: float, half_len: float = 0.15) -> Track:
    s = P.path_s(conv, x, y)
    cx, cy, _ = P.path_xy(conv, s)
    return Track(conv, s, t, max(0.0, v), (x - cx, y - cy), half_len, "camera")


@dataclass
class Plan:
    """Where and when an arm meets a product."""
    arm: str
    product: str
    t_start: float           # the arm starts moving (computes its IK first)
    t_hover: float           # it hovers over the product
    t_contact: float         # it touches the product (grasp)
    o_contact: float         # offset of the contact point from the pick point
    slack: float             # s the contact could still be later before the product leaves the window
    track: Track = field(repr=False, default=None)

    def hover_xy(self):
        return self.track.xy_at(self.t_hover)

    def contact_xy(self):
        return self.track.xy_at(self.t_contact)


def plan_rendezvous(arm: str, product: str, track: Track, height: float, now: float, t_free: float,
                    lead: float, t_descend: float, limit: float = 0.8) -> Optional[Plan]:
    """The earliest feasible rendezvous of `arm` (free at t_free, needing `lead` s from deciding to
    hovering, `t_descend` s to come down) with a product moving as `track`. None if it cannot make it."""
    lo, hi = window(arm, height, limit)
    t_in = track.time_at_offset(arm, lo, now)
    if t_in is None:
        return None
    t_hover = max(max(now, t_free) + lead, t_in)
    o_hover = track.offset_at(arm, t_hover)
    if o_hover < lo - 1e-3:                       # standing short of the window (queued): no rendezvous
        return None
    t_contact = t_hover + t_descend
    o_contact = track.offset_at(arm, t_contact)
    if o_contact > hi:
        return None
    slack = (hi - o_contact) / track.v if track.v > 1e-3 else math.inf
    return Plan(arm, product, t_hover - lead, t_hover, t_contact, o_contact, slack, track)


# ------------------------------------------------------------------------------------ statistics
@dataclass
class ArmStats:
    picks: int = 0
    missed: int = 0                  # contact too far from the product, or the rendezvous was lost
    late: int = 0                    # planned rendezvous no longer reachable when the arm started
    stretched: float = 0.0           # s the arms slowed their approach to arrive with the product
    contact_err: List[float] = field(default_factory=list)
    lead_used: List[float] = field(default_factory=list)


class _Ema:
    def __init__(self, value: float, k: float = 0.3):
        self.value, self.k = value, k

    def add(self, x: float):
        self.value += self.k * (x - self.value)


# ------------------------------------------------------------------------------------ scheduler
class RendezvousScheduler:
    """Assigns products on shared pick conveyors to arms by rendezvous window and arm availability, and
    makes sure only one arm commits to a product. Thread safe; ticked by the arms (at most 5 Hz)."""

    def __init__(self, lab, assign: str = "balanced", pool: str = "rules", horizon: float = 45.0):
        if assign not in ASSIGN_MODES:
            raise ValueError(f"rendezvous_assign must be one of {', '.join(ASSIGN_MODES)}")
        if pool not in POOL_MODES:
            raise ValueError(f"rendezvous_pool must be one of {', '.join(POOL_MODES)}")
        self.lab = lab
        self.assign_mode = assign
        self.pool_mode = pool
        self.horizon = horizon
        self.lock = threading.RLock()
        self.pools = self._pools(pool)
        self.committed: Dict[str, str] = {}          # product -> arm working on it
        self.assigned: Dict[str, Plan] = {}          # product -> tentative plan (pool conveyors)
        self.busy_until: Dict[str, float] = {}       # arm -> when it is free again (estimate)
        self.load: Dict[str, int] = {}               # arm -> products committed so far
        self.stats: Dict[str, ArmStats] = {a: ArmStats() for a in P.STATIONS}
        self.ik_time = {a: _Ema(1.5) for a in P.STATIONS}          # s to compute the IK of a rendezvous (learnt)
        self.tail = {a: _Ema(10.0) for a in P.STATIONS}            # s from contact to free again
        self.reassigned = 0
        self.unassignable: set = set()
        self._last_tick = 0.0
        self._prev_owner: Dict[str, str] = {}

    # ---------------------------------------------------------------- pools
    @staticmethod
    def _pools(mode: str) -> Dict[str, List[str]]:
        """conveyor -> interchangeable arms picking from it, in the order products meet them."""
        out: Dict[str, List[str]] = {}
        if mode == "none":
            return out
        for a in P.STATIONS:
            st = P.station(a)
            if mode == "rules" and st["rule"] not in ("all", "share"):
                continue
            out.setdefault(st["pick_conv"], []).append(a)
        for conv, arms in out.items():                # upstream first: furthest before its pick point
            if P.is_ring(conv):
                feed = [pl for _b, _pk, (c, pl), _r in L.ARMS.values() if c == conv]
                s0 = P.path_s(conv, *feed[0]) if feed else 0.0
                n = P.length(conv)
                arms.sort(key=lambda a: ((P.path_s(conv, *P.station(a)["pick"]) - s0) * travel_sign(conv)) % n)
            else:
                arms.sort(key=lambda a: P.path_s(conv, *P.station(a)["pick"]))
        return out

    def pool_of(self, arm: str) -> List[str]:
        return self.pools.get(P.station(arm)["pick_conv"], [])

    # ---------------------------------------------------------------- timing model
    def lead(self, arm: str) -> float:
        """s from deciding to hovering over the product: IK + the approach move."""
        return self.ik_time[arm].value + self.lab.timing["move_duration"] * 1.3

    def t_descend(self) -> float:
        return self.lab.timing["move_duration"]

    def free_at(self, arm: str, now: float) -> float:
        return max(now, self.busy_until.get(arm, 0.0))

    def plan(self, arm: str, product: str, track: Track, height: float, now: float,
             t_free: Optional[float] = None) -> Optional[Plan]:
        return plan_rendezvous(arm, product, track, height, now, self.free_at(arm, now) if t_free is None else t_free,
                               self.lead(arm), self.t_descend(), self.lab.timing.get("rendezvous_window", 0.8))

    # ---------------------------------------------------------------- assignment
    def tick(self, force: bool = False):
        now = time.time()
        if not force and now - self._last_tick < 0.2:
            return
        with self.lock:
            self._last_tick = now
            self.assigned = {}
            for conv, arms in self.pools.items():
                if len(arms) >= 2:
                    self._assign(conv, arms, now)

    def _assign(self, conv: str, arms: List[str], now: float):
        sim = self.lab.sim
        with sim.lock:
            prods = [p for p in sim.on_conveyor(conv) if p.name not in self.committed]
        avail = {a: self.free_at(a, now) for a in arms}
        load = {a: self.load.get(a, 0) for a in arms}
        jobs = []
        for p in prods:
            tr = track_from_product(p, now)
            first = None
            for a in arms:
                t = tr.time_at_offset(a, window(a, p.height)[0], now)
                if t is not None and (first is None or t < first):
                    first = t
            if first is not None and first - now <= self.horizon:
                jobs.append((first, p, tr))
        jobs.sort(key=lambda j: j[0])
        for _first, p, tr in jobs:
            options = []
            for a in arms:
                pl = plan_rendezvous(a, p.name, tr, p.height, now, avail[a], self.lead(a), self.t_descend(),
                                     self.lab.timing.get("rendezvous_window", 0.8))
                if pl is not None:
                    options.append(pl)
            if not options:
                if p.name not in self.unassignable:
                    self.unassignable.add(p.name)
                continue
            if self.assign_mode == "earliest":
                best = min(options, key=lambda pl: pl.t_contact)
            else:
                best = min(options, key=lambda pl: (load[pl.arm], pl.t_contact))
            self.assigned[p.name] = best
            self.unassignable.discard(p.name)
            prev = self._prev_owner.get(p.name)
            if prev is not None and prev != best.arm:
                self.reassigned += 1
            self._prev_owner[p.name] = best.arm
            avail[best.arm] = best.t_contact + self.tail[best.arm].value
            load[best.arm] += 1

    # ---------------------------------------------------------------- arms ask
    def may_take(self, arm: str, product: str) -> bool:
        """False when another arm works on the product, or the scheduler gave it to another arm of the pool."""
        with self.lock:
            owner = self.committed.get(product)
            if owner is not None:
                return owner == arm
            plan = self.assigned.get(product)
            if plan is None or plan.arm == arm:
                return True
            return arm not in self.pool_of(plan.arm)

    def owner(self, product: str) -> Optional[str]:
        with self.lock:
            if product in self.committed:
                return self.committed[product]
            plan = self.assigned.get(product)
            return plan.arm if plan else None

    def commit(self, arm: str, plan: Plan) -> bool:
        with self.lock:
            if not self.may_take(arm, plan.product):
                return False
            self.committed[plan.product] = arm
            self.assigned.pop(plan.product, None)
            self.busy_until[arm] = plan.t_contact + self.tail[arm].value
            self.load[arm] = self.load.get(arm, 0) + 1
            return True

    def release(self, arm: str, product: str, picked: bool):
        with self.lock:
            if self.committed.get(product) == arm:
                del self.committed[product]
            if not picked:
                self.load[arm] = max(0, self.load.get(arm, 0) - 1)
            self.busy_until[arm] = time.time()
            self._prev_owner.pop(product, None)

    # ---------------------------------------------------------------- report
    def report(self, stopped: Optional[Dict[str, float]] = None) -> str:
        lines = [f"rendezvous scheduler: assign={self.assign_mode}, pools="
                 + (", ".join(f"{c}: {'/'.join(a)}" for c, a in self.pools.items() if len(a) >= 2) or "none")
                 + f", re-assignments {self.reassigned}"]
        for a in P.STATIONS:
            s = self.stats[a]
            if not (s.picks or s.missed or s.late):
                continue
            err = f"{100 * sum(s.contact_err) / len(s.contact_err):.1f} cm" if s.contact_err else "n/a"
            lines.append(f"  {a:3s} picks {s.picks:3d}  missed {s.missed}  late {s.late}  mean contact error {err}  "
                         f"approach stretched {s.stretched:.1f} s  IK {self.ik_time[a].value:.2f} s  "
                         f"contact->free {self.tail[a].value:.1f} s")
        if stopped is not None:
            total = sum(stopped.values())
            lines.append(f"belt stop time for arms: {total:.1f} s"
                         + (" (" + ", ".join(f"{c} {v:.0f} s" for c, v in stopped.items() if v > 0.05) + ")" if total > 0.05 else ""))
        return "\n".join(lines)
