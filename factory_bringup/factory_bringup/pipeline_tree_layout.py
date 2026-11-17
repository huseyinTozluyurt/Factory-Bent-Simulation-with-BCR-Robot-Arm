"""Layout of the tree-of-three pipeline world (pipeline_tree_factory.sdf): the same parts as the
pipeline world (13 bents, 12 BCR arms), but the inbound conveyor S1 feeds THREE arms, and from there
the factory fans out as a ternary tree: S1 -> 3 rings -> 3 output lines per ring.

    level 0   S1 inbound (one straight belt, three arms along it)
    level 1   A1, A2, A3 share the products of S1 between three sorting rings (rule "share":
              an arm takes a product while it has delivered no more than the other S1 arms;
              A3 at the end of S1 takes every product the others let pass)
    level 2   each ring sorts by size onto three output lines that end over a dock box

Flow:  S1 inbound -A1 (share)-> C1 branch 1 sort
                                  C1 -A4 (large)  -> S2 -> dock 1  large 1
                                  C1 -A5 (small)  -> S3 -> dock 2  small 1
                                  C1 -A6 (others) -> S4 -> dock 3  standard 1
                  -A2 (share)-> C2 branch 2 sort
                                  C2 -A7 (large)  -> S5 -> dock 4  large 2
                                  C2 -A8 (small)  -> S6 -> dock 5  small 2
                                  C2 -A9 (others) -> S7 -> dock 6  standard 2
                  -A3 (rest)-> C3 branch 3 sort
                                  C3 -A10 (large)  -> S8  -> dock 7  large 3
                                  C3 -A11 (small)  -> S9  -> dock 8  small 3
                                  C3 -A12 (others) -> S10 -> dock 9  standard 3

1 + 3 + 9 = 13 bents (1 inbound straight, 3 rings, 9 output lines) and 3 + 9 = 12 arms: a complete
ternary tree of depth 2 has exactly the bent and arm count of the pipeline world.
"""
import math

WORLD_NAME = "pipeline_tree_world"
BASE_SDF = "pipeline_tree_factory.sdf"          # layout world (make_pipeline_world.py)
LAB_SDF = "pipeline_tree_factory_lab.sdf"       # vision-lab world (make_pipeline_lab_world.py)
BELT_TOP_Z = 1.375
RING_PATH_RADIUS = 3.42
ARM_BASE_Z = -0.508
BELT_SPEED = 0.2394

# geometry of an arm station, as in the pipeline world: base 2.45 m from the pick point of a straight,
# 2.58 m from a ring's product path; a straight starts 7.85 m from a ring centre
_PICK, _PLACE_RING, _BASE_RING, _LINE_START, _LINE_END = 2.45, 3.42, 6.0, 7.85, 18.0

CIRCLES = {
    "C1": (-24.0, 8.45, "C1 BRANCH 1 SORT"),
    "C2": (-16.0, -8.45, "C2 BRANCH 2 SORT"),
    "C3": (0.0, 0.0, "C3 BRANCH 3 SORT"),
}


def _ray(c, ang_deg, r):
    cx, cy, _ = CIRCLES[c]
    a = math.radians(ang_deg)
    return round(cx + r * math.cos(a), 4), round(cy + r * math.sin(a), 4)


def _ring_exit(ring, ang_deg, line_end):
    """(arm base, pick on the ring, place on the line, line start, line end) for an arm at angle ang."""
    return (_ray(ring, ang_deg, _BASE_RING), _ray(ring, ang_deg, _PLACE_RING),
            _ray(ring, ang_deg, _BASE_RING + _PICK), _ray(ring, ang_deg, _LINE_START), _ray(ring, ang_deg, line_end))


# ring exits: arm -> (ring, angle in degrees, distance of the line end from the ring centre, line, rule,
#                     line sign, dock sign)
# Rings run counter-clockwise; from the feeding arm a product meets "large", then "small", then "all".
_EXITS = {
    "A4": ("C1", 0.0, 17.0, "S2", "large", "S2 LARGE 1", "DOCK 1 LARGE 1"),
    "A5": ("C1", 90.0, _LINE_END - 0.4, "S3", "small", "S3 SMALL 1", "DOCK 2 SMALL 1"),
    "A6": ("C1", 180.0, 16.0, "S4", "all", "S4 STANDARD 1", "DOCK 3 STANDARD 1"),
    "A7": ("C2", 180.0, 18.0, "S5", "large", "S5 LARGE 2", "DOCK 4 LARGE 2"),
    "A8": ("C2", 270.0, _LINE_END - 0.4, "S6", "small", "S6 SMALL 2", "DOCK 5 SMALL 2"),
    "A9": ("C2", 0.0, 13.0, "S7", "all", "S7 STANDARD 2", "DOCK 6 STANDARD 2"),
    "A10": ("C3", 300.0, 16.0, "S8", "large", "S8 LARGE 3", "DOCK 7 LARGE 3"),
    "A11": ("C3", 0.0, 16.0, "S9", "small", "S9 SMALL 3", "DOCK 8 SMALL 3"),
    "A12": ("C3", 60.0, 16.0, "S10", "all", "S10 STANDARD 3", "DOCK 9 STANDARD 3"),
}

# straight transport conveyors: name -> (start xy, end xy, sign text, dock text or None)
STRAIGHTS = {"S1": ((-32.0, 0.0), (-8.05, 0.0), "S1 INBOUND", None)}
# BCR arm stations: name -> (base xy, (conveyor, pick xy), (conveyor, place xy), rule)
ARMS = {
    # S1 -> the three branches: two arms beside the belt, one at its end
    "A1": ((-24.0, _PICK), ("S1", (-24.0, 0.0)), ("C1", _ray("C1", 270.0, _PLACE_RING)), "share"),
    "A2": ((-16.0, -_PICK), ("S1", (-16.0, 0.0)), ("C2", _ray("C2", 90.0, _PLACE_RING)), "share"),
    "A3": ((-6.0, 0.0), ("S1", (-8.45, 0.0)), ("C3", _ray("C3", 180.0, _PLACE_RING)), "all"),
}
for _arm, (_ring, _ang, _end, _line, _rule, _sign, _dock) in _EXITS.items():
    _base, _pick, _place, _start, _stop = _ring_exit(_ring, _ang, _end)
    STRAIGHTS[_line] = (_start, _stop, _sign, _dock)
    ARMS[_arm] = (_base, (_ring, _pick), (_line, _place), _rule)

HALL = (-46.0, 24.0, -30.0, 30.0)     # x min, x max, y min, y max of the hall walls
WALKWAYS_Y = ()                       # no painted walkway lines across the hall (they would cross the belts)
LIGHTS = tuple((x, y) for x in (-36.0, -18.0, 0.0, 16.0) for y in (-18.0, 0.0, 18.0))
