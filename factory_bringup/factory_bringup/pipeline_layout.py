"""Layout of the pipeline factory world (pipeline_factory.sdf): straight conveyors carry products
between stations, circular conveyors are selection loops where BCR arms pick out the products
that match their rule, and every final straight line ends over a dock box.

The world generator (my_factory_worlds/tools/make_pipeline_world.py) builds the world from this
table; a later pipeline controller can read the same table.

Flow:  S1 inbound -> A1 -> C1 sorter 1
         C1 -> A2 (recyclable)     -> S2 -> A4 -> C2 recycling sort
                C2 -> A6 (cardboard)   -> S4 -> dock 1
                C2 -> A7 (other)       -> S5 -> dock 2
         C1 -> A3 (all others)     -> S3 -> A5 -> C3 general sort
                C3 -> A9 (large)       -> S7 -> dock 3
                C3 -> A8 (small, medium) -> S6 -> A10 -> C4 shipping sort
                       C4 -> A11 (small)   -> S8 -> dock 4
                       C4 -> A12 (other)   -> S9 -> dock 5
"""
WORLD_NAME = "pipeline_factory_world"
BELT_TOP_Z = 1.375          # top of every belt, as in the other worlds
RING_PATH_RADIUS = 3.42     # products ride the circular conveyors at this radius (counter-clockwise)
ARM_BASE_Z = -0.508         # arm base height, as the BCR arm in the other worlds
BELT_SPEED = 0.2394         # m/s

# circular selection conveyors: name -> (centre x, centre y, sign text)
CIRCLES = {
    "C1": (0.0, 0.0, "C1 SORT RECYCLABLE"),
    "C2": (0.0, 21.0, "C2 SORT CARDBOARD"),
    "C3": (22.0, 0.0, "C3 SORT BY SIZE"),
    "C4": (22.0, -22.0, "C4 SORT SHIPPING"),
}

# straight transport conveyors: name -> (start xy, end xy, sign text, dock text or None);
# products move from start to end. A dock box stands past the end of an output line.
STRAIGHTS = {
    "S1": ((-22.0, 0.0), (-8.05, 0.0), "S1 INBOUND", None),
    "S2": ((0.0, 7.85), (0.0, 12.95), "S2 TO RECYCLING", None),
    "S3": ((7.85, 0.0), (13.95, 0.0), "S3 TO GENERAL", None),
    "S4": ((-7.85, 21.0), (-18.0, 21.0), "S4 CARDBOARD", "DOCK 1 CARDBOARD"),
    "S5": ((7.85, 21.0), (18.0, 21.0), "S5 CONTAINERS", "DOCK 2 CONTAINERS"),
    "S6": ((22.0, -7.85), (22.0, -13.95), "S6 TO SHIPPING", None),
    "S7": ((29.85, 0.0), (40.0, 0.0), "S7 LARGE", "DOCK 3 LARGE"),
    "S8": ((14.15, -22.0), (4.0, -22.0), "S8 EXPRESS", "DOCK 4 EXPRESS"),
    "S9": ((29.85, -22.0), (40.0, -22.0), "S9 STANDARD", "DOCK 5 STANDARD"),
}

# BCR arm stations: name -> (base xy, (conveyor, pick xy), (conveyor, place xy), rule)
# rule: which products this arm takes from a circular conveyor ("all" = every product that reaches it)
ARMS = {
    "A1": ((-6.0, 0.0), ("S1", (-8.45, 0.0)), ("C1", (-3.42, 0.0)), "all"),
    "A2": ((0.0, 6.0), ("C1", (0.0, 3.42)), ("S2", (0.0, 8.45)), "recyclable"),
    "A3": ((6.0, 0.0), ("C1", (3.42, 0.0)), ("S3", (8.45, 0.0)), "all"),
    "A4": ((0.0, 15.0), ("S2", (0.0, 12.55)), ("C2", (0.0, 17.58)), "all"),
    "A5": ((16.0, 0.0), ("S3", (13.55, 0.0)), ("C3", (18.58, 0.0)), "all"),
    "A6": ((-6.0, 21.0), ("C2", (-3.42, 21.0)), ("S4", (-8.45, 21.0)), "cardboard"),
    "A7": ((6.0, 21.0), ("C2", (3.42, 21.0)), ("S5", (8.45, 21.0)), "all"),
    "A8": ((22.0, -6.0), ("C3", (22.0, -3.42)), ("S6", (22.0, -8.45)), "not_large"),
    "A9": ((28.0, 0.0), ("C3", (25.42, 0.0)), ("S7", (30.45, 0.0)), "all"),
    "A10": ((22.0, -16.0), ("S6", (22.0, -13.55)), ("C4", (22.0, -18.58)), "all"),
    "A11": ((16.0, -22.0), ("C4", (18.58, -22.0)), ("S8", (13.55, -22.0)), "small"),
    "A12": ((28.0, -22.0), ("C4", (25.42, -22.0)), ("S9", (30.45, -22.0)), "all"),
}

HALL = (-26.0, 47.0, -28.0, 28.0)   # x min, x max, y min, y max of the hall walls
