"""Builds pipeline_factory.sdf, the pipeline factory world (layout only: everything is static), from
pipeline_layout.py: straight transport conveyors, circular selection conveyors copied from the
dual world, 12 BCR arms posed over their pick points, dock boxes, a hall and floor labels.

Run next to pipeline_layout.py, urdf_ik.py, arm.urdf (the BCR arm's robot_description) and
dual.sdf (a copy of dual_circular_factory_conveyor.sdf):
    python3 make_pipeline_world.py            -> pipeline_factory.sdf
    python3 make_pipeline_world.py --preview  -> pipeline_preview.sdf (arms as plain cylinders, for renders)"""
import math, re, sys
import xml.etree.ElementTree as ET
import numpy as np
import pipeline_layout as L
from urdf_ik import UrdfChain

PREVIEW = "--preview" in sys.argv
DUAL = open("dual.sdf").read()
URDF = open("arm.urdf").read()


def mat(c, emissive=None):
    r, g, b = c
    s = (f"<material><ambient>{r*0.8:.3f} {g*0.8:.3f} {b*0.8:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
         f"<specular>0.15 0.15 0.15 1</specular>")
    if emissive:
        s += f"<emissive>{emissive[0]:.3f} {emissive[1]:.3f} {emissive[2]:.3f} 1</emissive>"
    return s + "</material>"


class Model:
    """A static model whose parts are posed in the model frame (pose x y z yaw)."""
    def __init__(self, name, pose=(0, 0, 0, 0)):
        self.name, self.pose, self.parts, self.n = name, pose, [], 0

    def _add(self, x, y, z, roll, pitch, yaw, geom, color, collide, emissive=None):
        self.n += 1
        p = f"<pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>"
        if collide:
            self.parts.append(f'<collision name="c{self.n:04d}">{p}<geometry>{geom}</geometry></collision>')
        self.parts.append(f'<visual name="v{self.n:04d}">{p}<geometry>{geom}</geometry>{mat(color, emissive)}</visual>')

    def box(self, x, y, z, sx, sy, sz, color, yaw=0.0, collide=False, emissive=None):
        self._add(x, y, z, 0, 0, yaw, f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>", color, collide, emissive)

    def cyl(self, x, y, z, r, h, color, roll=0.0, pitch=0.0, yaw=0.0, collide=False):
        self._add(x, y, z, roll, pitch, yaw, f"<cylinder><radius>{r:.4f}</radius><length>{h:.4f}</length></cylinder>",
                  color, collide)

    def sdf(self):
        x, y, z, yaw = self.pose
        return (f'    <model name="{self.name}">\n      <static>true</static>\n'
                f'      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>\n      <link name="link">\n        '
                + "\n        ".join(self.parts) + "\n      </link>\n    </model>\n")


BELT = (0.08, 0.08, 0.09)
FRAME = (0.55, 0.58, 0.62)
STEEL = (0.26, 0.29, 0.33)
YELLOW = (0.98, 0.76, 0.08)
WHITE = (0.95, 0.95, 0.95)
DOCK_COLORS = [(0.75, 0.50, 0.25), (0.20, 0.55, 0.85), (0.80, 0.25, 0.25), (0.25, 0.65, 0.35), (0.55, 0.35, 0.75)]

# 3 x 5 block letters for the floor labels
FONT = {
    "A": ["111", "101", "111", "101", "101"], "B": ["110", "101", "110", "101", "110"], "C": ["111", "100", "100", "100", "111"],
    "D": ["110", "101", "101", "101", "110"], "E": ["111", "100", "110", "100", "111"], "F": ["111", "100", "110", "100", "100"],
    "G": ["111", "100", "101", "101", "111"], "H": ["101", "101", "111", "101", "101"], "I": ["111", "010", "010", "010", "111"],
    "J": ["001", "001", "001", "101", "111"], "K": ["101", "101", "110", "101", "101"], "L": ["100", "100", "100", "100", "111"],
    "M": ["101", "111", "111", "101", "101"], "N": ["110", "101", "101", "101", "101"], "O": ["111", "101", "101", "101", "111"],
    "P": ["111", "101", "111", "100", "100"], "Q": ["111", "101", "101", "111", "001"], "R": ["111", "101", "110", "101", "101"],
    "S": ["111", "100", "111", "001", "111"], "T": ["111", "010", "010", "010", "010"], "U": ["101", "101", "101", "101", "111"],
    "V": ["101", "101", "101", "101", "010"], "W": ["101", "101", "111", "111", "101"], "X": ["101", "101", "010", "101", "101"],
    "Y": ["101", "101", "010", "010", "010"], "Z": ["111", "001", "010", "100", "111"], " ": ["000"] * 5,
    "0": ["111", "101", "101", "101", "111"], "1": ["010", "110", "010", "010", "111"], "2": ["111", "001", "111", "100", "111"],
    "3": ["111", "001", "111", "001", "111"], "4": ["101", "101", "111", "001", "001"], "5": ["111", "100", "111", "001", "111"],
    "6": ["111", "100", "111", "101", "111"], "7": ["111", "001", "001", "001", "001"], "8": ["111", "101", "111", "101", "111"],
    "9": ["111", "101", "111", "001", "111"],
}


def text_width(text, px):
    return (len(text) * 4 - 1) * px


def floor_text(m, text, cx, cy, z, px, color, yaw=0.0, patch=None):
    """Block letters lying flat, centred on (cx, cy) in the model frame, reading along yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    w = text_width(text, px)
    if patch:
        m.box(cx, cy, z - 0.002, w + 4 * px, 9 * px, 0.004, patch, yaw=yaw)
    for i, ch in enumerate(text):
        for r, row in enumerate(FONT[ch]):
            col = 0
            while col < 3:
                if row[col] == "1":
                    start = col
                    while col < 3 and row[col] == "1":
                        col += 1
                    u = -w / 2 + (i * 4 + (start + col) / 2) * px      # along the text
                    v = (2 - r) * px                                      # across (rows top to bottom)
                    m.box(cx + c * u - s * v, cy + s * u + c * v, z, (col - start) * px, px, 0.004, color, yaw=yaw)
                else:
                    col += 1


# ------------------------------------------------------------------ straight conveyor
def straight(name, start, end, label, dock, dock_color):
    (x0, y0), (x1, y1) = start, end
    length = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    m = Model(f"conveyor_{name}", ((x0 + x1) / 2, (y0 + y1) / 2, 0.0, yaw))
    h = length / 2
    m.box(0, 0, 1.36, length, 1.08, 0.03, BELT, collide=True)                      # belt, top at 1.375
    for side in (-1, 1):
        m.box(0, side * 0.58, 1.33, length, 0.08, 0.20, FRAME, collide=True)       # side frames, lip above the belt
        m.box(0, side * 0.58, 1.44, length, 0.04, 0.03, YELLOW)                     # safety edge
    m.box(0, 0, 1.22, length, 0.90, 0.10, STEEL)                                    # under frame
    n_legs = max(2, int(length // 2.4) + 2)
    for i in range(n_legs):
        x = -h + 0.3 + i * (length - 0.6) / (n_legs - 1)
        for side in (-1, 1):
            m.box(x, side * 0.42, 0.585, 0.08, 0.08, 1.17, STEEL)
            m.box(x, side * 0.42, 0.01, 0.22, 0.22, 0.02, STEEL)
    for x in (-h, h):                                                                # end rollers
        m.cyl(x, 0, 1.315, 0.06, 1.08, FRAME, roll=math.pi / 2)
    k = 1.0
    while k < length - 0.5:                                                          # chevrons: direction of travel
        for sgn in (-1, 1):
            m.box(-h + k - 0.12, sgn * 0.13, 1.377, 0.36, 0.05, 0.004, YELLOW, yaw=-sgn * 0.6)
        k += 2.0
    # label on the floor beside the belt, reading along it
    flip = math.cos(yaw) < -0.1 or (abs(math.cos(yaw)) <= 0.1 and math.sin(yaw) < 0)    # keep labels upright
    floor_text(m, label, 0, 1.25 if flip else -1.25, 0.012, 0.11, WHITE, yaw=math.pi if flip else 0.0,
               patch=(0.15, 0.18, 0.24))
    parts = [m.sdf()]
    if dock:
        d = Model(f"dock_{name}", (x1 + math.cos(yaw) * 1.7, y1 + math.sin(yaw) * 1.7, 0.0, yaw))
        d.box(0, 0, 0.075, 2.6, 2.8, 0.12, (0.20, 0.22, 0.28), collide=True)          # floor top 0.135
        for side in (-1, 1):
            d.box(side * 1.27, 0, 0.47, 0.06, 2.8, 0.80, dock_color, collide=True)    # front / back walls
            d.box(0, side * 1.37, 0.47, 2.6, 0.06, 0.80, dock_color, collide=True)    # side walls
        floor_text(d, dock, 0, 2.0 if flip else -2.0, 0.012, 0.11, WHITE, yaw=math.pi if flip else 0.0, patch=dock_color)
        parts.append(d.sdf())
    return parts


# ------------------------------------------------------------------ circular conveyors (from the dual world)
def circle(name, cx, cy, label):
    dx, dy = cx + 6.0, cy
    out = []
    for m in re.finditer(r'\n    <model name="left_([^"]+)">.*?\n    </model>', DUAL, re.S):
        body = m.group(0).replace(f'<model name="left_{m.group(1)}">', f'<model name="{name}_{m.group(1)}">', 1)
        head = re.match(r'(\n    <model name="[^"]+">\n      <static>true</static>\n)(      <pose>([^<]+)</pose>\n)?', body)
        if head.group(2):
            v = [float(t) for t in head.group(3).split()]
            v[0] += dx; v[1] += dy
            pose = "      <pose>" + " ".join(f"{t:.4f}" for t in v) + "</pose>\n"
        else:
            pose = f"      <pose>{dx:.4f} {dy:.4f} 0 0 0 0</pose>\n"
        out.append(head.group(1) + pose + body[head.end():])
    lab = Model(f"{name}_label", (cx, cy, 0.0, 0.0))
    floor_text(lab, label.split(" ", 1)[0], 0, 0.35, 1.34, 0.16, WHITE)            # on the centre platform
    floor_text(lab, label.split(" ", 1)[1], 0, -0.45, 1.34, 0.07, YELLOW)
    return out + [lab.sdf()]


# ------------------------------------------------------------------ BCR arms
ROBOT = ET.fromstring(URDF)
COLORS = {}
for g in ROBOT.findall("gazebo"):
    d = g.find("material/diffuse")
    if g.get("reference") and d is not None:
        COLORS[g.get("reference")] = tuple(float(v) for v in d.text.split()[:3])
CHAIN = UrdfChain(URDF, "base_link", "gripper_tip")
NAMES = [f"joint{i}" for i in range(1, 8)]
SEED = [1.30, -0.75, 0.0, -0.80, 0.0, -1.20, 0.0]


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr], [-sp, cp * sr, cp * cr]])


def tf(xyz="0 0 0", r="0 0 0"):
    T = np.eye(4)
    T[:3, :3] = rpy(*[float(v) for v in r.split()])
    T[:3, 3] = [float(v) for v in xyz.split()]
    return T


def axis_rot(axis, a):
    k = np.array(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K
    return T


def link_frames(q):
    """base_link-frame transform of every link for joint values q."""
    frames = {"base_link": np.eye(4)}
    joints = ROBOT.findall("joint")
    changed = True
    while changed:
        changed = False
        for j in joints:
            parent, child = j.find("parent").get("link"), j.find("child").get("link")
            if parent in frames and child not in frames:
                o = j.find("origin")
                T = frames[parent] @ (tf(o.get("xyz", "0 0 0"), o.get("rpy", "0 0 0")) if o is not None else np.eye(4))
                if j.get("type") in ("revolute", "continuous"):
                    T = T @ axis_rot([float(v) for v in j.find("axis").get("xyz").split()], q.get(j.get("name"), 0.0))
                frames[child] = T
                changed = True
    return frames


def ready_posture(base, target):
    """Joint values with the gripper tip over target (x, y, z) for an arm based at base (x, y)."""
    t = (target[0] - base[0], target[1] - base[1], target[2] - L.ARM_BASE_Z)
    bearing = math.atan2(t[1], t[0])
    seeds = []
    for j1 in (bearing, bearing + math.pi / 2, bearing - math.pi / 2):
        for j2 in (-0.75, 0.0, 0.75):
            s = list(SEED); s[0], s[1] = j1, j2
            seeds.append(dict(zip(NAMES, s)))
    q, err = CHAIN.solve_position(t, seeds, active=NAMES)
    assert err < 0.005, (base, target, err)
    return q


def euler(R):
    return (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1.0, min(1.0, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


def arm(name, base, pick):
    q = ready_posture(base, (pick[0], pick[1], 2.0))                  # waiting over its pick point
    frames = link_frames(q)
    m = Model(f"bcr_arm_{name}", (base[0], base[1], L.ARM_BASE_Z, 0.0))
    if PREVIEW:                                                        # renderable stand-in: a cylinder per link
        pts = [frames[n][:3, 3] for n in ("base_link", "link1", "link2", "link3", "link4", "link5", "link6", "flange", "gripper_tip")]
        for a, b in zip(pts, pts[1:]):
            d = b - a
            Lg = float(np.linalg.norm(d))
            if Lg < 1e-3:
                continue
            mid = (a + b) / 2
            m.cyl(mid[0], mid[1], mid[2], 0.07, Lg, (0.95, 0.55, 0.10),
                  pitch=math.acos(max(-1, min(1, d[2] / Lg))), yaw=math.atan2(d[1], d[0]))
    else:
        for link in ROBOT.findall("link"):
            if link.get("name") not in frames:
                continue
            for k, v in enumerate(link.findall("visual")):
                mesh = v.find("geometry/mesh")
                if mesh is None:
                    continue
                o = v.find("origin")
                T = frames[link.get("name")] @ (tf(o.get("xyz", "0 0 0"), o.get("rpy", "0 0 0")) if o is not None else np.eye(4))
                r, p, y = euler(T[:3, :3])
                uri = mesh.get("filename").replace("package://", "model://")
                sc = mesh.get("scale", "1 1 1")
                m.n += 1
                c = COLORS.get(link.get("name"), (0.6, 0.6, 0.6))
                m.parts.append(f'<visual name="{link.get("name")}_{k}"><pose>{T[0,3]:.4f} {T[1,3]:.4f} {T[2,3]:.4f} '
                               f'{r:.4f} {p:.4f} {y:.4f}</pose><geometry><mesh><uri>{uri}</uri><scale>{sc}</scale></mesh>'
                               f'</geometry>{mat(c)}</visual>')
    # pedestal and floor marker, as next to the arm in the other worlds
    ped = Model(f"arm_pedestal_{name}", (base[0] + 0.2, base[1] + 0.154, 0.0, 0.0))
    ped.cyl(0, 0, 0.18, 0.35, 0.28, (0.30, 0.32, 0.36), collide=True)
    ped.box(0, 0, 0.035, 2.4, 2.4, 0.04, YELLOW)
    ped.box(0, 0, 0.037, 2.2, 2.2, 0.04, (0.22, 0.24, 0.28))
    floor_text(ped, name, 0, -0.85, 0.06, 0.09, WHITE)
    return [m.sdf(), ped.sdf()]


# ------------------------------------------------------------------ hall
def hall():
    x0, x1, y0, y1 = L.HALL
    cx, cy, w, d = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    m = Model("pipeline_hall")
    m.box(cx, cy, 0.003, w, d, 0.006, (0.33, 0.40, 0.38))                          # epoxy floor
    for x, y, sx, sy in ((x0, cy, 0.3, d), (x1, cy, 0.3, d), (cx, y0, w, 0.3), (cx, y1, w, 0.3)):
        m.box(x, y, 4.0, sx, sy, 8.0, (0.80, 0.81, 0.82))
        m.box(x, y, 0.6, sx + 0.04, sy + 0.04, 1.2, (0.17, 0.24, 0.33))
        m.box(x, y, 6.1, sx + 0.04, sy + 0.04, 1.3, (0.62, 0.78, 0.92), emissive=(0.25, 0.33, 0.40))
    x = x0 + 4
    while x < x1 - 2:                                                                   # columns
        for y in (y0 + 0.35, y1 - 0.35):
            m.box(x, y, 4.0, 0.4, 0.4, 8.0, (0.26, 0.29, 0.33))
        x += 8
    # walkway lines between the lines and loops
    for y in (-11.0, 11.0):
        m.box(cx, y, 0.008, w - 2, 0.12, 0.004, YELLOW)
    return m.sdf()


LIGHTS = "".join(f"""    <light name="hall_light_{i}" type="point">
      <pose>{x} {y} 7.0 0 0 0</pose>
      <diffuse>0.5 0.5 0.47 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation><range>18</range><constant>0.4</constant><linear>0.05</linear><quadratic>0.005</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>
""" for i, (x, y) in enumerate((x, y) for x in (-14, 6, 26) for y in (-16, 0, 16)))


def world():
    ground = re.search(r'\n    <model name="ground_plane">.*?\n    </model>', DUAL, re.S).group(0)
    ground = re.sub(r"<size>90 80</size>", "<size>120 100</size>", ground)
    parts = [hall()]
    for name, (cx, cy, label) in L.CIRCLES.items():
        parts += circle(name, cx, cy, label)
    docks = 0
    for name, (start, end, label, dock) in L.STRAIGHTS.items():
        parts += straight(name, start, end, label, dock, DOCK_COLORS[docks % len(DOCK_COLORS)])
        docks += bool(dock)
    for name, (base, (_, pick), _place, _rule) in L.ARMS.items():
        parts += arm(name, base, pick)
    flow = "\n".join("      " + line for line in L.__doc__.strip().splitlines())
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{L.WORLD_NAME}">
    <!--
{flow}
      Layout only: every conveyor and arm is static. Built by make_pipeline_world.py.
    -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>

    <gravity>0 0 -9.8</gravity>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 18 0 0 0</pose>
      <diffuse>0.95 0.95 0.95 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.1 -0.9</direction>
    </light>
{LIGHTS}
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.62 0.74 0.86 1</background>
    </scene>
{ground}
""" + "".join(parts) + "  </world>\n</sdf>\n"


out = "pipeline_preview.sdf" if PREVIEW else "pipeline_factory.sdf"
text = world()
assert "--" not in "".join(re.findall(r"<!--(.*?)-->", text, re.S)), "a comment contains --"
ET.fromstring(text.split("?>", 1)[1])                                  # well-formed
open(out, "w").write(text)
print(out, len(text), "bytes,", len(re.findall(r'\n    <model name="', text)), "models")
