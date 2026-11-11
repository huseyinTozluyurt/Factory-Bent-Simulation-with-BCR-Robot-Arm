#!/usr/bin/env python3
"""
Realistic looks of the products (no ROS imports; used through product_catalog.product_sdf).

Every product type keeps ONE simple collision shape, exactly its catalogue box / cylinder /
sphere, so grasping, the bins, the size lookups and the physics do not change. Only the
<visual> part is detailed: several primitive parts (gable roof and cap of a milk carton,
shoulder, neck and cap of a bottle, lid and ribbon of a gift box, seams of a basketball ...)
and printed labels, which are PNG textures drawn here with OpenCV into TEXTURE_DIR the first
time they are needed (fictional brands only).

Rules that keep the vision code working:
  * every part stays inside the catalogue size (thin labels stick out 1-2 mm at most), so the
    overhead camera sees the same footprint and the side camera the same height;
  * seen from above, the product is one solid blob whose middle has the catalogue colour
    (product_catalog.PRODUCT_TYPES[...]["rgb"]): labels and prints are on the sides, and
    top details (tape, seams, caps, ribbons) are thin, small or near the edges, so the colour
    classifier (vision_lab/classify.py) and opencv_color still find the type by its colour;
  * nothing is saturated green (the overhead detector drops the green counting gate).

  product_visuals_xml(ptype, size, yaw)   ->  the <visual> elements of one product link
"""

import hashlib
import math
import os
import re
import zlib
from typing import Callable, Dict, List, Optional, Sequence, Tuple

TEXTURE_DIR = os.path.expanduser("~/ros2_ws/src/factory_bent_project/factory_bringup/product_textures")
TEXTURE_VERSION = 2            # bump to redraw every label
PRODUCT_SIZES: Dict[str, Tuple] = {}     # filled by product_catalog: type -> (size, rgb), for ensure_textures

RGB = Tuple[float, float, float]
Pose = Tuple[float, float, float, float, float, float]


# ---------------------------------------------------------------------------- colours / materials
def dk(c: RGB, f: float = 0.7) -> RGB:
    return tuple(max(0.0, min(1.0, v * f)) for v in c)


def lt(c: RGB, f: float = 0.35) -> RGB:
    return tuple(max(0.0, min(1.0, v + (1.0 - v) * f)) for v in c)


def mat(rgb: RGB = (1.0, 1.0, 1.0), spec: float = 0.1, tex: Optional[str] = None, metal: float = 0.0,
        rough: float = 0.8) -> Dict:
    return {"rgb": tuple(rgb), "spec": spec, "tex": tex, "metal": metal, "rough": rough}


def material_xml(m: Dict) -> str:
    r, g, b = (1.0, 1.0, 1.0) if m.get("tex") else m["rgb"]
    s = m.get("spec", 0.1)
    xml = (f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient><diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
           f"<specular>{s:.2f} {s:.2f} {s:.2f} 1</specular>")
    tex = texture_path(m["tex"]) if m.get("tex") else None
    if tex or m.get("metal", 0.0) > 0.0:
        xml += (f"<pbr><metal>" + (f"<albedo_map>{tex}</albedo_map>" if tex else "")
                + f"<metalness>{m.get('metal', 0.0):.2f}</metalness><roughness>{m.get('rough', 0.8):.2f}</roughness>"
                f"</metal></pbr>")
    return xml + "</material>"


# ---------------------------------------------------------------------------- parts
class Part:
    __slots__ = ("name", "geom", "pose", "mat", "alpha")

    def __init__(self, name: str, geom: str, pose: Sequence[float], m: Dict, alpha: float = 0.0):
        self.name, self.geom, self.pose, self.mat, self.alpha = name, geom, tuple(pose) + (0.0,) * (6 - len(pose)), m, alpha


def gbox(sx, sy, sz) -> str:
    return f"<box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box>"


def gcyl(r, length) -> str:
    return f"<cylinder><radius>{r:.4f}</radius><length>{length:.4f}</length></cylinder>"


def gsph(r) -> str:
    return f"<sphere><radius>{r:.4f}</radius></sphere>"


def _rpy_to_m(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _m_to_rpy(m):
    p = math.asin(max(-1.0, min(1.0, -m[2][0])))
    if abs(math.cos(p)) > 1e-6:
        return math.atan2(m[2][1], m[2][2]), p, math.atan2(m[1][0], m[0][0])
    return math.atan2(-m[1][2], m[1][1]), p, 0.0


def _yawed(pose: Pose, yaw: float) -> Pose:
    if not yaw:
        return pose
    x, y, z, r, p, w = pose
    c, s = math.cos(yaw), math.sin(yaw)
    rz = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
    m = _rpy_to_m(r, p, w)
    mm = [[sum(rz[i][k] * m[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return (c * x - s * y, s * x + c * y, z) + _m_to_rpy(mm)


def _is_thin(geom: str) -> bool:
    """Tape, seams, stickers, lids: thinner than 6 mm. They cast no shadow (shadow acne on them otherwise)."""
    m = re.search(r"<size>([^<]+)</size>", geom) or re.search(r"<length>([^<]+)</length>", geom)
    return bool(m) and min(float(v) for v in m.group(1).split()) < 0.006


def visuals_xml(parts: List[Part], yaw: float = 0.0) -> str:
    out = []
    for p in parts:
        x, y, z, r, pi, w = _yawed(p.pose, yaw)
        tr = f"<transparency>{p.alpha:.2f}</transparency>" if p.alpha > 0 else ""
        if _is_thin(p.geom):
            tr += "<cast_shadows>false</cast_shadows>"
        material = material_xml(p.mat) if p.mat is not None else ""     # None: a mesh keeps its own materials
        out.append(f'<visual name="{p.name}"><pose>{x:.4f} {y:.4f} {z:.4f} {r:.4f} {pi:.4f} {w:.4f}</pose>'
                   f"<geometry>{p.geom}</geometry>{material}{tr}</visual>")
    return "".join(out)


# ---------------------------------------------------------------------------- label textures
# A label is a list of drawing ops on a w x h picture (coordinates as fractions of the size):
#   ("bg", rgb)  ("rect", x0, y0, x1, y1, rgb)  ("circle", cx, cy, r_of_h, rgb)
#   ("text", "WORD", cx, cy, max_w, max_h, rgb[, thickness])   ("barcode", x0, y0, x1, y1)
#   ("lines", x0, y0, x1, y1, n, rgb)  ("arrow", cx, cy, size_of_h, rgb)  ("ring", cx, cy, r, rgb, t)
LABELS: Dict[str, Tuple[int, int, List, bool]] = {}


def label(name: str, w_m: float, h_m: float, ops: List, px_per_m: float = 1600.0, wrap: bool = False) -> str:
    """Register a label texture of w_m x h_m metres; returns its name for mat(tex=...).
    wrap: the label goes around a cylinder (band()), whose texture coordinates run the other way."""
    w = int(max(64, min(1024, round(w_m * px_per_m))))
    h = int(max(64, min(1024, round(h_m * px_per_m))))
    LABELS[name] = (w, h, ops, wrap)
    return name


def texture_path(name: str) -> str:
    """PNG of a registered label; the file name carries a hash of its drawing, so a changed label
    (or product colour) is drawn again instead of reusing an old picture."""
    digest = hashlib.md5(repr((LABELS[name], TEXTURE_VERSION)).encode()).hexdigest()[:8]
    path = os.path.join(TEXTURE_DIR, f"{name}_{digest}.png")
    if not os.path.exists(path):
        draw_label(name, path)
    return path


def _bgr(c: RGB):
    return (int(c[2] * 255), int(c[1] * 255), int(c[0] * 255))


def draw_label(name: str, path: str):
    import cv2
    import numpy as np
    w, h, ops, wrap = LABELS[name]
    img = np.zeros((max(1, h), max(1, w), 3), np.uint8)
    font = cv2.FONT_HERSHEY_DUPLEX
    for op in ops:
        kind = op[0]
        if kind == "file":                        # a photo (custom products): used as it is
            photo = cv2.imread(op[1])
            if photo is not None:
                img = photo
            continue
        if kind == "bg":
            img[:] = _bgr(op[1])
        elif kind == "rect":
            _, x0, y0, x1, y1, c = op
            cv2.rectangle(img, (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h)), _bgr(c), -1, cv2.LINE_AA)
        elif kind == "circle":
            _, cx, cy, r, c = op
            cv2.circle(img, (int(cx * w), int(cy * h)), max(1, int(r * h)), _bgr(c), -1, cv2.LINE_AA)
        elif kind == "ring":
            _, cx, cy, r, c, t = op
            cv2.circle(img, (int(cx * w), int(cy * h)), max(1, int(r * h)), _bgr(c), max(1, int(t * h)), cv2.LINE_AA)
        elif kind == "text":
            text, cx, cy, mw, mh, c = op[1:7]
            thick_f = op[7] if len(op) > 7 else 0.08
            (tw, th), _ = cv2.getTextSize(text, font, 1.0, 2)
            s = min(mw * w / max(1, tw), mh * h / max(1, th))
            t = max(1, int(round(th * s * thick_f)))
            (tw, th), base = cv2.getTextSize(text, font, s, t)
            org = (int(cx * w - tw / 2), int(cy * h + th / 2))
            cv2.putText(img, text, org, font, s, _bgr(c), t, cv2.LINE_AA)
        elif kind == "barcode":
            _, x0, y0, x1, y1 = op
            cv2.rectangle(img, (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h)), (255, 255, 255), -1)
            n = 30
            rng = np.random.default_rng(zlib.crc32(name.encode()))
            bw = (x1 - x0) * w / (n + 4)
            for i in range(n):
                if rng.random() < 0.55:
                    xa = int(x0 * w + (i + 2) * bw)
                    cv2.rectangle(img, (xa, int((y0 + 0.1 * (y1 - y0)) * h)),
                                  (int(xa + bw * rng.uniform(0.4, 1.0)), int((y1 - 0.1 * (y1 - y0)) * h)), (0, 0, 0), -1)
        elif kind == "lines":
            _, x0, y0, x1, y1, n, c = op
            for i in range(n):
                yy = y0 + (y1 - y0) * (i + 0.5) / n
                xe = x1 - (x1 - x0) * (0.3 if i % 3 == 2 else 0.0)
                cv2.line(img, (int(x0 * w), int(yy * h)), (int(xe * w), int(yy * h)), _bgr(c), max(1, h // 120), cv2.LINE_AA)
        elif kind == "arrow":
            _, cx, cy, s, c = op
            hs = s * h
            pts = np.array([[cx * w, cy * h - hs], [cx * w - hs * 0.6, cy * h - hs * 0.2], [cx * w - hs * 0.2, cy * h - hs * 0.2],
                            [cx * w - hs * 0.2, cy * h + hs], [cx * w + hs * 0.2, cy * h + hs], [cx * w + hs * 0.2, cy * h - hs * 0.2],
                            [cx * w + hs * 0.6, cy * h - hs * 0.2]], np.int32)
            cv2.fillPoly(img, [pts], _bgr(c), cv2.LINE_AA)
    if wrap:                                      # a Gazebo cylinder maps the picture upside down
        img = cv2.rotate(img, cv2.ROTATE_180)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cv2.imwrite(path, img)


def ensure_textures(names: Optional[Sequence[str]] = None, prune: bool = False) -> List[str]:
    """Draw the label PNGs (all, or the given ones) that are missing; returns their paths.
    prune: also delete the PNGs in TEXTURE_DIR that no current label uses."""
    for t, spec in PRODUCT_SIZES.items():          # register every product's labels
        product_parts(t, *spec)
    paths = [texture_path(n) for n in (names or list(LABELS))]
    if prune and os.path.isdir(TEXTURE_DIR):
        keep = {os.path.basename(p) for p in (texture_path(n) for n in LABELS)}
        for f in os.listdir(TEXTURE_DIR):
            if f.endswith(".png") and f not in keep:
                os.remove(os.path.join(TEXTURE_DIR, f))
    return paths


# ---------------------------------------------------------------------------- part helpers
WHITE, BLACK, SILVER = (0.95, 0.95, 0.95), (0.05, 0.05, 0.05), (0.78, 0.79, 0.80)


def panel_y(name, tex, w, h, x, z, y_face, side=1) -> Part:
    """Printed panel on the +y (side=1) or -y face at y = y_face, centred at (x, z), w along x."""
    return Part(name, gbox(w, 0.0016, h), (x, side * (y_face + 0.0009), z, 0, 0, 0 if side > 0 else math.pi), mat(tex=tex))


def panel_x(name, tex, w, h, y, z, x_face, side=1) -> Part:
    """Printed panel on the +x (side=1) or -x face at x = x_face, centred at (y, z), w along y."""
    return Part(name, gbox(0.0016, w, h), (side * (x_face + 0.0009), y, z, 0, 0, 0 if side > 0 else math.pi), mat(tex=tex))


def band(name, tex, r, h, z) -> Part:
    """Printed label wrapped around a cylinder of radius r."""
    if tex in LABELS:
        LABELS[tex] = LABELS[tex][:3] + (True,)
    return Part(name, gcyl(r + 0.0012, h), (0, 0, z), mat(tex=tex, rough=0.6))


def boxy(name, sx, sy, sz, z, c, **kw) -> Part:
    return Part(name, gbox(sx, sy, sz), (0, 0, z), mat(c, **kw))


Recipe = Callable[[float, float, float, RGB], List[Part]]
RECIPES: Dict[str, Recipe] = {}


def recipe(name: str):
    def deco(fn: Recipe) -> Recipe:
        RECIPES[name] = fn
        return fn
    return deco


# ============================================================================ boxes
@recipe("shipping_box_large")
def _shipping_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    tape = mat((0.58, 0.42, 0.24), spec=0.45, rough=0.3)
    print_ = label("shipping_side", sx - 0.02, sz - 0.03, [
        ("bg", c), ("arrow", 0.12, 0.30, 0.16, (0.1, 0.08, 0.06)), ("arrow", 0.24, 0.30, 0.16, (0.1, 0.08, 0.06)),
        ("text", "THIS SIDE UP", 0.18, 0.56, 0.30, 0.09, (0.1, 0.08, 0.06)),
        ("rect", 0.52, 0.12, 0.95, 0.72, (0.97, 0.97, 0.95)),
        ("text", "BCR LOGISTICS", 0.735, 0.20, 0.38, 0.08, (0.05, 0.05, 0.05)),
        ("lines", 0.55, 0.28, 0.92, 0.50, 4, (0.15, 0.15, 0.15)),
        ("barcode", 0.56, 0.52, 0.91, 0.68),
        ("rect", 0.05, 0.78, 0.48, 0.94, (0.75, 0.10, 0.08)), ("text", "FRAGILE", 0.265, 0.86, 0.40, 0.12, WHITE),
    ])
    end = label("shipping_end", sy - 0.03, sz - 0.04, [
        ("bg", c), ("ring", 0.5, 0.40, 0.24, (0.1, 0.08, 0.06), 0.03),
        ("text", "RECYCLE", 0.5, 0.40, 0.40, 0.10, (0.1, 0.08, 0.06)),
        ("text", "MAX 20 KG", 0.5, 0.82, 0.6, 0.10, (0.1, 0.08, 0.06))])
    return [
        boxy("body", sx, sy, sz, 0, c, rough=0.95),
        Part("flap_seam", gbox(sx, 0.004, 0.001), (0, 0, hz + 0.0004), mat(dk(c, 0.45))),
        Part("tape_top", gbox(sx + 0.003, 0.055, 0.0012), (0, 0, hz + 0.0012), tape),
        Part("tape_front", gbox(0.0012, 0.055, 0.07), (hx + 0.0009, 0, hz - 0.035), tape),
        Part("tape_back", gbox(0.0012, 0.055, 0.07), (-hx - 0.0009, 0, hz - 0.035), tape),
        panel_y("print_l", print_, sx - 0.02, sz - 0.03, 0, -0.005, hy, 1),
        panel_y("print_r", print_, sx - 0.02, sz - 0.03, 0, -0.005, hy, -1),
        panel_x("end_f", end, sy - 0.03, sz - 0.10, 0, -0.03, hx, 1),
        panel_x("end_b", end, sy - 0.03, sz - 0.10, 0, -0.03, hx, -1),
    ]


@recipe("milk_carton")
def _milk_carton(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz * 0.80
    top = -hz + body_h
    s = sy / math.sqrt(2.0)                       # diamond prism = gable roof, rises sy/2
    front = label("milk_front", sx, body_h - 0.01, [
        ("bg", c), ("rect", 0, 0.62, 1, 1, (0.12, 0.38, 0.80)), ("rect", 0, 0.58, 1, 0.62, (0.45, 0.70, 0.95)),
        ("text", "FARM", 0.5, 0.14, 0.7, 0.10, (0.12, 0.38, 0.80)), ("text", "FRESH", 0.5, 0.27, 0.8, 0.11, (0.12, 0.38, 0.80)),
        ("text", "MILK", 0.5, 0.45, 0.85, 0.16, (0.05, 0.20, 0.55)),
        ("text", "3.5% FAT", 0.5, 0.72, 0.6, 0.07, WHITE), ("text", "1 L", 0.5, 0.86, 0.4, 0.10, WHITE)])
    side = label("milk_side", sy, body_h - 0.01, [
        ("bg", c), ("text", "NUTRITION", 0.5, 0.10, 0.85, 0.06, (0.12, 0.38, 0.80)),
        ("lines", 0.08, 0.17, 0.92, 0.55, 8, (0.25, 0.25, 0.3)), ("barcode", 0.12, 0.62, 0.88, 0.80),
        ("rect", 0, 0.88, 1, 1, (0.12, 0.38, 0.80))])
    cap_n = (0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    cy_, cz_ = sy / 4, top + sy / 4
    return [
        boxy("body", sx, sy, body_h, -hz + body_h / 2, c, rough=0.6),
        Part("roof", gbox(sx, s, s), (0, 0, top, math.pi / 4, 0, 0), mat(c, rough=0.6)),
        Part("fin", gbox(sx, 0.006, hz - (top + sy / 2) + 0.004), (0, 0, (hz + top + sy / 2) / 2 - 0.002, 0, 0, 0), mat(c, rough=0.6)),
        Part("cap", gcyl(0.016, 0.014), (0, cy_ + cap_n[1] * 0.007, cz_ + cap_n[2] * 0.007, -math.pi / 4, 0, 0),
             mat((0.92, 0.94, 0.97), spec=0.3, rough=0.4)),
        panel_y("front", front, sx - 0.004, body_h - 0.01, 0, -hz + body_h / 2, hy, 1),
        panel_y("back", front, sx - 0.004, body_h - 0.01, 0, -hz + body_h / 2, hy, -1),
        panel_x("side_a", side, sy - 0.004, body_h - 0.01, 0, -hz + body_h / 2, hx, 1),
        panel_x("side_b", side, sy - 0.004, body_h - 0.01, 0, -hz + body_h / 2, hx, -1),
    ]


@recipe("bottle_boxy")
def _bottle_boxy(sx, sy, sz, c):
    hz = sz / 2
    body_h, sh_h, neck_h, cap_h = sz * 0.71, sz * 0.10, sz * 0.12, sz * 0.07
    z0 = -hz
    lab_ = label("aqua_label", sx * 2, 0.13, [
        ("bg", (0.96, 0.97, 1.0)), ("rect", 0, 0.75, 1, 1, dk(c, 0.8)),
        ("text", "AQUA PURE", 0.25, 0.30, 0.42, 0.22, dk(c, 0.7)), ("text", "AQUA PURE", 0.75, 0.30, 0.42, 0.22, dk(c, 0.7)),
        ("text", "mineral water 1.5 L", 0.25, 0.58, 0.40, 0.10, (0.2, 0.2, 0.3)),
        ("barcode", 0.62, 0.45, 0.88, 0.70)])
    glass = mat(c, spec=0.8, rough=0.15)
    return [
        Part("body", gbox(sx, sy, body_h), (0, 0, z0 + body_h / 2), glass, alpha=0.25),
        Part("water", gbox(sx * 0.9, sy * 0.9, body_h * 0.9), (0, 0, z0 + body_h * 0.46), mat(lt(c, 0.2), spec=0.6, rough=0.1), alpha=0.3),
        Part("shoulder", gbox(sx * 0.8, sy * 0.8, sh_h), (0, 0, z0 + body_h + sh_h / 2), glass, alpha=0.2),
        Part("neck", gcyl(sx * 0.18, neck_h), (0, 0, z0 + body_h + sh_h + neck_h / 2), glass, alpha=0.15),
        Part("ring", gcyl(sx * 0.22, 0.006), (0, 0, z0 + body_h + sh_h + neck_h * 0.75), mat(dk(c, 0.6), spec=0.3)),
        Part("cap", gcyl(sx * 0.2, cap_h), (0, 0, hz - cap_h / 2), mat(dk(c, 0.55), spec=0.4, rough=0.4)),
        Part("label", gbox(sx + 0.003, sy + 0.003, 0.13), (0, 0, z0 + body_h * 0.45), mat(tex=lab_, rough=0.5)),
    ]


@recipe("sneaker_pair")
def _sneaker_pair(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    w = (sy - 0.01) / 2
    sole_h, toe_h = 0.028, 0.06
    upper = mat(c, spec=0.15, rough=0.7)
    sole = mat((0.93, 0.93, 0.90), spec=0.1)
    out = []
    for i, yc in enumerate((-(w / 2 + 0.005), w / 2 + 0.005)):
        n = f"s{i}"
        out += [
            Part(n + "_sole", gbox(sx - w / 2, w, sole_h), (-w / 4, yc, -hz + sole_h / 2), sole),
            Part(n + "_sole_toe", gcyl(w / 2, sole_h), (hx - w / 2, yc, -hz + sole_h / 2), sole),
            Part(n + "_heel", gbox(0.19, w * 0.96, sz - sole_h), (-hx + 0.095, yc, -hz + sole_h + (sz - sole_h) / 2), upper),
            Part(n + "_vamp", gbox(sx - 0.19 - w / 2, w * 0.96, toe_h), (-hx + 0.19 + (sx - 0.19 - w / 2) / 2, yc, -hz + sole_h + toe_h / 2), upper),
            Part(n + "_toe", gcyl(w * 0.48, toe_h), (hx - w / 2, yc, -hz + sole_h + toe_h / 2), upper),
            Part(n + "_tongue", gbox(0.07, w * 0.5, 0.03), (-hx + 0.215, yc, -hz + sole_h + toe_h + 0.01, 0, -0.35, 0), mat(dk(c, 1.3))),
            Part(n + "_collar", gbox(0.09, w * 0.6, 0.002), (-hx + 0.085, yc, hz + 0.0005), mat((0.02, 0.02, 0.02))),
            Part(n + "_stripe_o", gbox(0.11, 0.0015, 0.018), (-0.02, yc + (w * 0.48 + 0.0008) * (1 if yc > 0 else -1),
                                                              -hz + sole_h + 0.03, 0, -0.35, 0), mat((0.92, 0.92, 0.92))),
            Part(n + "_heel_tab", gbox(0.004, w * 0.4, 0.04), (-hx - 0.001, yc, hz - 0.03), mat((0.85, 0.25, 0.10))),
        ]
        for k in range(4):
            out.append(Part(f"{n}_lace{k}", gbox(0.008, w * 0.55, 0.004),
                            (-hx + 0.20 + k * 0.025, yc, -hz + sole_h + toe_h + 0.002), mat((0.30, 0.30, 0.32))))
    return out


@recipe("packet_pouch")
def _packet_pouch(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    seal_h = 0.03
    body_h = sz - seal_h
    front = label("pouch_front", sx, body_h, [
        ("bg", c), ("rect", 0, 0, 1, 0.12, (0.85, 0.15, 0.10)), ("circle", 0.5, 0.52, 0.22, (0.98, 0.70, 0.20)),
        ("text", "CRUNCHY", 0.5, 0.30, 0.8, 0.10, (0.75, 0.10, 0.08)), ("text", "CORN", 0.5, 0.52, 0.5, 0.12, (0.45, 0.20, 0.05)),
        ("text", "SNACKS", 0.5, 0.74, 0.8, 0.10, (0.75, 0.10, 0.08)), ("text", "150 g", 0.82, 0.92, 0.25, 0.06, (0.3, 0.2, 0.05))])
    return [
        Part("body", gbox(sx - 0.006, sy, body_h - 0.01), (0, 0, -hz + (body_h - 0.01) / 2 + 0.01), mat(c, spec=0.5, rough=0.35)),
        Part("gusset", gbox(sx - 0.02, sy * 0.85, 0.012), (0, 0, -hz + 0.006), mat(dk(c, 0.9), spec=0.4)),
        Part("shoulder", gbox(sx, sy * 0.6, 0.02), (0, 0, -hz + body_h - 0.01), mat(c, spec=0.5, rough=0.35)),
        Part("seal", gbox(sx, 0.008, seal_h), (0, 0, hz - seal_h / 2), mat(dk(c, 0.85), spec=0.4)),
        Part("crimp", gbox(sx + 0.001, 0.009, 0.004), (0, 0, hz - 0.006), mat(dk(c, 0.7))),
        Part("notch", gbox(0.012, 0.010, 0.006), (hx - 0.012, 0, hz - seal_h + 0.004), mat(dk(c, 0.6))),
        panel_y("front", front, sx - 0.012, body_h - 0.02, 0, -hz + body_h / 2 + 0.005, hy, 1),
        panel_y("back", front, sx - 0.012, body_h - 0.02, 0, -hz + body_h / 2 + 0.005, hy, -1),
    ]


@recipe("shoe_box")
def _shoe_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lid_h = 0.045
    lid = label("shoebox_lid", sx - 0.01, lid_h - 0.01, [
        ("bg", c), ("text", "STRIDE", 0.25, 0.5, 0.30, 0.6, WHITE), ("rect", 0.45, 0.45, 0.95, 0.55, (0.95, 0.45, 0.10))])
    endl = label("shoebox_end", sy - 0.04, sz - lid_h - 0.03, [
        ("bg", (0.97, 0.97, 0.95)), ("text", "STRIDE RUNNER 3", 0.5, 0.15, 0.9, 0.12, BLACK),
        ("text", "EU 42  US 8.5", 0.5, 0.38, 0.8, 0.12, BLACK), ("barcode", 0.1, 0.55, 0.9, 0.92)])
    return [
        Part("base", gbox(sx - 0.006, sy - 0.006, sz - 0.01), (0, 0, -0.005), mat(dk(c, 0.92), rough=0.9)),
        Part("lid", gbox(sx, sy, lid_h), (0, 0, hz - lid_h / 2), mat(c, rough=0.85)),
        panel_y("lid_l", lid, sx - 0.01, lid_h - 0.01, 0, hz - lid_h / 2, hy, 1),
        panel_y("lid_r", lid, sx - 0.01, lid_h - 0.01, 0, hz - lid_h / 2, hy, -1),
        panel_x("end", endl, sy - 0.04, sz - lid_h - 0.03, 0, -hz + (sz - lid_h) / 2, hx - 0.003, 1),
    ]


@recipe("long_package")
def _long_package(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    side = label("express_side", sx - 0.02, sz - 0.02, [
        ("bg", c), ("rect", 0, 0.70, 1, 0.80, WHITE), ("rect", 0, 0.84, 1, 0.88, WHITE),
        ("text", "EXPRESS", 0.30, 0.35, 0.50, 0.30, WHITE), ("arrow", 0.70, 0.35, 0.2, WHITE),
        ("rect", 0.80, 0.08, 0.98, 0.62, WHITE), ("barcode", 0.82, 0.38, 0.96, 0.58),
        ("lines", 0.82, 0.10, 0.96, 0.34, 4, (0.2, 0.2, 0.2))])
    tape = mat((0.92, 0.88, 0.80), spec=0.4, rough=0.3)
    return [
        boxy("body", sx, sy, sz, 0, c, rough=0.85),
        Part("flap_a", gbox(0.003, sy, 0.001), (hx - 0.05, 0, hz + 0.0004), mat(dk(c, 0.5))),
        Part("flap_b", gbox(0.003, sy, 0.001), (-hx + 0.05, 0, hz + 0.0004), mat(dk(c, 0.5))),
        Part("tape_a", gbox(0.05, sy + 0.003, 0.0012), (hx - 0.05, 0, hz + 0.0012), tape),
        Part("tape_b", gbox(0.05, sy + 0.003, 0.0012), (-hx + 0.05, 0, hz + 0.0012), tape),
        panel_y("side_l", side, sx - 0.12, sz - 0.02, 0, 0, hy, 1),
        panel_y("side_r", side, sx - 0.12, sz - 0.02, 0, 0, hy, -1),
    ]


@recipe("pickup_box")
def _pickup_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lid_h = 0.03
    plastic = dict(spec=0.35, rough=0.45)
    card = label("pickup_card", 0.12, 0.07, [
        ("bg", (0.98, 0.98, 0.96)), ("text", "PICK-UP", 0.5, 0.22, 0.8, 0.22, BLACK),
        ("text", "ORDER 0417", 0.5, 0.52, 0.85, 0.16, (0.2, 0.2, 0.2)), ("barcode", 0.1, 0.68, 0.9, 0.95)])
    out = [
        Part("tub", gbox(sx - 0.012, sy - 0.012, sz - lid_h), (0, 0, -hz + (sz - lid_h) / 2), mat(dk(c, 0.95), **plastic)),
        Part("lid", gbox(sx, sy, lid_h), (0, 0, hz - lid_h / 2), mat(c, **plastic)),
        Part("lid_rib_a", gbox(sx - 0.03, 0.012, 0.004), (0, sy * 0.36, hz + 0.001), mat(dk(c, 0.8), **plastic)),
        Part("lid_rib_b", gbox(sx - 0.03, 0.012, 0.004), (0, -sy * 0.36, hz + 0.001), mat(dk(c, 0.8), **plastic)),
        Part("handle_a", gbox(0.004, 0.11, 0.03), (hx - 0.0045, 0, hz - lid_h - 0.03), mat((0.08, 0.05, 0.04))),
        Part("handle_b", gbox(0.004, 0.11, 0.03), (-hx + 0.0045, 0, hz - lid_h - 0.03), mat((0.08, 0.05, 0.04))),
        panel_y("card", card, 0.12, 0.07, -0.06, 0.0, hy - 0.006, 1),
    ]
    for k, x in enumerate((-0.12, 0.0, 0.12)):
        if k != 1:
            out.append(Part(f"rib_y{k}", gbox(0.014, sy - 0.004, sz - lid_h - 0.03), (x, 0, -hz + (sz - lid_h) / 2 + 0.005), mat(c, **plastic)))
    return out


@recipe("cereal_box")
def _cereal_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = label("cereal_front", sx, sz, [
        ("bg", c), ("rect", 0, 0, 1, 0.08, (0.80, 0.10, 0.10)),
        ("text", "GOLDEN", 0.5, 0.17, 0.8, 0.08, (0.75, 0.08, 0.08)), ("text", "CRUNCH", 0.5, 0.30, 0.9, 0.12, (0.75, 0.08, 0.08)),
        ("circle", 0.5, 0.66, 0.20, (0.97, 0.97, 0.98)), ("circle", 0.5, 0.62, 0.15, (0.98, 0.80, 0.30)),
        ("circle", 0.42, 0.60, 0.03, (0.85, 0.55, 0.10)), ("circle", 0.56, 0.58, 0.03, (0.85, 0.55, 0.10)),
        ("circle", 0.50, 0.66, 0.03, (0.85, 0.55, 0.10)), ("rect", 0.15, 0.70, 0.85, 0.80, (0.20, 0.35, 0.80)),
        ("text", "WHOLE GRAIN", 0.5, 0.90, 0.7, 0.05, (0.2, 0.1, 0.0)), ("text", "500g", 0.85, 0.95, 0.2, 0.04, (0.2, 0.1, 0.0))])
    side = label("cereal_side", sy, sz, [
        ("bg", lt(c, 0.6)), ("text", "NUTRITION", 0.5, 0.08, 0.9, 0.05, (0.5, 0.2, 0.0)),
        ("lines", 0.1, 0.13, 0.9, 0.60, 12, (0.3, 0.2, 0.1)), ("barcode", 0.1, 0.70, 0.9, 0.85)])
    return [
        boxy("body", sx, sy, sz, 0, c, rough=0.7),
        Part("flap", gbox(sx, 0.003, 0.001), (0, 0, hz + 0.0004), mat(dk(c, 0.55))),
        panel_y("front", front, sx - 0.004, sz - 0.004, 0, 0, hy, 1),
        panel_y("back", front, sx - 0.004, sz - 0.004, 0, 0, hy, -1),
        panel_x("side_a", side, sy - 0.004, sz - 0.004, 0, 0, hx, 1),
        panel_x("side_b", side, sy - 0.004, sz - 0.004, 0, 0, hx, -1),
    ]


@recipe("detergent_bottle")
def _detergent_bottle(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    r = sy / 2
    body_h, sh_h = sz * 0.74, sz * 0.10
    z_body = -hz + body_h / 2
    plastic = dict(spec=0.5, rough=0.3)
    lab_ = label("detergent_label", sx - 2 * r + 0.06, body_h * 0.6, [
        ("bg", WHITE), ("circle", 0.5, 0.35, 0.25, lt(c, 0.4)), ("text", "SPARKLE", 0.5, 0.30, 0.85, 0.16, dk(c, 0.6)),
        ("text", "CLEAN", 0.5, 0.48, 0.6, 0.12, dk(c, 0.6)), ("text", "2 L  40 washes", 0.5, 0.80, 0.8, 0.08, (0.2, 0.2, 0.3))])
    cap_x = hx - r
    return [
        Part("body_mid", gbox(sx - 2 * r, sy, body_h), (0, 0, z_body), mat(c, **plastic)),
        Part("body_end_a", gcyl(r, body_h), (hx - r, 0, z_body), mat(c, **plastic)),
        Part("body_end_b", gcyl(r, body_h), (-hx + r, 0, z_body), mat(c, **plastic)),
        Part("shoulder", gbox(sx - 0.02, sy - 0.02, sh_h), (0, 0, -hz + body_h + sh_h / 2), mat(c, **plastic)),
        Part("handle_post", gbox(0.018, 0.026, sz - body_h - sh_h), (-hx + 0.022, 0, hz - (sz - body_h - sh_h) / 2), mat(c, **plastic)),
        Part("handle_bar", gbox(0.07, 0.026, 0.016), (-hx + 0.05, 0, hz - 0.008), mat(c, **plastic)),
        Part("spout", gcyl(0.024, sz - body_h - sh_h - 0.01), (cap_x - 0.01, 0, -hz + body_h + sh_h + (sz - body_h - sh_h - 0.01) / 2), mat(dk(c, 0.75), **plastic)),
        Part("cap", gcyl(0.027, 0.012), (cap_x - 0.01, 0, hz - 0.016), mat(dk(c, 0.6), spec=0.4)),
        panel_y("label_f", lab_, sx - 2 * r + 0.06, body_h * 0.6, 0, z_body, hy - 0.0, 1),
        panel_y("label_b", lab_, sx - 2 * r + 0.06, body_h * 0.6, 0, z_body, hy - 0.0, -1),
    ]


@recipe("soda_six_pack")
def _soda_six_pack(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    side = label("soda_side", sx - 0.01, sz - 0.02, [
        ("bg", c), ("text", "FIZZ", 0.30, 0.38, 0.40, 0.40, WHITE), ("text", "COLA", 0.30, 0.75, 0.40, 0.25, WHITE),
        ("rect", 0.60, 0.15, 0.95, 0.85, dk(c, 0.75)), ("text", "6 x 330ml", 0.775, 0.5, 0.32, 0.18, WHITE)])
    end = label("soda_end", sy - 0.01, sz - 0.02, [("bg", c), ("text", "6", 0.5, 0.45, 0.4, 0.6, WHITE),
                                                    ("text", "CANS", 0.5, 0.85, 0.7, 0.15, WHITE)])
    out = [
        boxy("carton", sx, sy, sz - 0.004, -0.002, c, rough=0.6, spec=0.3),
        Part("handle_slot", gbox(0.08, 0.022, 0.001), (0, 0, hz - 0.0035), mat(dk(c, 0.3))),
        Part("fold_a", gbox(sx, 0.003, 0.001), (0, sy * 0.25, hz - 0.0035), mat(dk(c, 0.7))),
        Part("fold_b", gbox(sx, 0.003, 0.001), (0, -sy * 0.25, hz - 0.0035), mat(dk(c, 0.7))),
        panel_y("side_l", side, sx - 0.01, sz - 0.02, 0, -0.002, hy, 1),
        panel_y("side_r", side, sx - 0.01, sz - 0.02, 0, -0.002, hy, -1),
        panel_x("end_a", end, sy - 0.04, sz - 0.05, 0, -0.012, hx, 1),
        panel_x("end_b", end, sy - 0.04, sz - 0.05, 0, -0.012, hx, -1),
    ]
    # can rims peek out of the open ends of the carton
    for j, yc in enumerate((-sy / 4, sy / 4)):
        for sgn in (1, -1):
            out.append(Part(f"can_end_{j}_{sgn > 0}", gcyl(0.036, 0.002), (sgn * (hx + 0.0005), yc, hz - 0.03, 0, math.pi / 2, 0),
                            mat(SILVER, metal=0.5, rough=0.35)))
    return out


@recipe("book_parcel")
def _book_parcel(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    twine = mat((0.72, 0.58, 0.38), rough=0.95)
    lab_ = label("parcel_label", 0.13, 0.06, [
        ("bg", (0.98, 0.97, 0.93)), ("rect", 0.72, 0.08, 0.95, 0.55, (0.20, 0.45, 0.75)),
        ("text", "PRINTED MATTER", 0.36, 0.20, 0.66, 0.14, BLACK), ("lines", 0.05, 0.38, 0.65, 0.92, 4, (0.15, 0.15, 0.2))])
    return [
        boxy("body", sx, sy, sz, 0, c, rough=0.95),
        Part("twine_x_top", gbox(sx + 0.002, 0.004, 0.003), (0, 0, hz + 0.0012), twine),
        Part("twine_y_top", gbox(0.004, sy + 0.002, 0.003), (0, 0, hz + 0.0012), twine),
        Part("twine_y_e1", gbox(0.003, 0.004, sz + 0.002), (hx + 0.0012, 0, 0), twine),
        Part("twine_y_e2", gbox(0.003, 0.004, sz + 0.002), (-hx - 0.0012, 0, 0), twine),
        Part("twine_x_s1", gbox(0.004, 0.003, sz + 0.002), (0, hy + 0.0012, 0), twine),
        Part("twine_x_s2", gbox(0.004, 0.003, sz + 0.002), (0, -hy - 0.0012, 0), twine),
        Part("knot", gsph(0.009), (0, 0, hz - 0.005), twine),
        Part("fold_a", gbox(0.003, sy, 0.001), (hx - 0.03, 0, hz + 0.0004), mat(dk(c, 0.75))),
        Part("fold_b", gbox(0.003, sy, 0.001), (-hx + 0.03, 0, hz + 0.0004), mat(dk(c, 0.75))),
        panel_y("label", lab_, 0.11, sz - 0.02, sx * 0.22, 0, hy, 1),
    ]


@recipe("toy_box")
def _toy_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = label("toy_front", sx, sz, [
        ("bg", c), ("text", "BUILD", 0.5, 0.13, 0.7, 0.13, (1.0, 0.85, 0.10)), ("text", "BLOCKS", 0.5, 0.28, 0.8, 0.12, WHITE),
        ("rect", 0.15, 0.45, 0.40, 0.68, (0.95, 0.20, 0.15)), ("rect", 0.42, 0.45, 0.67, 0.68, (0.15, 0.45, 0.95)),
        ("rect", 0.28, 0.70, 0.53, 0.92, (1.0, 0.80, 0.05)), ("rect", 0.55, 0.70, 0.80, 0.92, (0.20, 0.70, 0.30)),
        ("circle", 0.85, 0.16, 0.10, (1.0, 0.85, 0.10)), ("text", "3+", 0.85, 0.16, 0.12, 0.10, c)])
    side = label("toy_side", sy, sz, [("bg", c), ("text", "120 PCS", 0.5, 0.3, 0.8, 0.15, WHITE),
                                      ("rect", 0.1, 0.5, 0.9, 0.9, lt(c, 0.5)), ("text", "AGES 3+", 0.5, 0.7, 0.6, 0.12, c)])
    trim = mat((1.0, 0.82, 0.10), spec=0.2)
    out = [boxy("body", sx, sy, sz, 0, c, rough=0.6, spec=0.25),
           panel_y("front", front, sx - 0.03, sz - 0.03, 0, 0, hy, 1), panel_y("back", front, sx - 0.03, sz - 0.03, 0, 0, hy, -1),
           panel_x("side_a", side, sy - 0.03, sz - 0.03, 0, 0, hx, 1), panel_x("side_b", side, sy - 0.03, sz - 0.03, 0, 0, hx, -1)]
    for i, (x, y) in enumerate(((hx, hy), (hx, -hy), (-hx, hy), (-hx, -hy))):
        out.append(Part(f"trim{i}", gbox(0.012, 0.012, sz), (x - math.copysign(0.005, x), y - math.copysign(0.005, y), 0), trim))
    return out


@recipe("pizza_box")
def _pizza_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    side = label("pizza_side", sx - 0.02, sz - 0.02, [
        ("bg", c), ("text", "HOT & FRESH PIZZA", 0.5, 0.5, 0.80, 0.55, (0.70, 0.10, 0.05))])
    return [
        Part("base", gbox(sx - 0.004, sy - 0.004, sz - 0.006), (0, 0, -0.003), mat(dk(c, 0.95), rough=0.95)),
        Part("lid", gbox(sx, sy, 0.012), (0, 0, hz - 0.006), mat(c, rough=0.95)),
        Part("lid_front", gbox(0.004, sy, sz * 0.6), (hx - 0.002, 0, hz - sz * 0.3), mat(c, rough=0.95)),
        Part("lid_fold", gbox(0.003, sy - 0.02, 0.001), (hx - 0.025, 0, hz + 0.0004), mat(dk(c, 0.6))),
        Part("vent_a", gbox(0.03, 0.008, 0.001), (-hx + 0.04, hy - 0.05, hz + 0.0004), mat(dk(c, 0.3))),
        Part("vent_b", gbox(0.03, 0.008, 0.001), (-hx + 0.04, -hy + 0.05, hz + 0.0004), mat(dk(c, 0.3))),
        panel_y("side_l", side, sx - 0.04, sz - 0.03, 0, -0.005, hy, 1),
        panel_y("side_r", side, sx - 0.04, sz - 0.03, 0, -0.005, hy, -1),
    ]


@recipe("tissue_box")
def _tissue_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz - 0.014
    side = label("tissue_side", sx, body_h, [
        ("bg", c), ("circle", 0.15, 0.3, 0.18, lt(c, 0.5)), ("circle", 0.45, 0.7, 0.22, lt(c, 0.5)),
        ("circle", 0.82, 0.35, 0.16, lt(c, 0.5)), ("text", "SOFTY", 0.5, 0.45, 0.45, 0.30, (0.55, 0.10, 0.35)),
        ("text", "3-ply  80 tissues", 0.5, 0.82, 0.55, 0.12, (0.45, 0.15, 0.30))])
    tissue = mat((0.98, 0.98, 0.99), rough=0.95)
    return [
        boxy("body", sx, sy, body_h, -hz + body_h / 2, c, rough=0.8),
        Part("film", gbox(sx * 0.45, sy * 0.28, 0.001), (0, 0, -hz + body_h + 0.0004), mat(lt(c, 0.6), spec=0.6, rough=0.2)),
        Part("tissue_a", gbox(0.06, 0.002, 0.03), (0, 0.004, -hz + body_h + 0.008, 0.45, 0, 0), tissue),
        Part("tissue_b", gbox(0.05, 0.002, 0.026), (0.006, -0.004, -hz + body_h + 0.007, -0.5, 0, 0.2), tissue),
        panel_y("side_l", side, sx - 0.004, body_h - 0.004, 0, -hz + body_h / 2, hy, 1),
        panel_y("side_r", side, sx - 0.004, body_h - 0.004, 0, -hz + body_h / 2, hy, -1),
    ]


@recipe("gift_box")
def _gift_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    box_h = sz - 0.035                  # the bow sits on top, inside the catalogue height
    top = -hz + box_h
    lid_h = 0.05
    ribbon = mat((0.55, 0.02, 0.22), spec=0.7, rough=0.25)
    rw = 0.03
    out = [
        Part("base", gbox(sx - 0.008, sy - 0.008, box_h - 0.01), (0, 0, -hz + (box_h - 0.01) / 2), mat(c, spec=0.3, rough=0.5)),
        Part("lid", gbox(sx, sy, lid_h), (0, 0, top - lid_h / 2), mat(c, spec=0.3, rough=0.5)),
        Part("rib_x_top", gbox(sx + 0.002, rw, 0.002), (0, 0, top + 0.0008), ribbon),
        Part("rib_y_top", gbox(rw, sy + 0.002, 0.002), (0, 0, top + 0.0008), ribbon),
        Part("rib_x1", gbox(0.002, rw, box_h), (hx + 0.0008, 0, -hz + box_h / 2), ribbon),
        Part("rib_x2", gbox(0.002, rw, box_h), (-hx - 0.0008, 0, -hz + box_h / 2), ribbon),
        Part("rib_y1", gbox(rw, 0.002, box_h), (0, hy + 0.0008, -hz + box_h / 2), ribbon),
        Part("rib_y2", gbox(rw, 0.002, box_h), (0, -hy - 0.0008, -hz + box_h / 2), ribbon),
        Part("knot", gsph(0.014), (0, 0, top + 0.01), ribbon),
    ]
    for i, a in enumerate((0.6, 0.6 + math.pi)):
        out.append(Part(f"loop{i}", gcyl(0.022, 0.012), (0.03 * math.cos(a), 0.03 * math.sin(a), top + 0.02, math.pi / 2, 0.4 * (1 if i else -1), a + math.pi / 2), ribbon))
    for i, a in enumerate((2.3, -0.9)):
        out.append(Part(f"tail{i}", gbox(0.05, 0.014, 0.002), (0.035 * math.cos(a), 0.035 * math.sin(a), top + 0.003, 0, 0, a), ribbon))
    return out


@recipe("tool_case")
def _tool_case(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    d = sy - 0.03                                  # the handle on the front takes 3 cm
    yb = -(sy - d) / 2
    plastic = dict(spec=0.4, rough=0.4)
    black = mat((0.06, 0.06, 0.06), spec=0.3, rough=0.5)
    out = [
        Part("lower", gbox(sx, d, sz * 0.58), (0, yb, -hz + sz * 0.29), mat(c, **plastic)),
        Part("upper", gbox(sx - 0.008, d - 0.008, sz * 0.42), (0, yb, hz - sz * 0.21), mat(c, **plastic)),
        Part("seam", gbox(sx + 0.002, d + 0.002, 0.008), (0, yb, -hz + sz * 0.58), black),
        Part("grip", gcyl(0.012, 0.13), (0, hy - 0.014, -hz + sz * 0.58, 0, math.pi / 2, 0), black),
        Part("grip_post_a", gbox(0.014, 0.03, 0.016), (0.06, hy - 0.016, -hz + sz * 0.58), black),
        Part("grip_post_b", gbox(0.014, 0.03, 0.016), (-0.06, hy - 0.016, -hz + sz * 0.58), black),
        Part("latch_a", gbox(0.035, 0.008, 0.03), (0.14, yb + d / 2 + 0.003, -hz + sz * 0.58), black),
        Part("latch_b", gbox(0.035, 0.008, 0.03), (-0.14, yb + d / 2 + 0.003, -hz + sz * 0.58), black),
        Part("hinge", gbox(sx - 0.06, 0.008, 0.012), (0, yb - d / 2 - 0.003, -hz + sz * 0.58), black),
    ]
    for k, y in enumerate((-0.05, 0.0, 0.05)):
        out.append(Part(f"top_rib{k}", gbox(sx - 0.06, 0.006, 0.003), (0, yb + y * d / 0.17, hz + 0.0012), mat(dk(c, 0.85), **plastic)))
    return out


@recipe("laptop_box")
def _laptop_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    side = label("laptop_side", sx - 0.02, sz - 0.02, [
        ("bg", (0.15, 0.15, 0.17)), ("text", "ULTRABOOK 14", 0.30, 0.5, 0.5, 0.45, (0.92, 0.92, 0.95)),
        ("rect", 0.70, 0.25, 0.95, 0.75, (0.92, 0.92, 0.95)), ("barcode", 0.72, 0.30, 0.93, 0.70)])
    line = mat(lt(c, 0.25), spec=0.6, rough=0.2)
    w, h, t = sx * 0.55, sy * 0.5, 0.004
    return [
        boxy("body", sx, sy, sz, 0, c, spec=0.4, rough=0.35),
        Part("emb_a", gbox(w, t, 0.001), (0, h / 2, hz + 0.0004), line), Part("emb_b", gbox(w, t, 0.001), (0, -h / 2, hz + 0.0004), line),
        Part("emb_c", gbox(t, h, 0.001), (w / 2, 0, hz + 0.0004), line), Part("emb_d", gbox(t, h, 0.001), (-w / 2, 0, hz + 0.0004), line),
        panel_y("side_l", side, sx - 0.02, sz - 0.02, 0, 0, hy, 1),
        panel_y("side_r", side, sx - 0.02, sz - 0.02, 0, 0, hy, -1),
        Part("pull_tab", gbox(0.002, 0.04, 0.02), (hx + 0.001, 0, 0), mat((0.85, 0.15, 0.10))),
    ]


@recipe("pet_food_bag")
def _pet_food_bag(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz - 0.05
    front = label("petfood_front", sx, body_h, [
        ("bg", c), ("rect", 0, 0.78, 1, 1, (0.95, 0.75, 0.10)),
        ("text", "HAPPY", 0.5, 0.10, 0.7, 0.09, WHITE), ("text", "PAWS", 0.5, 0.22, 0.8, 0.11, WHITE),
        ("circle", 0.5, 0.52, 0.11, (0.95, 0.75, 0.10)), ("circle", 0.38, 0.38, 0.04, (0.95, 0.75, 0.10)),
        ("circle", 0.46, 0.34, 0.04, (0.95, 0.75, 0.10)), ("circle", 0.54, 0.34, 0.04, (0.95, 0.75, 0.10)),
        ("circle", 0.62, 0.38, 0.04, (0.95, 0.75, 0.10)),
        ("text", "DOG FOOD", 0.5, 0.86, 0.7, 0.07, dk(c, 0.5)), ("text", "10 kg", 0.5, 0.95, 0.3, 0.05, dk(c, 0.5))])
    return [
        Part("body", gbox(sx, sy - 0.01, body_h), (0, 0, -hz + body_h / 2), mat(c, spec=0.35, rough=0.5)),
        Part("bottom", gbox(sx - 0.01, sy, 0.03), (0, 0, -hz + 0.015), mat(c, spec=0.35, rough=0.5)),
        Part("fold", gbox(sx, sy * 0.45, 0.05), (0, 0, hz - 0.025), mat(dk(c, 0.9), spec=0.35, rough=0.5)),
        Part("fold_top", gbox(sx + 0.002, 0.012, 0.012), (0, 0, hz - 0.006), mat(dk(c, 0.75))),
        Part("handle", gbox(0.07, 0.004, 0.018), (0, sy * 0.225 + 0.002, hz - 0.02), mat((0.15, 0.15, 0.15))),
        panel_y("front", front, sx - 0.006, body_h - 0.02, 0, -hz + body_h / 2, hy - 0.005, 1),
        panel_y("back", front, sx - 0.006, body_h - 0.02, 0, -hz + body_h / 2, hy - 0.005, -1),
    ]


@recipe("kettle_box")
def _kettle_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = label("kettle_front", sx, sz, [
        ("bg", c), ("rect", 0, 0.82, 1, 1, dk(c, 0.5)), ("circle", 0.5, 0.48, 0.22, (0.85, 0.86, 0.88)),
        ("rect", 0.36, 0.22, 0.64, 0.30, (0.85, 0.86, 0.88)), ("ring", 0.73, 0.48, 0.12, (0.25, 0.25, 0.28), 0.03),
        ("rect", 0.36, 0.67, 0.64, 0.72, (0.25, 0.25, 0.28)), ("text", "1.7 L", 0.5, 0.48, 0.20, 0.08, (0.25, 0.25, 0.28)),
        ("text", "ELECTRIC KETTLE", 0.5, 0.91, 0.85, 0.09, WHITE), ("text", "BREWMASTER", 0.5, 0.10, 0.8, 0.08, dk(c, 0.45))])
    side = label("kettle_side", sy, sz, [("bg", c), ("text", "2200 W", 0.5, 0.25, 0.6, 0.12, dk(c, 0.45)),
                                         ("lines", 0.1, 0.40, 0.9, 0.75, 6, dk(c, 0.45)), ("barcode", 0.15, 0.80, 0.85, 0.95)])
    return [
        boxy("body", sx, sy, sz, 0, c, rough=0.6, spec=0.25),
        Part("strap", gbox(0.03, sy + 0.003, 0.0015), (0, 0, hz + 0.0008), mat(dk(c, 0.8), spec=0.4)),
        Part("strap_a", gbox(0.03, 0.0015, 0.04), (0, hy + 0.0015, hz - 0.02), mat(dk(c, 0.8), spec=0.4)),
        Part("strap_b", gbox(0.03, 0.0015, 0.04), (0, -hy - 0.0015, hz - 0.02), mat(dk(c, 0.8), spec=0.4)),
        panel_y("front", front, sx - 0.01, sz - 0.05, 0, -0.02, hy, 1),
        panel_y("back", front, sx - 0.01, sz - 0.05, 0, -0.02, hy, -1),
        panel_x("side_a", side, sy - 0.01, sz - 0.05, 0, -0.02, hx, 1),
        panel_x("side_b", side, sy - 0.01, sz - 0.05, 0, -0.02, hx, -1),
    ]


@recipe("electronics_box")
def _electronics_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lid_h = 0.045
    side = label("speaker_side", sx - 0.02, sz - 0.02, [
        ("bg", c), ("circle", 0.15, 0.5, 0.35, (0.10, 0.10, 0.12)), ("ring", 0.15, 0.5, 0.25, (0.35, 0.35, 0.40), 0.05),
        ("text", "SMART SPEAKER", 0.60, 0.40, 0.62, 0.28, WHITE), ("text", "wi-fi  bluetooth", 0.60, 0.75, 0.45, 0.16, lt(c, 0.6))])
    seal = label("seal_sticker", 0.04, 0.04, [("bg", (0.85, 0.85, 0.88)), ("ring", 0.5, 0.5, 0.35, (0.4, 0.4, 0.45), 0.08),
                                              ("text", "SEALED", 0.5, 0.5, 0.75, 0.2, (0.3, 0.3, 0.35))])
    return [
        Part("base", gbox(sx - 0.006, sy - 0.006, sz - 0.01), (0, 0, -0.005), mat((0.92, 0.92, 0.92), rough=0.7)),
        Part("lid", gbox(sx, sy, lid_h), (0, 0, hz - lid_h / 2), mat(c, spec=0.35, rough=0.4)),
        panel_y("side_l", side, sx - 0.02, lid_h - 0.008, 0, hz - lid_h / 2, hy, 1),
        panel_y("side_r", side, sx - 0.02, lid_h - 0.008, 0, hz - lid_h / 2, hy, -1),
        Part("seal", gbox(0.0016, 0.03, 0.05), (hx + 0.0009, 0, hz - lid_h + 0.004), mat(tex=seal)),
    ]


# ============================================================================ cylinders
@recipe("paint_can")
def _paint_can(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    metal = dict(spec=0.6, metal=0.45, rough=0.35)
    lab_ = label("paint_label", 2 * math.pi * r, 0.12, [
        ("bg", WHITE), ("rect", 0, 0, 1, 0.22, (0.15, 0.25, 0.60)), ("text", "PRO COAT", 0.25, 0.11, 0.25, 0.14, WHITE),
        ("text", "PRO COAT", 0.75, 0.11, 0.25, 0.14, WHITE),
        ("rect", 0.05, 0.35, 0.20, 0.85, (0.85, 0.25, 0.20)), ("rect", 0.55, 0.35, 0.70, 0.85, (0.85, 0.25, 0.20)),
        ("text", "INTERIOR WALL", 0.36, 0.50, 0.26, 0.12, (0.15, 0.25, 0.60)),
        ("text", "2.5 L  MATT", 0.36, 0.72, 0.20, 0.12, (0.2, 0.2, 0.2)),
        ("text", "INTERIOR WALL", 0.86, 0.50, 0.26, 0.12, (0.15, 0.25, 0.60))])
    return [
        Part("body", gcyl(r - 0.002, sz - 0.012), (0, 0, -0.002), mat(c, **metal)),
        Part("rim_top", gcyl(r, 0.008), (0, 0, hz - 0.006), mat(c, spec=0.5, metal=0.3, rough=0.4)),
        Part("lid", gcyl(r - 0.012, 0.0015), (0, 0, hz - 0.0015), mat(dk(c, 0.92), spec=0.5, metal=0.3, rough=0.4)),
        Part("rim_bottom", gcyl(r, 0.006), (0, 0, -hz + 0.003), mat(c, **metal)),
        band("label", lab_, r - 0.002, 0.12, -0.01),
        Part("ear_a", gbox(0.022, 0.006, 0.028), (0, r - 0.001, hz - 0.03), mat(c, **metal)),
        Part("ear_b", gbox(0.022, 0.006, 0.028), (0, -r + 0.001, hz - 0.03), mat(c, **metal)),
        Part("handle", gcyl(0.0018, 2 * r - 0.004), (0.02, 0, hz - 0.002, math.pi / 2, 0, 0), mat((0.6, 0.6, 0.62), metal=0.6, rough=0.3)),
    ]


@recipe("glass_jar")
def _glass_jar(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    body_h, neck_h, lid_h = sz - 0.03, 0.012, 0.018
    lab_ = label("jam_label", 2 * math.pi * r, 0.05, [
        ("bg", (0.98, 0.95, 0.85)), ("text", "HOMEMADE", 0.25, 0.32, 0.30, 0.32, (0.55, 0.08, 0.12)),
        ("text", "STRAWBERRY JAM", 0.25, 0.72, 0.36, 0.26, (0.3, 0.2, 0.1)),
        ("text", "HOMEMADE", 0.75, 0.32, 0.30, 0.32, (0.55, 0.08, 0.12)), ("text", "STRAWBERRY JAM", 0.75, 0.72, 0.36, 0.26, (0.3, 0.2, 0.1))])
    return [
        Part("glass", gcyl(r, body_h), (0, 0, -hz + body_h / 2), mat(lt(c, 0.5), spec=0.9, rough=0.05), alpha=0.6),
        Part("jam", gcyl(r - 0.005, body_h - 0.02), (0, 0, -hz + body_h / 2 - 0.005), mat((0.55, 0.05, 0.12), spec=0.5, rough=0.2)),
        Part("neck", gcyl(r - 0.006, neck_h), (0, 0, -hz + body_h + neck_h / 2), mat(lt(c, 0.5), spec=0.9, rough=0.05), alpha=0.5),
        Part("lid", gcyl(r - 0.002, lid_h), (0, 0, hz - lid_h / 2), mat(c, spec=0.6, metal=0.25, rough=0.35)),
        Part("lid_top", gcyl(r - 0.008, 0.0015), (0, 0, hz + 0.0002), mat(lt(c, 0.25), spec=0.6, metal=0.2, rough=0.35)),
        band("label", lab_, r, 0.05, -hz + body_h * 0.45),
    ]


@recipe("poster_tube")
def _poster_tube(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    cap_h = 0.014
    lab_ = label("tube_label", 2 * math.pi * r, 0.25, [
        ("bg", c), ("rect", 0.06, 0.30, 0.29, 0.70, (0.99, 0.93, 0.62)),
        ("text", "POSTER", 0.175, 0.36, 0.22, 0.10, (0.75, 0.10, 0.08)), ("text", "DO NOT BEND", 0.175, 0.50, 0.23, 0.07, BLACK),
        ("barcode", 0.07, 0.58, 0.28, 0.72), ("arrow", 0.70, 0.5, 0.25, dk(c, 0.5))])
    return [
        Part("tube", gcyl(r - 0.002, sz - 2 * cap_h), (0, 0, 0), mat(c, rough=0.9)),
        Part("cap_top", gcyl(r, cap_h), (0, 0, hz - cap_h / 2), mat(c, spec=0.05, rough=0.8)),
        Part("cap_top_grip", gcyl(r * 0.55, 0.002), (0, 0, hz + 0.0005), mat(dk(c, 0.92), spec=0.05, rough=0.8)),
        Part("cap_bottom", gcyl(r, cap_h), (0, 0, -hz + cap_h / 2), mat(dk(c, 0.82), spec=0.3, rough=0.5)),
        band("label", lab_, r - 0.002, 0.25, 0.0),
    ]


@recipe("water_jug")
def _water_jug(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    body_h, sh1, sh2, neck, cap = sz * 0.69, sz * 0.09, sz * 0.08, sz * 0.09, sz * 0.05
    plastic = mat(lt(c, 0.3), spec=0.9, rough=0.1)
    lab_ = label("jug_label", 2 * math.pi * r, 0.07, [
        ("bg", (0.95, 0.97, 1.0)), ("text", "SPRING WATER", 0.25, 0.35, 0.35, 0.35, dk(c, 0.6)),
        ("text", "19 L", 0.25, 0.78, 0.12, 0.30, dk(c, 0.6)), ("text", "SPRING WATER", 0.75, 0.35, 0.35, 0.35, dk(c, 0.6)),
        ("text", "19 L", 0.75, 0.78, 0.12, 0.30, dk(c, 0.6))])
    z = -hz
    out = [
        Part("body", gcyl(r, body_h), (0, 0, z + body_h / 2), plastic, alpha=0.4),
        Part("water", gcyl(r - 0.008, body_h - 0.01), (0, 0, z + body_h / 2), mat(c, spec=0.6, rough=0.1), alpha=0.25),
        Part("shoulder1", gcyl(r * 0.85, sh1), (0, 0, z + body_h + sh1 / 2), plastic, alpha=0.35),
        Part("shoulder2", gcyl(r * 0.6, sh2), (0, 0, z + body_h + sh1 + sh2 / 2), plastic, alpha=0.3),
        Part("neck", gcyl(r * 0.28, neck), (0, 0, z + body_h + sh1 + sh2 + neck / 2), plastic, alpha=0.2),
        Part("cap", gcyl(r * 0.32, cap), (0, 0, hz - cap / 2), mat(dk(c, 0.75), spec=0.4, rough=0.3)),
        band("label", lab_, r, 0.07, z + body_h * 0.62),
    ]
    for k, f in enumerate((0.12, 0.35)):
        out.append(Part(f"rib{k}", gcyl(r + 0.0015, 0.008), (0, 0, z + body_h * f), mat(c, spec=0.8, rough=0.15), alpha=0.2))
    return out


@recipe("coffee_can")
def _coffee_can(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    lab_ = label("coffee_label", 2 * math.pi * r, sz - 0.03, [
        ("bg", c), ("rect", 0, 0.70, 1, 0.78, (0.85, 0.65, 0.25)),
        ("text", "DARK ROAST", 0.25, 0.30, 0.38, 0.16, (0.95, 0.85, 0.60)), ("text", "COFFEE", 0.25, 0.52, 0.30, 0.16, WHITE),
        ("circle", 0.75, 0.40, 0.18, dk(c, 0.6)), ("text", "500 g", 0.75, 0.40, 0.10, 0.10, (0.95, 0.85, 0.60)),
        ("text", "ground", 0.25, 0.88, 0.2, 0.08, (0.95, 0.85, 0.60))])
    return [
        Part("can", gcyl(r - 0.0015, sz - 0.016), (0, 0, -0.004), mat(SILVER, spec=0.6, metal=0.5, rough=0.35)),
        band("label", lab_, r - 0.0015, sz - 0.03, -0.006),
        Part("lid", gcyl(r, 0.012), (0, 0, hz - 0.006), mat(c, spec=0.5, rough=0.3)),
        Part("lid_rim", gcyl(r * 0.8, 0.0015), (0, 0, hz + 0.0002), mat(dk(c, 0.85), spec=0.5, rough=0.3)),
        Part("bottom", gcyl(r - 0.003, 0.004), (0, 0, -hz + 0.002), mat(SILVER, spec=0.6, metal=0.5, rough=0.35)),
    ]


@recipe("flower_pot")
def _flower_pot(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    clay = dict(spec=0.05, rough=0.95)
    rim_h, h1, h2, h3 = sz * 0.19, sz * 0.28, sz * 0.28, sz * 0.25
    z = -hz
    return [
        Part("saucer", gcyl(r * 0.80, 0.012), (0, 0, z + 0.006), mat(dk(c, 0.9), **clay)),
        Part("body3", gcyl(r * 0.70, h3), (0, 0, z + h3 / 2), mat(c, **clay)),
        Part("body2", gcyl(r * 0.78, h2), (0, 0, z + h3 + h2 / 2), mat(c, **clay)),
        Part("body1", gcyl(r * 0.85, h1), (0, 0, z + h3 + h2 + h1 / 2), mat(c, **clay)),
        Part("rim", gcyl(r, rim_h), (0, 0, hz - rim_h / 2), mat(c, **clay)),
        Part("inside", gcyl(r * 0.86, 0.0015), (0, 0, hz + 0.0003), mat(dk(c, 0.8), **clay)),
        Part("drain", gcyl(r * 0.12, 0.0015), (0, 0, hz + 0.0008), mat(dk(c, 0.55), **clay)),
    ]


@recipe("candle_jar")
def _candle_jar(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    wax_h = sz * 0.78
    lab_ = label("candle_label", 2 * math.pi * r, 0.045, [
        ("bg", (0.20, 0.18, 0.16)), ("text", "VANILLA", 0.25, 0.40, 0.25, 0.40, (0.95, 0.88, 0.70)),
        ("text", "soy candle", 0.25, 0.80, 0.22, 0.22, (0.8, 0.75, 0.65)),
        ("text", "VANILLA", 0.75, 0.40, 0.25, 0.40, (0.95, 0.88, 0.70))])
    return [
        Part("glass", gcyl(r, sz), (0, 0, 0), mat((0.92, 0.90, 0.85), spec=0.9, rough=0.05), alpha=0.65),
        Part("wax", gcyl(r - 0.004, wax_h), (0, 0, -hz + 0.006 + wax_h / 2), mat(c, spec=0.2, rough=0.6)),
        Part("wax_top", gcyl(r - 0.005, 0.002), (0, 0, -hz + 0.006 + wax_h + 0.001), mat(lt(c, 0.2), spec=0.2, rough=0.6)),
        Part("wick", gcyl(0.0015, 0.012), (0, 0, -hz + 0.006 + wax_h + 0.006), mat((0.08, 0.06, 0.05))),
        Part("rim", gcyl(r + 0.0005, 0.004), (0, 0, hz - 0.002), mat((0.85, 0.85, 0.82), spec=0.9, rough=0.05), alpha=0.4),
        band("label", lab_, r, 0.045, -hz + sz * 0.38),
    ]


# ============================================================================ spheres
@recipe("basketball")
def _basketball(sx, sy, sz, c):
    r = sx / 2
    seam = mat((0.06, 0.05, 0.05), rough=0.9)
    rs = math.sqrt(max(0.0, r * r - (0.55 * r) ** 2))
    return [
        Part("ball", gsph(r), (0, 0, 0), mat(c, spec=0.15, rough=0.9)),
        Part("seam_eq", gcyl(r + 0.0006, 0.004), (0, 0, 0), seam),
        Part("seam_mer", gcyl(r + 0.0006, 0.004), (0, 0, 0, 0, math.pi / 2, 0), seam),
        Part("seam_c1", gcyl(rs + 0.0015, 0.004), (0.55 * r, 0, 0, math.pi / 2, 0, math.pi / 2), seam),
        Part("seam_c2", gcyl(rs + 0.0015, 0.004), (-0.55 * r, 0, 0, math.pi / 2, 0, math.pi / 2), seam),
    ]


def _icosahedron_dirs():
    p = (1 + 5 ** 0.5) / 2
    v = [(0, 1, p), (0, -1, p), (0, 1, -p), (0, -1, -p), (1, p, 0), (-1, p, 0), (1, -p, 0), (-1, -p, 0),
         (p, 0, 1), (-p, 0, 1), (p, 0, -1), (-p, 0, -1)]
    # turn so that a face (not a vertex) points up: the middle of the top stays white
    face = [(v[0][i] + v[1][i] + v[8][i]) / 3 for i in range(3)]
    n = math.sqrt(sum(f * f for f in face))
    fz = [f / n for f in face]
    ax = (fz[1], -fz[0], 0.0)                      # rotate fz onto +z
    s = math.sqrt(ax[0] ** 2 + ax[1] ** 2)
    ang = math.acos(fz[2])
    ux, uy = ax[0] / s, ax[1] / s
    ca, sa = math.cos(ang), math.sin(ang)
    out = []
    for x, y, z in v:
        k = math.sqrt(x * x + y * y + z * z)
        x, y, z = x / k, y / k, z / k
        d = ux * x + uy * y
        rx = x * ca + (uy * z) * sa + ux * d * (1 - ca)
        ry = y * ca + (-ux * z) * sa + uy * d * (1 - ca)
        rz = z * ca + (ux * y - uy * x) * sa
        out.append((rx, ry, rz))
    return out


@recipe("football")
def _football(sx, sy, sz, c):
    r = sx / 2
    patch = mat((0.06, 0.06, 0.07), spec=0.3, rough=0.5)
    out = [Part("ball", gsph(r), (0, 0, 0), mat(c, spec=0.3, rough=0.5))]
    for i, (x, y, z) in enumerate(_icosahedron_dirs()):
        pitch = math.acos(max(-1.0, min(1.0, z)))
        yaw = math.atan2(y, x)
        d = r - 0.0035
        out.append(Part(f"patch{i}", gcyl(r * 0.30, 0.008), (x * d, y * d, z * d, 0, pitch, yaw), patch))
    return out


# ============================================================================ new products
@recipe("wine_bottle")
def _wine_bottle(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    body_h, neck_h, foil_h = sz * 0.62, sz * 0.26, sz * 0.12
    glass = mat(c, spec=0.9, rough=0.08)
    lab_ = label("wine_label", 2 * math.pi * r, 0.09, [
        ("bg", (0.96, 0.93, 0.84)), ("rect", 0, 0, 0.5, 0.06, (0.55, 0.42, 0.15)), ("rect", 0, 0.94, 0.5, 1, (0.55, 0.42, 0.15)),
        ("text", "VALLEE", 0.25, 0.25, 0.30, 0.16, (0.45, 0.06, 0.12)), ("text", "ROUGE", 0.25, 0.46, 0.30, 0.16, (0.45, 0.06, 0.12)),
        ("text", "2019", 0.25, 0.72, 0.12, 0.14, (0.3, 0.25, 0.15)), ("lines", 0.58, 0.2, 0.92, 0.8, 6, (0.3, 0.25, 0.15))])
    z = -hz
    return [
        Part("body", gcyl(r, body_h), (0, 0, z + body_h / 2), glass, alpha=0.1),
        Part("wine", gcyl(r - 0.004, body_h - 0.01), (0, 0, z + body_h / 2), mat((0.30, 0.03, 0.08), spec=0.3, rough=0.2)),
        Part("shoulder", gsph(r), (0, 0, z + body_h), glass, alpha=0.1),
        Part("neck", gcyl(r * 0.30, neck_h), (0, 0, z + body_h + neck_h / 2), glass, alpha=0.1),
        Part("foil", gcyl(r * 0.33, foil_h), (0, 0, hz - foil_h / 2), mat((0.45, 0.06, 0.14), spec=0.7, metal=0.4, rough=0.3)),
        band("label", lab_, r, 0.09, z + body_h * 0.48),
    ]


@recipe("olive_oil_bottle")
def _olive_oil_bottle(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h, sh_h, neck_h, cap_h = sz * 0.74, sz * 0.08, sz * 0.10, sz * 0.08
    glass = mat((0.80, 0.86, 0.78), spec=0.9, rough=0.05)
    lab_ = label("olive_label", sx, body_h * 0.55, [
        ("bg", dk(c, 0.75)), ("rect", 0, 0, 1, 0.12, (0.85, 0.72, 0.30)), ("rect", 0, 0.88, 1, 1, (0.85, 0.72, 0.30)),
        ("text", "EXTRA", 0.5, 0.28, 0.7, 0.10, (0.97, 0.93, 0.75)), ("text", "VIRGIN", 0.5, 0.42, 0.8, 0.10, (0.97, 0.93, 0.75)),
        ("circle", 0.5, 0.60, 0.06, (0.85, 0.72, 0.30)), ("text", "OLIVE OIL", 0.5, 0.76, 0.85, 0.09, (0.97, 0.93, 0.75))])
    z = -hz
    return [
        Part("glass", gbox(sx, sy, body_h), (0, 0, z + body_h / 2), glass, alpha=0.55),
        # the bottle is full: the oil's top shows through the glass top, so the top view stays olive
        Part("oil", gbox(sx - 0.008, sy - 0.008, body_h - 0.003), (0, 0, z + (body_h - 0.003) / 2 + 0.0045), mat(c, spec=0.2, rough=0.4)),
        Part("shoulder", gcyl(sx * 0.35, sh_h), (0, 0, z + body_h + sh_h / 2), mat(c, spec=0.3, rough=0.2), alpha=0.25),
        Part("neck", gcyl(sx * 0.16, neck_h), (0, 0, z + body_h + sh_h + neck_h / 2), mat(c, spec=0.3, rough=0.2), alpha=0.25),
        Part("cap", gcyl(sx * 0.19, cap_h), (0, 0, hz - cap_h / 2), mat(dk(c, 0.9), spec=0.2, metal=0.5, rough=0.3)),
        panel_y("label_f", lab_, sx - 0.01, body_h * 0.55, 0, z + body_h * 0.45, hy, 1),
        panel_y("label_b", lab_, sx - 0.01, body_h * 0.55, 0, z + body_h * 0.45, hy, -1),
    ]


@recipe("ice_cream_tub")
def _ice_cream_tub(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    lid_h = 0.02
    lab_ = label("icecream_label", 2 * math.pi * r, sz - lid_h - 0.01, [
        ("bg", (0.98, 0.97, 0.95)), ("rect", 0, 0, 1, 0.2, c), ("rect", 0, 0.85, 1, 1, (0.30, 0.18, 0.10)),
        ("text", "MINT CHOC CHIP", 0.25, 0.42, 0.40, 0.18, (0.30, 0.18, 0.10)), ("circle", 0.75, 0.50, 0.25, c),
        ("circle", 0.72, 0.45, 0.04, (0.30, 0.18, 0.10)), ("circle", 0.79, 0.55, 0.04, (0.30, 0.18, 0.10)),
        ("text", "1 L ice cream", 0.25, 0.68, 0.30, 0.12, (0.4, 0.4, 0.4))])
    return [
        Part("tub", gcyl(r * 0.94, sz - lid_h), (0, 0, -hz + (sz - lid_h) / 2), mat((0.98, 0.97, 0.95), spec=0.3, rough=0.4)),
        band("label", lab_, r * 0.94, sz - lid_h - 0.01, -hz + (sz - lid_h) / 2),
        Part("lid", gcyl(r, lid_h), (0, 0, hz - lid_h / 2), mat(c, spec=0.4, rough=0.35)),
        Part("lid_ring", gcyl(r * 0.82, 0.0015), (0, 0, hz + 0.0002), mat(dk(c, 0.9), spec=0.4, rough=0.35)),
    ]


@recipe("egg_carton")
def _egg_carton(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    pulp = dict(spec=0.03, rough=1.0)
    base_h, lid_h = sz * 0.55, sz * 0.45
    lab_ = label("egg_label", sx * 0.7, lid_h - 0.006, [
        ("bg", (0.98, 0.96, 0.90)), ("text", "FREE RANGE EGGS", 0.40, 0.5, 0.65, 0.5, (0.45, 0.25, 0.08)),
        ("circle", 0.88, 0.5, 0.35, (0.95, 0.80, 0.55))])
    out = [
        Part("base", gbox(sx - 0.012, sy - 0.014, base_h), (0, 0, -hz + base_h / 2), mat(dk(c, 0.92), **pulp)),
        Part("lid", gbox(sx, sy, lid_h), (0, 0, hz - lid_h / 2), mat(c, **pulp)),
        panel_y("label_f", lab_, sx * 0.7, lid_h - 0.008, 0, hz - lid_h / 2, hy, 1),
        Part("tab_a", gbox(0.03, 0.006, 0.014), (0.08, hy - 0.003, hz - lid_h - 0.004), mat(c, **pulp)),
        Part("tab_b", gbox(0.03, 0.006, 0.014), (-0.08, hy - 0.003, hz - lid_h - 0.004), mat(c, **pulp)),
    ]
    for k in range(6):
        x = -hx + sx * (k + 0.5) / 6
        out.append(Part(f"top_dent{k}", gbox(0.006, sy * 0.7, 0.001), (x + sx / 12, 0, hz + 0.0004), mat(dk(c, 0.85), **pulp)) if k < 5 else
                   Part(f"top_dent{k}", gbox(sx * 0.9, 0.006, 0.001), (0, 0, hz + 0.0004), mat(dk(c, 0.85), **pulp)))
        for s_ in (1, -1):
            out.append(Part(f"cup{k}_{s_ > 0}", gsph(min(0.024, base_h * 0.55)), (x, s_ * (hy - 0.026), -hz + 0.022), mat(dk(c, 0.92), **pulp)))
    return out


@recipe("motor_oil_jug")
def _motor_oil_jug(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz * 0.78
    plastic = dict(spec=0.5, rough=0.35)
    lab_ = label("oil_label", sx - 0.03, body_h * 0.62, [
        ("bg", (0.10, 0.10, 0.10)), ("rect", 0, 0.70, 1, 0.78, (0.85, 0.65, 0.10)),
        ("text", "TURBO", 0.5, 0.18, 0.8, 0.14, (0.85, 0.65, 0.10)), ("text", "SYNTH", 0.5, 0.36, 0.7, 0.12, WHITE),
        ("text", "5W-30", 0.5, 0.56, 0.6, 0.14, WHITE), ("text", "MOTOR OIL  4 L", 0.5, 0.90, 0.85, 0.09, WHITE)])
    z = -hz
    return [
        Part("body", gbox(sx, sy, body_h), (0, 0, z + body_h / 2), mat(c, **plastic)),
        Part("shoulder", gbox(sx - 0.03, sy - 0.01, sz * 0.08), (0.015, 0, z + body_h + sz * 0.04), mat(c, **plastic)),
        Part("handle_post", gbox(0.03, sy * 0.4, sz - body_h), (-hx + 0.022, 0, hz - (sz - body_h) / 2), mat(c, **plastic)),
        Part("handle_bar", gbox(0.08, sy * 0.4, 0.02), (-hx + 0.06, 0, hz - 0.01), mat(c, **plastic)),
        Part("spout", gcyl(0.022, sz * 0.12), (hx - 0.045, 0, z + body_h + sz * 0.06), mat(c, **plastic)),
        Part("cap", gcyl(0.025, sz * 0.06), (hx - 0.045, 0, hz - sz * 0.03 - 0.004), mat((0.85, 0.65, 0.10), spec=0.4)),
        Part("level_strip", gbox(0.012, 0.0016, body_h * 0.8), (-hx + 0.015, hy + 0.0008, z + body_h * 0.5), mat(lt(c, 0.5), spec=0.8, rough=0.1), alpha=0.3),
        panel_y("label_f", lab_, sx - 0.03, body_h * 0.62, 0.008, z + body_h * 0.5, hy, 1),
        panel_y("label_b", lab_, sx - 0.03, body_h * 0.62, 0.008, z + body_h * 0.5, hy, -1),
    ]


@recipe("cooler_box")
def _cooler_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lid_h = 0.055
    plastic = dict(spec=0.4, rough=0.35)
    white = mat((0.95, 0.95, 0.95), **plastic)
    lab_ = label("cooler_logo", 0.18, 0.05, [("bg", WHITE), ("text", "ARCTIC CHILL", 0.5, 0.5, 0.9, 0.6, dk(c, 0.6))])
    return [
        Part("body", gbox(sx - 0.01, sy - 0.01, sz - lid_h), (0, 0, -hz + (sz - lid_h) / 2), mat(dk(c, 0.95), **plastic)),
        Part("base_band", gbox(sx - 0.004, sy - 0.004, 0.03), (0, 0, -hz + 0.015), white),
        Part("lid", gbox(sx, sy, lid_h), (0, 0, hz - lid_h / 2), mat(c, **plastic)),
        Part("lid_rim", gbox(sx + 0.002, sy + 0.002, 0.01), (0, 0, hz - lid_h + 0.005), white),
        Part("lid_cup_a", gcyl(0.035, 0.002), (sx * 0.32, sy * 0.25, hz + 0.0005), mat(dk(c, 0.8), **plastic)),
        Part("lid_cup_b", gcyl(0.035, 0.002), (sx * 0.32, -sy * 0.25, hz + 0.0005), mat(dk(c, 0.8), **plastic)),
        Part("handle_a", gbox(0.012, 0.12, 0.035), (hx - 0.004, 0, hz - lid_h - 0.03), white),
        Part("handle_b", gbox(0.012, 0.12, 0.035), (-hx + 0.004, 0, hz - lid_h - 0.03), white),
        Part("latch", gbox(0.05, 0.008, 0.035), (0, hy - 0.002, hz - lid_h + 0.004), white),
        Part("hinge", gbox(sx - 0.08, 0.01, 0.014), (0, -hy + 0.002, hz - lid_h), white),
        Part("drain", gcyl(0.012, 0.006), (hx - 0.004, sy * 0.3, -hz + 0.05, 0, math.pi / 2, 0), white),
        panel_y("logo", lab_, 0.18, 0.05, 0, -hz + (sz - lid_h) * 0.55, hy - 0.005, 1),
    ]


@recipe("spray_can")
def _spray_can(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    cap_h = sz * 0.22
    can_h = sz - cap_h - 0.01
    lab_ = label("spray_label", 2 * math.pi * r, can_h * 0.75, [
        ("bg", (0.12, 0.12, 0.14)), ("rect", 0, 0.30, 1, 0.55, c),
        ("text", "COLOR MAX", 0.25, 0.15, 0.36, 0.12, WHITE), ("text", "SPRAY PAINT", 0.25, 0.43, 0.38, 0.14, (0.1, 0.1, 0.1)),
        ("text", "400 ml", 0.25, 0.72, 0.18, 0.10, WHITE), ("text", "COLOR MAX", 0.75, 0.15, 0.36, 0.12, WHITE),
        ("text", "LIME GREEN", 0.75, 0.43, 0.36, 0.14, (0.1, 0.1, 0.1))])
    metal = mat(SILVER, spec=0.7, metal=0.5, rough=0.3)
    z = -hz
    return [
        Part("can", gcyl(r - 0.002, can_h), (0, 0, z + can_h / 2), metal),
        band("label", lab_, r - 0.002, can_h * 0.75, z + can_h * 0.45),
        Part("shoulder", gcyl(r - 0.006, 0.01), (0, 0, z + can_h + 0.005), metal),
        Part("cap", gcyl(r - 0.001, cap_h), (0, 0, hz - cap_h / 2), mat(c, spec=0.5, rough=0.3)),
        Part("cap_top", gcyl(r - 0.006, 0.0015), (0, 0, hz + 0.0002), mat(lt(c, 0.15), spec=0.5, rough=0.3)),
    ]


@recipe("coffee_bean_sack")
def _coffee_bean_sack(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    burlap = dict(spec=0.02, rough=1.0)
    stamp = label("sack_stamp", sx * 0.7, sz * 0.7, [
        ("bg", c), ("ring", 0.5, 0.48, 0.38, (0.25, 0.12, 0.06), 0.05),
        ("text", "ARABICA", 0.5, 0.40, 0.55, 0.20, (0.25, 0.12, 0.06)), ("text", "60 KG", 0.5, 0.65, 0.35, 0.15, (0.25, 0.12, 0.06))])
    stitch = mat(dk(c, 0.6), **burlap)
    er = sz / 2 - 0.005                  # rounded ends of the lying sack
    return [
        Part("body", gbox(sx - 2 * er, sy, sz - 0.01), (0, 0, -0.005), mat(c, **burlap)),
        Part("end_a", gcyl(er, sy - 0.01), (hx - er, 0, -0.005, math.pi / 2, 0, 0), mat(c, **burlap)),
        Part("end_b", gcyl(er, sy - 0.01), (-hx + er, 0, -0.005, math.pi / 2, 0, 0), mat(c, **burlap)),
        Part("ear_a", gbox(0.02, sy * 0.8, 0.03), (hx - 0.01, 0, 0.01), mat(c, **burlap)),
        Part("ear_b", gbox(0.02, sy * 0.8, 0.03), (-hx + 0.01, 0, 0.01), mat(c, **burlap)),
        Part("seam", gbox(0.006, sy * 0.8 + 0.002, 0.004), (hx - 0.012, 0, 0.026), stitch),
        Part("stripe_a", gbox(sx - 0.04, 0.012, 0.001), (0, sy * 0.38, hz + 0.0003), mat((0.55, 0.12, 0.10), **burlap)),
        Part("stripe_b", gbox(sx - 0.04, 0.012, 0.001), (0, -sy * 0.38, hz + 0.0003), mat((0.55, 0.12, 0.10), **burlap)),
        panel_y("stamp", stamp, sx * 0.6, sz * 0.7, 0, -0.005, hy, 1),
    ]


@recipe("shampoo_bottle")
def _shampoo_bottle(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    cap_h = sz * 0.16                   # flip-top cap at the BOTTOM (bottles stand on their cap)
    plastic = dict(spec=0.6, rough=0.25)
    lab_ = label("shampoo_label", sx - 0.02, sz * 0.5, [
        ("bg", c), ("text", "SILK", 0.5, 0.20, 0.6, 0.16, WHITE), ("text", "& SHINE", 0.5, 0.38, 0.7, 0.12, WHITE),
        ("circle", 0.5, 0.62, 0.10, lt(c, 0.6)), ("text", "SHAMPOO 400 ml", 0.5, 0.86, 0.9, 0.08, WHITE)])
    r = sy / 2
    body_h = sz - cap_h
    return [
        Part("body_mid", gbox(sx - 2 * r, sy, body_h), (0, 0, hz - body_h / 2), mat(c, **plastic)),
        Part("body_a", gcyl(r, body_h), (hx - r, 0, hz - body_h / 2), mat(c, **plastic)),
        Part("body_b", gcyl(r, body_h), (-hx + r, 0, hz - body_h / 2), mat(c, **plastic)),
        Part("top_dome", gbox(sx - 0.02, sy - 0.01, 0.004), (0, 0, hz + 0.0012), mat(lt(c, 0.1), **plastic)),
        Part("cap", gbox(sx * 0.62, sy * 0.85, cap_h), (0, 0, -hz + cap_h / 2), mat(WHITE, spec=0.5, rough=0.3)),
        Part("cap_lip", gbox(sx * 0.2, 0.004, 0.008), (0, sy * 0.425 + 0.001, -hz + cap_h - 0.01), mat((0.85, 0.85, 0.85))),
        panel_y("label_f", lab_, sx - 2 * r + 0.02, sz * 0.5, 0, hz - body_h * 0.45, hy, 1),
    ]


@recipe("bowling_ball")
def _bowling_ball(sx, sy, sz, c):
    r = sx / 2
    hole = mat((0.02, 0.02, 0.03))
    out = [Part("ball", gsph(r), (0, 0, 0), mat(c, spec=0.9, rough=0.12)),
           Part("swirl_a", gcyl(r * 0.995, 0.012), (0, 0, 0, 0.5, 0.3, 0), mat(lt(c, 0.3), spec=0.9, rough=0.12)),
           Part("swirl_b", gcyl(r * 0.995, 0.008), (0, 0, 0, -0.7, 0.9, 0.4), mat(lt(c, 0.2), spec=0.9, rough=0.12))]
    # finger holes on the side (seen by the side camera, not from above)
    for i, (a, el, hr) in enumerate(((0.0, 0.25, 0.013), (0.18, 0.55, 0.011), (-0.18, 0.55, 0.011))):
        d = (math.cos(el) * math.cos(a), math.cos(el) * math.sin(a), math.sin(el) - 0.6)
        n = math.sqrt(sum(v * v for v in d))
        x, y, z = (v / n for v in d)
        pitch = math.acos(max(-1.0, min(1.0, z)))
        out.append(Part(f"hole{i}", gcyl(hr, 0.006), (x * (r - 0.002), y * (r - 0.002), z * (r - 0.002), 0, pitch, math.atan2(y, x)), hole))
    return out


# ============================================================================ more products (grocery, home, DIY)
@recipe("juice_carton")
def _juice_carton(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    top_h = 0.018
    front = label("juice_front", sx, sz - top_h, [
        ("bg", (0.98, 0.95, 0.85)), ("rect", 0, 0, 1, 0.18, c), ("text", "SUNNY", 0.5, 0.09, 0.7, 0.10, WHITE),
        ("circle", 0.5, 0.45, 0.17, (1.0, 0.62, 0.10)), ("circle", 0.5, 0.45, 0.13, (1.0, 0.80, 0.35)),
        ("text", "ORANGE", 0.5, 0.70, 0.8, 0.08, dk(c, 0.8)), ("text", "100% JUICE", 0.5, 0.80, 0.8, 0.06, (0.3, 0.3, 0.3)),
        ("rect", 0, 0.90, 1, 1, (0.20, 0.55, 0.20)), ("text", "1 L", 0.5, 0.95, 0.3, 0.07, WHITE)])
    side = label("juice_side", sy, sz - top_h, [("bg", (0.98, 0.95, 0.85)), ("text", "NUTRITION", 0.5, 0.10, 0.85, 0.06, dk(c, 0.8)),
                                               ("lines", 0.1, 0.18, 0.9, 0.6, 7, (0.3, 0.3, 0.3)), ("barcode", 0.1, 0.68, 0.9, 0.85)])
    return [
        boxy("body", sx, sy, sz - top_h, -hz + (sz - top_h) / 2, c, rough=0.6),
        Part("top", gbox(sx, sy, 0.004), (0, 0, hz - top_h + 0.002), mat(c, rough=0.6)),
        Part("seal", gbox(sx, 0.006, 0.006), (0, -hy + 0.008, hz - top_h + 0.007), mat(dk(c, 0.85))),
        Part("spout", gcyl(0.017, 0.006), (0, 0.006, hz - top_h + 0.007), mat(WHITE, spec=0.3)),
        Part("cap", gcyl(0.015, 0.010), (0, 0.006, hz - top_h + 0.015), mat(c, spec=0.4, rough=0.3)),
        panel_y("front", front, sx - 0.004, sz - top_h - 0.004, 0, -hz + (sz - top_h) / 2, hy, 1),
        panel_y("back", front, sx - 0.004, sz - top_h - 0.004, 0, -hz + (sz - top_h) / 2, hy, -1),
        panel_x("side_a", side, sy - 0.004, sz - top_h - 0.004, 0, -hz + (sz - top_h) / 2, hx, 1),
        panel_x("side_b", side, sy - 0.004, sz - top_h - 0.004, 0, -hz + (sz - top_h) / 2, hx, -1),
    ]


@recipe("tea_box")
def _tea_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = label("tea_front", sx, sz, [
        ("bg", c), ("rect", 0, 0.82, 1, 1, (0.80, 0.65, 0.25)), ("text", "EARL GREY", 0.32, 0.25, 0.55, 0.20, (0.95, 0.85, 0.55)),
        ("text", "black tea", 0.32, 0.48, 0.40, 0.13, WHITE), ("circle", 0.78, 0.45, 0.22, WHITE), ("circle", 0.78, 0.40, 0.15, (0.55, 0.30, 0.12)),
        ("text", "20 TEA BAGS", 0.5, 0.91, 0.6, 0.12, c)])
    side = label("tea_side", sy, sz, [("bg", c), ("text", "40 g", 0.5, 0.35, 0.6, 0.2, WHITE), ("barcode", 0.1, 0.55, 0.9, 0.85)])
    return [
        boxy("body", sx, sy, sz, 0, c, rough=0.5, spec=0.3),
        Part("tear_strip", gbox(0.003, sy, 0.001), (hx - 0.03, 0, hz + 0.0004), mat((0.80, 0.65, 0.25))),
        Part("foil_seal", gbox(sx, sy, 0.004), (0, 0, hz - 0.002), mat(dk(c, 0.95), spec=0.5, rough=0.3)),
        panel_y("front", front, sx - 0.004, sz - 0.004, 0, 0, hy, 1), panel_y("back", front, sx - 0.004, sz - 0.004, 0, 0, hy, -1),
        panel_x("side_a", side, sy - 0.004, sz - 0.004, 0, 0, hx, 1), panel_x("side_b", side, sy - 0.004, sz - 0.004, 0, 0, hx, -1),
    ]


@recipe("printer_paper")
def _printer_paper(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    side = label("paper_side", sx - 0.01, sz - 0.006, [
        ("bg", c), ("rect", 0, 0.15, 1, 0.85, WHITE), ("text", "COPY PAPER", 0.30, 0.50, 0.42, 0.45, c),
        ("text", "A4  80 g/m2  500 sheets", 0.75, 0.50, 0.44, 0.30, (0.3, 0.3, 0.35))])
    end = label("paper_end", sy - 0.01, sz - 0.006, [("bg", c), ("text", "A4", 0.5, 0.5, 0.3, 0.6, WHITE)])
    return [
        boxy("ream", sx, sy, sz, 0, c, spec=0.35, rough=0.5),
        Part("fold_a", gbox(0.03, sy, 0.001), (hx - 0.015, 0, -hz - 0.0004), mat(dk(c, 0.85))),
        Part("seam", gbox(sx, 0.003, 0.001), (0, sy * 0.15, hz + 0.0004), mat(dk(c, 0.8))),
        panel_y("side_l", side, sx - 0.01, sz - 0.006, 0, 0, hy, 1), panel_y("side_r", side, sx - 0.01, sz - 0.006, 0, 0, hy, -1),
        panel_x("end_a", end, sy - 0.01, sz - 0.006, 0, 0, hx, 1), panel_x("end_b", end, sy - 0.01, sz - 0.006, 0, 0, hx, -1),
    ]


@recipe("phone_box")
def _phone_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lid_h = sz * 0.6
    side = label("phone_side", sx - 0.01, lid_h - 0.006, [("bg", c), ("text", "NOVA X12", 0.30, 0.5, 0.45, 0.55, WHITE),
                                                         ("text", "256 GB", 0.78, 0.5, 0.30, 0.45, lt(c, 0.6))])
    line = mat(lt(c, 0.3), spec=0.6, rough=0.2)
    w, h, t = sx * 0.55, sy * 0.62, 0.003
    return [
        Part("base", gbox(sx - 0.004, sy - 0.004, sz - 0.004), (0, 0, -0.002), mat((0.92, 0.92, 0.93), rough=0.5)),
        Part("lid", gbox(sx, sy, lid_h), (0, 0, hz - lid_h / 2), mat(c, spec=0.5, rough=0.3)),
        Part("emb_a", gbox(w, t, 0.001), (0, h / 2, hz + 0.0004), line), Part("emb_b", gbox(w, t, 0.001), (0, -h / 2, hz + 0.0004), line),
        Part("emb_c", gbox(t, h, 0.001), (w / 2, 0, hz + 0.0004), line), Part("emb_d", gbox(t, h, 0.001), (-w / 2, 0, hz + 0.0004), line),
        panel_y("side_l", side, sx - 0.01, lid_h - 0.006, 0, hz - lid_h / 2, hy, 1),
        panel_y("side_r", side, sx - 0.01, lid_h - 0.006, 0, hz - lid_h / 2, hy, -1),
        Part("seal_a", gbox(0.0015, 0.015, sz * 0.8), (hx + 0.0008, 0, 0), mat((0.95, 0.95, 0.97), spec=0.6), alpha=0.3),
    ]


@recipe("diaper_pack")
def _diaper_pack(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = label("diaper_front", sx, sz - 0.04, [
        ("bg", c), ("rect", 0, 0.78, 1, 1, WHITE), ("text", "SOFT STEPS", 0.5, 0.13, 0.8, 0.12, WHITE),
        ("circle", 0.5, 0.48, 0.20, (0.98, 0.85, 0.70)), ("circle", 0.43, 0.45, 0.025, (0.2, 0.2, 0.2)),
        ("circle", 0.57, 0.45, 0.025, (0.2, 0.2, 0.2)), ("ring", 0.5, 0.52, 0.06, (0.85, 0.35, 0.35), 0.02),
        ("text", "SIZE 4   52 diapers", 0.5, 0.89, 0.8, 0.10, dk(c, 0.6))])
    return [
        Part("bag", gbox(sx, sy - 0.01, sz - 0.04), (0, 0, -hz + (sz - 0.04) / 2), mat(c, spec=0.5, rough=0.3)),
        Part("bag_top", gbox(sx - 0.02, sy - 0.03, 0.02), (0, 0, hz - 0.03), mat(c, spec=0.5, rough=0.3)),
        Part("seal", gbox(sx - 0.02, 0.008, 0.02), (0, 0, hz - 0.01), mat(dk(c, 0.85), spec=0.5)),
        Part("handle_a", gbox(0.012, 0.006, 0.02), (-0.06, 0, hz - 0.01), mat(WHITE)),
        Part("handle_b", gbox(0.012, 0.006, 0.02), (0.06, 0, hz - 0.01), mat(WHITE)),
        Part("handle", gbox(0.13, 0.006, 0.008), (0, 0, hz - 0.004), mat(WHITE)),
        panel_y("front", front, sx - 0.02, sz - 0.06, 0, -hz + (sz - 0.04) / 2, hy - 0.005, 1),
        panel_y("back", front, sx - 0.02, sz - 0.06, 0, -hz + (sz - 0.04) / 2, hy - 0.005, -1),
    ]


@recipe("beer_crate")
def _beer_crate(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    plastic = dict(spec=0.35, rough=0.45)
    t = 0.012
    logo = label("crate_logo", 0.16, 0.06, [("bg", c), ("rect", 0.05, 0.1, 0.95, 0.9, WHITE),
                                          ("text", "BERGBRAU", 0.5, 0.5, 0.8, 0.5, (0.45, 0.25, 0.02))])
    out = [
        Part("floor", gbox(sx, sy, 0.02), (0, 0, -hz + 0.01), mat(dk(c, 0.8), **plastic)),
        Part("wall_a", gbox(sx, t, sz), (0, hy - t / 2, 0), mat(c, **plastic)),
        Part("wall_b", gbox(sx, t, sz), (0, -hy + t / 2, 0), mat(c, **plastic)),
        Part("wall_c", gbox(t, sy - 2 * t, sz), (hx - t / 2, 0, 0), mat(c, **plastic)),
        Part("wall_d", gbox(t, sy - 2 * t, sz), (-hx + t / 2, 0, 0), mat(c, **plastic)),
        Part("handle_a", gbox(0.002, 0.10, 0.03), (hx + 0.0005, 0, hz - 0.035), mat((0.10, 0.06, 0.01))),
        Part("handle_b", gbox(0.002, 0.10, 0.03), (-hx - 0.0005, 0, hz - 0.035), mat((0.10, 0.06, 0.01))),
        Part("shoulders", gbox(sx - 2 * t, sy - 2 * t, 0.01), (0, 0, hz - 0.055), mat((0.30, 0.17, 0.04), spec=0.8, rough=0.1)),
        panel_y("logo", logo, 0.16, 0.06, 0, -0.02, hy, 1), panel_y("logo_b", logo, 0.16, 0.06, 0, -0.02, hy, -1),
    ]
    nx, ny = 5, 4
    for i in range(nx):
        for j in range(ny):
            x = -hx + t + (sx - 2 * t) * (i + 0.5) / nx
            y = -hy + t + (sy - 2 * t) * (j + 0.5) / ny
            out.append(Part(f"neck{i}{j}", gcyl(0.013, 0.045), (x, y, hz - 0.03), mat((0.32, 0.18, 0.04), spec=0.8, rough=0.1)))
            out.append(Part(f"cap{i}{j}", gcyl(0.0145, 0.006), (x, y, hz - 0.004), mat((0.62, 0.48, 0.10), spec=0.6, metal=0.4, rough=0.3)))
    return out


@recipe("bread_loaf")
def _bread_loaf(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    crust = mat(c, spec=0.1, rough=0.9)
    body_h = sz - (sy / 2 - 0.003)          # the dome on top reaches the catalogue height
    lab_ = label("bread_label", 0.12, 0.05, [("bg", (0.85, 0.15, 0.10)), ("text", "FARM BAKERY", 0.5, 0.35, 0.85, 0.32, WHITE),
                                            ("text", "whole wheat", 0.5, 0.75, 0.6, 0.25, (1.0, 0.9, 0.7))])
    L = sx - 0.05
    return [
        Part("loaf", gbox(L, sy - 0.006, body_h), (-0.015, 0, -hz + body_h / 2), crust),
        Part("dome", gcyl(sy / 2 - 0.003, L), (-0.015, 0, -hz + body_h, 0, math.pi / 2, 0), crust),
        Part("cut_a", gbox(0.004, sy * 0.6, 0.003), (-0.08, 0, hz - 0.002, 0, 0, 0.5), mat(lt(c, 0.35))),
        Part("cut_b", gbox(0.004, sy * 0.6, 0.003), (0.0, 0, hz - 0.002, 0, 0, 0.5), mat(lt(c, 0.35))),
        Part("cut_c", gbox(0.004, sy * 0.6, 0.003), (0.08, 0, hz - 0.002, 0, 0, 0.5), mat(lt(c, 0.35))),
        Part("bag", gbox(L + 0.004, sy, sz), (-0.015, 0, 0), mat((0.95, 0.95, 0.95), spec=0.9, rough=0.1), alpha=0.8),
        Part("bag_tail", gbox(0.03, sy * 0.35, sz * 0.35), (hx - 0.03, 0, -0.01), mat((0.95, 0.95, 0.95), spec=0.9, rough=0.1), alpha=0.5),
        Part("twist_tie", gbox(0.008, sy * 0.4, sz * 0.4), (hx - 0.012, 0, -0.01), mat((0.85, 0.15, 0.10))),
        panel_y("label", lab_, 0.12, 0.05, -0.03, -0.01, hy, 1),
    ]


@recipe("watermelon")
def _watermelon(sx, sy, sz, c):
    r = sx / 2
    stripe = mat((0.30, 0.40, 0.22), spec=0.3, rough=0.5)
    out = [Part("melon", gsph(r), (0, 0, 0), mat(c, spec=0.35, rough=0.5)),
           Part("stem", gcyl(0.006, 0.01), (0, 0, r - 0.004), mat((0.40, 0.30, 0.12)))]
    for k in range(6):
        out.append(Part(f"stripe{k}", gcyl(r + 0.0008, 0.016), (0, 0, 0, math.pi / 2, 0, k * math.pi / 6), stripe))
    return out


@recipe("chips_tube")
def _chips_tube(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    lid_h = 0.016
    lab_ = label("chips_label", 2 * math.pi * r, sz - lid_h - 0.01, [
        ("bg", c), ("circle", 0.25, 0.40, 0.15, (0.98, 0.80, 0.35)), ("text", "CRISPY", 0.25, 0.62, 0.35, 0.08, WHITE),
        ("text", "SOUR CREAM", 0.25, 0.75, 0.40, 0.06, (0.95, 0.85, 0.40)), ("text", "& ONION", 0.25, 0.83, 0.30, 0.05, (0.95, 0.85, 0.40)),
        ("text", "CRISPY", 0.75, 0.62, 0.35, 0.08, WHITE), ("barcode", 0.62, 0.20, 0.88, 0.35)])
    return [
        Part("tube", gcyl(r - 0.0015, sz - lid_h), (0, 0, -hz + (sz - lid_h) / 2), mat(c, rough=0.6)),
        band("label", lab_, r - 0.0015, sz - lid_h - 0.01, -hz + (sz - lid_h) / 2),
        Part("rim", gcyl(r - 0.0005, 0.004), (0, 0, hz - lid_h + 0.001), mat(SILVER, metal=0.5, rough=0.3)),
        Part("lid", gcyl(r, lid_h - 0.002), (0, 0, hz - (lid_h - 0.002) / 2), mat(c, spec=0.5, rough=0.3), alpha=0.1),
        Part("bottom", gcyl(r - 0.001, 0.004), (0, 0, -hz + 0.002), mat(SILVER, metal=0.5, rough=0.3)),
    ]


@recipe("paint_bucket")
def _paint_bucket(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    plastic = dict(spec=0.35, rough=0.4)
    lab_ = label("bucket_label", 2 * math.pi * r, 0.18, [
        ("bg", WHITE), ("rect", 0, 0, 1, 0.18, c), ("text", "FACADE", 0.25, 0.38, 0.30, 0.18, c),
        ("text", "EXTERIOR PAINT", 0.25, 0.60, 0.36, 0.12, (0.25, 0.25, 0.25)), ("text", "15 L", 0.25, 0.82, 0.12, 0.14, (0.25, 0.25, 0.25)),
        ("rect", 0.55, 0.30, 0.95, 0.85, (0.85, 0.82, 0.70)), ("text", "SAND BEIGE", 0.75, 0.57, 0.32, 0.12, (0.3, 0.25, 0.2))])
    out = [
        Part("bucket", gcyl(r * 0.94, sz - 0.03), (0, 0, -0.015), mat(c, **plastic)),
        band("label", lab_, r * 0.94, 0.18, -0.03),
        Part("rim", gcyl(r, 0.03), (0, 0, hz - 0.024), mat(c, **plastic)),
        Part("lid", gcyl(r * 0.97, 0.008), (0, 0, hz - 0.004), mat(dk(c, 0.95), **plastic)),
        Part("lid_ring", gcyl(r * 0.80, 0.002), (0, 0, hz + 0.0005), mat(dk(c, 0.85), **plastic)),
        Part("handle", gcyl(0.003, 2 * r * 0.92), (0.03, 0, hz + 0.002, math.pi / 2, 0, 0), mat((0.55, 0.55, 0.58), metal=0.6, rough=0.3)),
        Part("grip", gcyl(0.009, 0.10), (0.03, 0, hz + 0.004, math.pi / 2, 0, 0), mat(dk(c, 0.6), **plastic)),
        Part("ear_a", gbox(0.03, 0.012, 0.03), (0, r - 0.004, hz - 0.045), mat(c, **plastic)),
        Part("ear_b", gbox(0.03, 0.012, 0.03), (0, -r + 0.004, hz - 0.045), mat(c, **plastic)),
    ]
    return out


@recipe("honey_jar")
def _honey_jar(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    lid_h = 0.02
    lab_ = label("honey_label", 2 * math.pi * r, 0.05, [
        ("bg", (0.98, 0.93, 0.70)), ("text", "WILDFLOWER", 0.25, 0.35, 0.32, 0.28, (0.45, 0.25, 0.02)),
        ("text", "HONEY 500 g", 0.25, 0.72, 0.26, 0.24, (0.3, 0.2, 0.05)), ("circle", 0.75, 0.5, 0.30, (0.95, 0.70, 0.10)),
        ("text", "BEE", 0.75, 0.5, 0.10, 0.20, (0.2, 0.15, 0.05))])
    return [
        Part("glass", gcyl(r, sz - lid_h), (0, 0, -hz + (sz - lid_h) / 2), mat((0.95, 0.90, 0.75), spec=0.9, rough=0.05), alpha=0.6),
        Part("honey", gcyl(r - 0.004, sz - lid_h - 0.012), (0, 0, -hz + (sz - lid_h) / 2 - 0.002), mat((0.72, 0.42, 0.04), spec=0.7, rough=0.15)),
        Part("lid", gcyl(r - 0.001, lid_h), (0, 0, hz - lid_h / 2), mat(c, spec=0.5, metal=0.3, rough=0.35)),
        Part("lid_top", gcyl(r - 0.008, 0.0015), (0, 0, hz + 0.0002), mat(lt(c, 0.15), spec=0.5, metal=0.3, rough=0.35)),
        band("label", lab_, r, 0.05, -hz + (sz - lid_h) * 0.45),
    ]


@recipe("ketchup_bottle")
def _ketchup_bottle(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    cap_h = sz * 0.17                    # squeeze bottle standing on its cap
    plastic = dict(spec=0.6, rough=0.25)
    r = sy / 2
    lab_ = label("ketchup_label", sx - 0.02, sz * 0.4, [
        ("bg", WHITE), ("circle", 0.5, 0.40, 0.25, c), ("circle", 0.5, 0.18, 0.06, (0.25, 0.55, 0.15)),
        ("text", "TOMATO", 0.5, 0.72, 0.8, 0.13, c), ("text", "KETCHUP", 0.5, 0.87, 0.85, 0.12, (0.2, 0.2, 0.2))])
    body_h = sz - cap_h
    return [
        Part("body_mid", gbox(sx - 2 * r, sy, body_h), (0, 0, hz - body_h / 2), mat(c, **plastic)),
        Part("body_a", gcyl(r, body_h), (hx - r, 0, hz - body_h / 2), mat(c, **plastic)),
        Part("body_b", gcyl(r, body_h), (-hx + r, 0, hz - body_h / 2), mat(c, **plastic)),
        Part("cap", gcyl(sy * 0.45, cap_h), (0, 0, -hz + cap_h / 2), mat(WHITE, spec=0.5, rough=0.3)),
        Part("cap_flip", gbox(0.02, 0.004, 0.012), (0, sy * 0.45, -hz + cap_h - 0.008), mat((0.85, 0.85, 0.85))),
        panel_y("label_f", lab_, sx - 2 * r + 0.02, sz * 0.4, 0, hz - body_h * 0.5, hy, 1),
    ]


@recipe("car_tire")
def _car_tire(sx, sy, sz, c):
    """A wheel lying flat: black tyre around an alloy rim (the rim is the catalogue colour)."""
    r, hz = sx / 2, sz / 2
    tyre = (0.07, 0.07, 0.08)
    rubber = dict(spec=0.15, rough=0.9)
    alloy = mat(c, spec=0.7, metal=0.5, rough=0.25)
    tread = label("tire_tread", 2 * math.pi * r, sz, [("bg", tyre)] + [
        ("rect", k / 40.0, 0.12, k / 40.0 + 0.01, 0.88, (0.15, 0.15, 0.16)) for k in range(40)] + [
        ("rect", 0, 0.47, 1, 0.53, (0.15, 0.15, 0.16))])
    out = [
        Part("tread", gcyl(r - 0.0015, sz - 0.02), (0, 0, 0), mat(tyre, **rubber)),
        band("tread_print", tread, r - 0.0015, sz - 0.03, 0),
        Part("wall_top", gcyl(r - 0.01, 0.01), (0, 0, hz - 0.005), mat(tyre, **rubber)),
        Part("wall_bottom", gcyl(r - 0.01, 0.01), (0, 0, -hz + 0.005), mat(tyre, **rubber)),
        Part("letters", gcyl(r * 0.86, 0.0015), (0, 0, hz + 0.0003), mat((0.30, 0.30, 0.32), **rubber)),
        Part("wall_inner", gcyl(r * 0.82, 0.0015), (0, 0, hz + 0.0008), mat(tyre, **rubber)),
        Part("rim", gcyl(r * 0.66, 0.004), (0, 0, hz - 0.006), alloy),
        Part("rim_lip", gcyl(r * 0.68, 0.0015), (0, 0, hz + 0.0012), alloy),
        Part("rim_well", gcyl(r * 0.60, 0.0015), (0, 0, hz + 0.0016), mat(dk(c, 0.75), spec=0.5, metal=0.5, rough=0.3)),
        Part("hub", gcyl(r * 0.16, 0.004), (0, 0, hz + 0.0018), alloy),
        Part("cap", gcyl(r * 0.07, 0.0015), (0, 0, hz + 0.0038), mat((0.10, 0.10, 0.12), spec=0.6)),
    ]
    for k in range(5):                       # spokes
        a = k * 2 * math.pi / 5
        out.append(Part(f"spoke{k}", gbox(r * 0.46, 0.028, 0.0015), (r * 0.38 * math.cos(a), r * 0.38 * math.sin(a), hz + 0.0021, 0, 0, a), alloy))
        b = a + math.pi / 5
        out.append(Part(f"nut{k}", gcyl(0.006, 0.0015), (r * 0.10 * math.cos(b), r * 0.10 * math.sin(b), hz + 0.0036), mat(dk(c, 0.6), metal=0.5)))
    return out


@recipe("traffic_cone")
def _traffic_cone(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    pvc = dict(spec=0.35, rough=0.4)
    base_h = 0.03
    out = [Part("base_disc", gcyl(r, base_h * 0.6), (0, 0, -hz + base_h * 0.3), mat(c, **pvc)),
           Part("base", gbox(r * 1.41, r * 1.41, base_h), (0, 0, -hz + base_h / 2), mat(c, **pvc))]
    n = 8
    h = (sz - base_h) / n
    for k in range(n):
        rk = r * (0.62 - 0.52 * k / n)
        out.append(Part(f"cone{k}", gcyl(rk, h), (0, 0, -hz + base_h + h * (k + 0.5)), mat(c, **pvc)))
    for k, (z, rk) in enumerate(((0.38, 0.45), (0.62, 0.32))):
        out.append(Part(f"reflect{k}", gcyl(r * rk + 0.003, h * 0.9), (0, 0, -hz + base_h + (sz - base_h) * z),
                        mat((0.95, 0.95, 0.95), spec=0.8, rough=0.2)))
    return out


@recipe("fire_extinguisher")
def _fire_extinguisher(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    paint = dict(spec=0.6, rough=0.25)
    black = mat((0.06, 0.06, 0.06), spec=0.3, rough=0.5)
    body_h = sz - r - 0.05
    lab_ = label("extinguisher_label", 2 * math.pi * r, 0.14, [
        ("bg", c), ("rect", 0.02, 0.05, 0.48, 0.95, WHITE), ("text", "FIRE", 0.25, 0.22, 0.25, 0.16, c),
        ("text", "EXTINGUISHER", 0.25, 0.42, 0.42, 0.12, c), ("text", "ABC  6 kg", 0.25, 0.62, 0.30, 0.12, (0.15, 0.15, 0.15)),
        ("lines", 0.06, 0.72, 0.44, 0.92, 3, (0.2, 0.2, 0.2))])
    z0 = -hz
    return [
        Part("body", gcyl(r, body_h), (0, 0, z0 + body_h / 2), mat(c, **paint)),
        Part("foot", gcyl(r + 0.0005, 0.02), (0, 0, z0 + 0.01), black),
        band("label", lab_, r, 0.14, z0 + body_h * 0.5),
        Part("dome", gsph(r), (0, 0, z0 + body_h), mat(c, **paint)),
        Part("neck", gcyl(0.018, 0.03), (0, 0, z0 + body_h + r + 0.005), mat((0.70, 0.70, 0.72), metal=0.6, rough=0.3)),
        Part("valve", gbox(0.03, 0.03, 0.02), (0, 0, hz - 0.025), mat((0.70, 0.70, 0.72), metal=0.6, rough=0.3)),
        Part("lever", gbox(0.07, 0.016, 0.006), (-0.025, 0, hz - 0.004, 0, -0.15, 0), mat((0.72, 0.72, 0.74), metal=0.6, rough=0.3)),
        Part("gauge", gcyl(0.011, 0.006), (0, -0.016, hz - 0.025, math.pi / 2, 0, 0), mat(WHITE)),
        Part("hose", gcyl(0.006, 0.20), (0.02, r + 0.003, z0 + body_h - 0.05, 0, 0.05, 0), black),
        Part("nozzle", gcyl(0.009, 0.03), (0.02, r + 0.003, z0 + body_h - 0.165), black),
    ]


@recipe("cabbage")
def _cabbage(sx, sy, sz, c):
    r = sx / 2
    leaf = mat(lt(c, 0.2), spec=0.3, rough=0.6)
    vein = mat(lt(c, 0.45), rough=0.7)
    out = [Part("head", gsph(r * 0.97), (0, 0, 0), mat(c, spec=0.3, rough=0.6))]
    for k in range(5):
        a = k * 2 * math.pi / 5
        out.append(Part(f"leaf{k}", gcyl(r * 0.78, 0.004), (0.45 * r * math.cos(a), 0.45 * r * math.sin(a), -0.15 * r,
                                                            0, 1.1, a), leaf))
    for k in range(3):
        out.append(Part(f"vein{k}", gcyl(r * 0.975, 0.004), (0, 0, 0, math.pi / 2, 0, k * math.pi / 3 + 0.3), vein))
    return out


@recipe("microwave_box")
def _microwave_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = label("microwave_front", sx - 0.02, sz - 0.03, [
        ("bg", c), ("rect", 0.08, 0.15, 0.70, 0.72, (0.10, 0.10, 0.11)), ("rect", 0.12, 0.22, 0.52, 0.65, (0.30, 0.32, 0.34)),
        ("rect", 0.56, 0.22, 0.66, 0.65, (0.18, 0.18, 0.20)), ("circle", 0.61, 0.32, 0.03, (0.6, 0.6, 0.6)),
        ("circle", 0.61, 0.50, 0.03, (0.6, 0.6, 0.6)), ("text", "MICROWAVE OVEN", 0.40, 0.84, 0.62, 0.09, (0.1, 0.08, 0.06)),
        ("text", "25 L  900 W", 0.86, 0.40, 0.22, 0.08, (0.1, 0.08, 0.06)), ("rect", 0.75, 0.55, 0.97, 0.75, WHITE),
        ("barcode", 0.76, 0.57, 0.96, 0.73)])
    side = label("microwave_side", sy - 0.02, sz - 0.03, [("bg", c), ("text", "HOMECOOK", 0.5, 0.3, 0.8, 0.14, (0.65, 0.10, 0.08)),
                                                           ("arrow", 0.5, 0.65, 0.18, (0.1, 0.08, 0.06))])
    tape = mat((0.58, 0.42, 0.24), spec=0.45, rough=0.3)
    return [
        boxy("body", sx, sy, sz, 0, c, rough=0.95),
        Part("tape_top", gbox(sx + 0.003, 0.06, 0.0012), (0, 0, hz + 0.0012), tape),
        Part("tape_a", gbox(0.0012, 0.06, 0.08), (hx + 0.0009, 0, hz - 0.04), tape),
        Part("tape_b", gbox(0.0012, 0.06, 0.08), (-hx - 0.0009, 0, hz - 0.04), tape),
        Part("grip_a", gbox(0.002, 0.10, 0.03), (hx + 0.0008, 0, hz - 0.08), mat((0.10, 0.07, 0.04))),
        Part("grip_b", gbox(0.002, 0.10, 0.03), (-hx - 0.0008, 0, hz - 0.08), mat((0.10, 0.07, 0.04))),
        panel_y("front", front, sx - 0.02, sz - 0.03, 0, -0.005, hy, 1), panel_y("back", front, sx - 0.02, sz - 0.03, 0, -0.005, hy, -1),
        panel_x("side_a", side, sy - 0.14, sz - 0.14, 0, -0.06, hx, 1), panel_x("side_b", side, sy - 0.14, sz - 0.14, 0, -0.06, hx, -1),
    ]


@recipe("first_aid_kit")
def _first_aid_kit(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    plastic = dict(spec=0.45, rough=0.35)
    white = mat((0.96, 0.96, 0.96), spec=0.3)
    out = [
        Part("base", gbox(sx, sy - 0.02, sz * 0.55), (0, -0.01, -hz + sz * 0.275), mat(dk(c, 0.92), **plastic)),
        Part("lid", gbox(sx - 0.004, sy - 0.024, sz * 0.45), (0, -0.01, hz - sz * 0.225), mat(c, **plastic)),
        Part("cross_a", gbox(0.07, 0.022, 0.002), (0, -0.01, hz + 0.0008), white),
        Part("cross_b", gbox(0.022, 0.07, 0.002), (0, -0.01, hz + 0.0008), white),
        Part("front_cross_a", gbox(0.05, 0.002, 0.016), (0.07, hy - 0.019, 0), white),
        Part("front_cross_b", gbox(0.016, 0.002, 0.05), (0.07, hy - 0.019, 0), white),
        Part("handle", gbox(0.10, 0.012, 0.012), (-0.03, hy - 0.006, 0), mat((0.95, 0.95, 0.95), **plastic)),
        Part("latch_a", gbox(0.025, 0.008, 0.02), (-0.10, hy - 0.016, 0), white),
        Part("latch_b", gbox(0.025, 0.008, 0.02), (0.11, hy - 0.016, 0), white),
    ]
    return out


@recipe("globe")
def _globe(sx, sy, sz, c):
    r = sx / 2
    land = mat((0.38, 0.42, 0.20), spec=0.3, rough=0.6)
    sand = mat((0.62, 0.52, 0.30), spec=0.3, rough=0.6)
    out = [Part("sea", gsph(r), (0, 0, 0), mat(c, spec=0.6, rough=0.25)),
           Part("meridian", gcyl(r + 0.0012, 0.006), (0, 0, 0, math.pi / 2, 0, 0.4), mat((0.70, 0.55, 0.20), spec=0.7, metal=0.5, rough=0.3)),
           Part("equator", gcyl(r + 0.0006, 0.003), (0, 0, 0), mat((0.85, 0.85, 0.80)))]
    patches = [((0.35, 1.2), 0.30, land), ((0.20, 2.2), 0.24, sand), ((-0.40, 2.0), 0.22, land), ((0.10, -0.6), 0.28, land),
               ((-0.30, -1.4), 0.20, sand), ((0.70, 0.3), 0.18, land), ((-0.70, -2.6), 0.16, land), ((0.45, -2.4), 0.22, sand)]
    for i, ((el, az), size, m) in enumerate(patches):
        x, y, z = math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)
        d = r - 0.003
        out.append(Part(f"land{i}", gcyl(r * size, 0.008), (x * d, y * d, z * d, 0, math.acos(z), math.atan2(y, x)), m))
    return out


@recipe("wooden_crate")
def _wooden_crate(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    wood = label("wood_grain", 0.4, 0.1, [("bg", c), ("lines", 0.0, 0.05, 1.0, 0.95, 9, dk(c, 0.8))])
    stencil = label("crate_stencil", sx - 0.08, sz * 0.5, [
        ("bg", c), ("lines", 0.0, 0.05, 1.0, 0.95, 8, dk(c, 0.82)), ("text", "HANDLE WITH CARE", 0.5, 0.35, 0.85, 0.22, (0.15, 0.10, 0.06)),
        ("arrow", 0.5, 0.75, 0.16, (0.15, 0.10, 0.06))])
    wm = mat(tex=wood, rough=0.95)
    post = mat(dk(c, 0.8), rough=0.95)
    out = [Part("inside", gbox(sx - 0.03, sy - 0.03, sz - 0.03), (0, 0, -0.005), mat((0.12, 0.09, 0.05)))]
    n = 5
    for k in range(n):                                   # top slats with small gaps
        w = sy / n - 0.008
        out.append(Part(f"top{k}", gbox(sx - 0.002, w, 0.018), (0, -hy + (k + 0.5) * sy / n, hz - 0.009), wm))
    for s in (1, -1):
        for k, z in enumerate((-hz + 0.04, 0.0, hz - 0.06)):
            out.append(Part(f"side{s > 0}{k}", gbox(sx, 0.015, 0.06), (0, s * (hy - 0.0075), z), wm))
            out.append(Part(f"end{s > 0}{k}", gbox(0.015, sy - 0.03, 0.06), (s * (hx - 0.0075), 0, z), wm))
        for t in (1, -1):
            out.append(Part(f"post{s > 0}{t > 0}", gbox(0.03, 0.03, sz - 0.02), (s * (hx - 0.015), t * (hy - 0.015), -0.01), post))
    out.append(panel_y("stencil", stencil, sx - 0.08, sz * 0.5, 0, -0.02, hy, 1))
    return out


@recipe("baby_formula_tin")
def _baby_formula_tin(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    lid_h = 0.03
    lab_ = label("formula_label", 2 * math.pi * r, sz - lid_h - 0.02, [
        ("bg", (0.98, 0.96, 0.97)), ("rect", 0, 0, 1, 0.18, c), ("text", "LITTLE ONE", 0.25, 0.33, 0.34, 0.12, dk(c, 0.7)),
        ("circle", 0.25, 0.58, 0.14, (0.98, 0.85, 0.72)), ("text", "infant milk  800 g", 0.25, 0.86, 0.36, 0.08, (0.3, 0.3, 0.35)),
        ("text", "STAGE 1", 0.75, 0.45, 0.25, 0.12, dk(c, 0.7)), ("barcode", 0.62, 0.62, 0.88, 0.85)])
    return [
        Part("tin", gcyl(r - 0.002, sz - lid_h), (0, 0, -hz + (sz - lid_h) / 2), mat(SILVER, spec=0.6, metal=0.5, rough=0.3)),
        band("label", lab_, r - 0.002, sz - lid_h - 0.02, -hz + (sz - lid_h) / 2),
        Part("lid", gcyl(r, lid_h), (0, 0, hz - lid_h / 2), mat(c, spec=0.45, rough=0.3)),
        Part("lid_tab", gbox(0.03, 0.012, 0.004), (r - 0.017, 0, hz + 0.001), mat(dk(c, 0.85), spec=0.45)),
        Part("lid_ring", gcyl(r * 0.85, 0.0015), (0, 0, hz + 0.0003), mat(lt(c, 0.15), spec=0.45, rough=0.3)),
    ]


@recipe("kettlebell")
def _kettlebell(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    iron = dict(spec=0.35, rough=0.55)
    ball_r = r
    zc = -hz + ball_r
    handle_top = hz - 0.012
    post_h = handle_top - (zc + ball_r * 0.55)
    return [
        Part("ball", gsph(ball_r), (0, 0, zc), mat(c, **iron)),
        Part("base", gcyl(ball_r * 0.6, 0.012), (0, 0, -hz + 0.006), mat(c, **iron)),
        Part("post_a", gcyl(0.017, post_h), (r * 0.55, 0, zc + ball_r * 0.55 + post_h / 2, 0, -0.25, 0), mat(c, **iron)),
        Part("post_b", gcyl(0.017, post_h), (-r * 0.55, 0, zc + ball_r * 0.55 + post_h / 2, 0, 0.25, 0), mat(c, **iron)),
        Part("handle", gcyl(0.017, r * 1.1), (0, 0, handle_top, 0, math.pi / 2, 0), mat(c, **iron)),
        Part("weight_mark", gcyl(0.03, 0.002), (0, ball_r * 0.98, zc, math.pi / 2, 0, 0), mat((0.85, 0.70, 0.15), spec=0.5)),
    ]


# ============================================================================ product families (used by the products below)
def print_ops(c, title, sub="", extra="", fg=WHITE, accent=None, picture=None, band_top=True):
    """A typical printed front: brand band, product name, a simple picture, small print."""
    accent = accent or dk(c, 0.6)
    ops = [("bg", c)]
    if band_top:
        ops += [("rect", 0, 0, 1, 0.14, accent)]
    ops += [("text", title, 0.5, 0.30, 0.85, 0.14, fg)]
    if sub:
        ops += [("text", sub, 0.5, 0.45, 0.80, 0.08, fg)]
    if picture:
        ops += picture
    ops += [("rect", 0, 0.86, 1, 1, accent)]
    if extra:
        ops += [("text", extra, 0.5, 0.93, 0.6, 0.08, fg)]
    return ops


def info_ops(c, fg=(0.2, 0.2, 0.2), bg=None):
    return [("bg", bg or lt(c, 0.75)), ("text", "INGREDIENTS", 0.5, 0.10, 0.85, 0.06, fg),
            ("lines", 0.1, 0.18, 0.9, 0.58, 7, fg), ("barcode", 0.1, 0.66, 0.9, 0.86)]


def carton(name, sx, sy, sz, c, front, side=None, seam=True, tape=False, rough=0.6):
    """Printed cardboard / paperboard carton: front and back print, side panels, flap seam."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    f = label(f"{name}_front", sx, sz, front)
    s_ = label(f"{name}_side", sy, sz, side or info_ops(c))
    out = [boxy("body", sx, sy, sz, 0, c, rough=rough, spec=0.25),
           panel_y("front", f, sx - 0.004, sz - 0.004, 0, 0, hy, 1), panel_y("back", f, sx - 0.004, sz - 0.004, 0, 0, hy, -1),
           panel_x("side_a", s_, sy - 0.004, sz - 0.004, 0, 0, hx, 1), panel_x("side_b", s_, sy - 0.004, sz - 0.004, 0, 0, hx, -1)]
    if seam:
        out.append(Part("flap", gbox(sx, 0.003, 0.001), (0, 0, hz + 0.0004), mat(dk(c, 0.6))))
    if tape:
        t = mat((0.58, 0.42, 0.24), spec=0.45, rough=0.3)
        out += [Part("tape_top", gbox(sx + 0.003, 0.05, 0.0012), (0, 0, hz + 0.0012), t),
                Part("tape_a", gbox(0.0012, 0.05, 0.07), (hx + 0.0009, 0, hz - 0.035), t),
                Part("tape_b", gbox(0.0012, 0.05, 0.07), (-hx - 0.0009, 0, hz - 0.035), t)]
    return out


def sack(name, sx, sy, sz, c, front, fold_c=None):
    """Paper / plastic bag standing up, folded and sealed at the top."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz - 0.04
    f = label(f"{name}_front", sx, body_h, front)
    return [Part("body", gbox(sx, sy - 0.008, body_h), (0, 0, -hz + body_h / 2), mat(c, spec=0.3, rough=0.6)),
            Part("bottom", gbox(sx - 0.008, sy, 0.025), (0, 0, -hz + 0.0125), mat(c, spec=0.3, rough=0.6)),
            Part("fold", gbox(sx, sy * 0.5, 0.04), (0, 0, hz - 0.02), mat(fold_c or dk(c, 0.92), spec=0.3, rough=0.6)),
            Part("seal", gbox(sx + 0.002, 0.010, 0.010), (0, 0, hz - 0.005), mat(dk(c, 0.75))),
            panel_y("front", f, sx - 0.006, body_h - 0.02, 0, -hz + body_h / 2, hy - 0.004, 1),
            panel_y("back", f, sx - 0.006, body_h - 0.02, 0, -hz + body_h / 2, hy - 0.004, -1)]


def tin(name, sx, sz, c, wrap, lid_h=0.02, body=SILVER, metal_lid=False, lid_ring=True):
    """Round can / tin / canister with a printed wrap and a lid of the catalogue colour."""
    r, hz = sx / 2, sz / 2
    lab_ = label(f"{name}_wrap", 2 * math.pi * r, sz - lid_h - 0.012, wrap)
    lidm = mat(c, spec=0.5, metal=0.4 if metal_lid else 0.0, rough=0.3)
    out = [Part("body", gcyl(r - 0.0015, sz - lid_h), (0, 0, -hz + (sz - lid_h) / 2), mat(body, spec=0.6, metal=0.4, rough=0.3)),
           band("wrap", lab_, r - 0.0015, sz - lid_h - 0.012, -hz + (sz - lid_h) / 2),
           Part("lid", gcyl(r, lid_h), (0, 0, hz - lid_h / 2), lidm),
           Part("bottom_rim", gcyl(r - 0.0005, 0.005), (0, 0, -hz + 0.0025), mat(body, spec=0.6, metal=0.4, rough=0.3))]
    if lid_ring:
        out.append(Part("lid_ring", gcyl(r * 0.84, 0.0015), (0, 0, hz + 0.0003), mat(dk(c, 0.88), spec=0.5, rough=0.3)))
    return out


def jug(name, sx, sy, sz, c, front, cap_c=None, spout_x=0.6):
    """Plastic jug with a moulded handle at the back and a screw cap at the front."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz * 0.76
    plastic = dict(spec=0.5, rough=0.3)
    f = label(f"{name}_front", sx - 0.03, body_h * 0.6, front)
    z0 = -hz
    rr = min(0.03, sy / 2)
    cap_x = hx * spout_x - 0.01
    return [
        Part("body_mid", gbox(sx - 2 * rr, sy, body_h), (0, 0, z0 + body_h / 2), mat(c, **plastic)),
        Part("body_a", gcyl(rr, body_h), (hx - rr, hy - rr, z0 + body_h / 2), mat(c, **plastic)),
        Part("body_b", gcyl(rr, body_h), (hx - rr, -hy + rr, z0 + body_h / 2), mat(c, **plastic)),
        Part("body_c", gbox(2 * rr, sy - 2 * rr, body_h), (hx - rr, 0, z0 + body_h / 2), mat(c, **plastic)),
        Part("back", gbox(2 * rr, sy, body_h), (-hx + rr, 0, z0 + body_h / 2), mat(c, **plastic)),
        Part("shoulder", gbox(sx - 0.02, sy - 0.012, sz * 0.08), (0, 0, z0 + body_h + sz * 0.04), mat(c, **plastic)),
        Part("handle_post", gbox(0.026, sy * 0.4, sz - body_h), (-hx + 0.02, 0, hz - (sz - body_h) / 2), mat(c, **plastic)),
        Part("handle_bar", gbox(sx * 0.4, sy * 0.4, 0.02), (-hx + sx * 0.2 + 0.005, 0, hz - 0.01), mat(c, **plastic)),
        Part("neck", gcyl(0.02, sz * 0.12), (cap_x, 0, z0 + body_h + sz * 0.06), mat(c, **plastic)),
        Part("cap", gcyl(0.024, sz * 0.06), (cap_x, 0, hz - sz * 0.03 - 0.003), mat(cap_c or dk(c, 0.7), spec=0.4)),
        panel_y("front", f, sx - 0.03, body_h * 0.6, 0.005, z0 + body_h * 0.5, hy, 1),
        panel_y("back_label", f, sx - 0.03, body_h * 0.6, 0.005, z0 + body_h * 0.5, hy, -1),
    ]


def pic_circle(col, inner=None, cx=0.5, cy=0.65, r=0.15):
    ops = [("circle", cx, cy, r, col)]
    if inner:
        ops.append(("circle", cx, cy, r * 0.6, inner))
    return ops


# ============================================================================ 38 more products (to 100)
@recipe("pasta_box")
def _pasta_box(sx, sy, sz, c):
    return carton("pasta", sx, sy, sz, c, print_ops(c, "SPAGHETTI", "n.5  durum wheat", "500 g", accent=(0.85, 0.75, 0.20),
                  picture=[("rect", 0.15, 0.55, 0.85, 0.80, (0.98, 0.90, 0.60))] + [("rect", 0.17, 0.58 + k * 0.035, 0.83, 0.59 + k * 0.035, (0.85, 0.70, 0.30)) for k in range(6)]))


@recipe("cracker_box")
def _cracker_box(sx, sy, sz, c):
    return carton("cracker", sx, sy, sz, c, print_ops(c, "CRACKERS", "baked, not fried", "250 g", accent=(0.95, 0.80, 0.25),
                  picture=[("rect", 0.30, 0.52, 0.70, 0.82, (0.90, 0.72, 0.40))] + [("circle", 0.38 + 0.08 * i, 0.60 + 0.1 * j, 0.012, (0.6, 0.4, 0.2)) for i in range(4) for j in range(3)]))


@recipe("sugar_bag")
def _sugar_bag(sx, sy, sz, c):
    return sack("sugar", sx, sy, sz, c, print_ops(c, "SUGAR", "white crystal", "1 kg", accent=(0.15, 0.25, 0.60),
                picture=pic_circle(WHITE, (0.95, 0.95, 0.98))))


@recipe("flour_bag")
def _flour_bag(sx, sy, sz, c):
    return sack("flour", sx, sy, sz, c, print_ops(c, "FLOUR", "all purpose  type 550", "2 kg", fg=(0.55, 0.20, 0.10), accent=(0.75, 0.30, 0.15),
                picture=[("circle", 0.40, 0.66, 0.06, (0.85, 0.70, 0.35)), ("circle", 0.50, 0.62, 0.06, (0.85, 0.70, 0.35)),
                         ("circle", 0.60, 0.66, 0.06, (0.85, 0.70, 0.35)), ("rect", 0.49, 0.66, 0.51, 0.80, (0.6, 0.45, 0.2))]))


@recipe("rice_bag")
def _rice_bag(sx, sy, sz, c):
    return sack("rice", sx, sy, sz, c, print_ops(c, "BASMATI", "long grain rice", "5 kg", accent=(0.85, 0.65, 0.15),
                picture=pic_circle((0.98, 0.97, 0.92), (0.92, 0.90, 0.82))), fold_c=(0.85, 0.65, 0.15))


@recipe("cat_litter_box")
def _cat_litter_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    out = carton("litter", sx, sy, sz, c, print_ops(c, "CLUMP CAT", "litter  odour control", "10 L", accent=(0.95, 0.75, 0.20),
                 picture=[("circle", 0.5, 0.66, 0.12, (0.95, 0.75, 0.20)), ("circle", 0.43, 0.56, 0.04, (0.95, 0.75, 0.20)),
                          ("circle", 0.57, 0.56, 0.04, (0.95, 0.75, 0.20))]), seam=False)
    out += [Part("handle_slot", gbox(0.10, 0.025, 0.001), (0, 0, hz + 0.0004), mat(dk(c, 0.3))),
            Part("handle_flap", gbox(0.12, 0.004, 0.012), (0, 0, hz - 0.005), mat(c, rough=0.6))]
    return out


@recipe("headphones_box")
def _headphones_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = [("bg", c), ("ring", 0.5, 0.48, 0.30, (0.85, 0.85, 0.88), 0.08), ("rect", 0.24, 0.45, 0.33, 0.80, (0.15, 0.15, 0.17)),
             ("rect", 0.67, 0.45, 0.76, 0.80, (0.15, 0.15, 0.17)), ("text", "AURA 700", 0.5, 0.12, 0.5, 0.12, WHITE),
             ("text", "noise cancelling", 0.5, 0.92, 0.6, 0.08, lt(c, 0.6))]
    out = carton("headphones", sx, sy, sz, c, front, side=[("bg", c), ("text", "AURA", 0.5, 0.5, 0.8, 0.3, WHITE)], seam=False)
    out.append(Part("sleeve_edge", gbox(0.004, sy + 0.002, sz + 0.002), (hx - 0.03, 0, 0), mat(lt(c, 0.3), spec=0.5)))
    return out


@recipe("tv_box")
def _tv_box(sx, sy, sz, c):
    front = [("bg", c), ("rect", 0.08, 0.12, 0.72, 0.78, (0.05, 0.05, 0.06)), ("rect", 0.10, 0.15, 0.70, 0.74, (0.15, 0.35, 0.65)),
             ("circle", 0.30, 0.45, 0.12, (0.95, 0.60, 0.15)), ("rect", 0.36, 0.78, 0.44, 0.84, (0.3, 0.3, 0.3)),
             ("text", "4K UHD", 0.86, 0.30, 0.22, 0.12, WHITE), ("text", "43\"", 0.86, 0.55, 0.15, 0.16, WHITE),
             ("text", "SMART TV", 0.40, 0.92, 0.5, 0.08, lt(c, 0.6))]
    hz = sz / 2
    out = carton("tv", sx, sy, sz, c, front, side=[("bg", c), ("text", "VISTA", 0.5, 0.3, 0.8, 0.12, WHITE),
                                                   ("arrow", 0.5, 0.65, 0.12, WHITE)], tape=True)
    out += [Part("grip_a", gbox(0.002, 0.08, 0.03), (sx / 2 + 0.0015, 0, hz - 0.06), mat((0.02, 0.02, 0.02))),
            Part("grip_b", gbox(0.002, 0.08, 0.03), (-sx / 2 - 0.0015, 0, hz - 0.06), mat((0.02, 0.02, 0.02)))]
    return out


@recipe("blender_box")
def _blender_box(sx, sy, sz, c):
    front = [("bg", c), ("rect", 0, 0, 1, 0.12, dk(c, 0.6)), ("text", "POWERMIX", 0.5, 0.06, 0.7, 0.08, WHITE),
             ("rect", 0.38, 0.25, 0.62, 0.62, (0.80, 0.90, 0.95)), ("rect", 0.40, 0.45, 0.60, 0.62, (0.90, 0.40, 0.50)),
             ("rect", 0.34, 0.62, 0.66, 0.80, (0.20, 0.20, 0.22)), ("circle", 0.5, 0.71, 0.03, (0.8, 0.8, 0.8)),
             ("text", "1200 W blender", 0.5, 0.90, 0.7, 0.07, WHITE)]
    return carton("blender", sx, sy, sz, c, front, side=print_ops(c, "1.5 L", "glass jar", "6 speeds"), tape=True)


@recipe("vacuum_box")
def _vacuum_box(sx, sy, sz, c):
    front = [("bg", c), ("text", "CYCLONE", 0.5, 0.10, 0.7, 0.09, (0.1, 0.1, 0.1)),
             ("rect", 0.47, 0.18, 0.53, 0.72, (0.25, 0.25, 0.28)), ("circle", 0.5, 0.25, 0.06, (0.6, 0.3, 0.6)),
             ("rect", 0.40, 0.72, 0.62, 0.80, (0.25, 0.25, 0.28)), ("text", "cordless vacuum", 0.5, 0.90, 0.75, 0.07, (0.1, 0.1, 0.1))]
    return carton("vacuum", sx, sy, sz, c, front, side=print_ops(c, "60 min", "battery", "", fg=(0.1, 0.1, 0.1)), tape=True)


@recipe("board_game_box")
def _board_game_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lid_h = sz * 0.75
    side = label("game_side", sx, lid_h - 0.006, [("bg", c), ("text", "ISLAND QUEST", 0.35, 0.5, 0.55, 0.55, (0.98, 0.85, 0.30)),
                                                ("text", "2-4 players  8+", 0.80, 0.5, 0.32, 0.4, WHITE)])
    end = label("game_end", sy, lid_h - 0.006, [("bg", c), ("text", "ISLAND QUEST", 0.5, 0.5, 0.85, 0.5, (0.98, 0.85, 0.30))])
    return [Part("base", gbox(sx - 0.004, sy - 0.004, sz - 0.004), (0, 0, -0.002), mat((0.15, 0.15, 0.15), rough=0.6)),
            Part("lid", gbox(sx, sy, lid_h), (0, 0, hz - lid_h / 2), mat(c, spec=0.4, rough=0.35)),
            Part("compass_ring", gcyl(sy * 0.12, 0.0015), (sx * 0.30, sy * 0.25, hz + 0.0003), mat((0.80, 0.65, 0.25))),
            panel_y("side_l", side, sx - 0.006, lid_h - 0.006, 0, hz - lid_h / 2, hy, 1),
            panel_y("side_r", side, sx - 0.006, lid_h - 0.006, 0, hz - lid_h / 2, hy, -1),
            panel_x("end_a", end, sy - 0.006, lid_h - 0.006, 0, hz - lid_h / 2, hx, 1),
            panel_x("end_b", end, sy - 0.006, lid_h - 0.006, 0, hz - lid_h / 2, hx, -1)]


@recipe("puzzle_box")
def _puzzle_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    side = label("puzzle_side", sx, sz - 0.006, [("bg", c), ("text", "1000 PIECES", 0.30, 0.5, 0.45, 0.55, WHITE),
                                                ("rect", 0.60, 0.2, 0.95, 0.8, (0.35, 0.55, 0.80)), ("rect", 0.60, 0.55, 0.95, 0.8, (0.30, 0.45, 0.20))])
    return [boxy("box", sx, sy, sz, 0, c, spec=0.4, rough=0.35),
            Part("lid_edge", gbox(sx + 0.002, sy + 0.002, 0.004), (0, 0, -hz + sz * 0.3), mat(dk(c, 0.8))),
            panel_y("side_l", side, sx - 0.006, sz * 0.6, 0, hz - sz * 0.32, hy, 1),
            panel_y("side_r", side, sx - 0.006, sz * 0.6, 0, hz - sz * 0.32, hy, -1)]


@recipe("drone_box")
def _drone_box(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = [("bg", c), ("rect", 0.35, 0.40, 0.65, 0.55, (0.85, 0.85, 0.88))] + [
        ("ring", x, y, 0.13, (0.85, 0.85, 0.88), 0.03) for x, y in ((0.25, 0.32), (0.75, 0.32), (0.25, 0.65), (0.75, 0.65))] + [
        ("text", "SKYCAM MINI", 0.5, 0.88, 0.6, 0.12, WHITE)]
    out = carton("drone", sx, sy, sz, c, front, side=[("bg", c), ("text", "4K", 0.5, 0.5, 0.5, 0.4, WHITE)], seam=False)
    out.append(Part("lid_line", gbox(sx + 0.002, sy + 0.002, 0.003), (0, 0, hz - 0.03), mat(dk(c, 0.7))))
    return out


@recipe("coffee_machine_box")
def _coffee_machine_box(sx, sy, sz, c):
    front = [("bg", c), ("rect", 0.30, 0.20, 0.70, 0.75, (0.15, 0.15, 0.16)), ("rect", 0.34, 0.28, 0.66, 0.40, (0.55, 0.55, 0.58)),
             ("rect", 0.42, 0.55, 0.58, 0.70, WHITE), ("rect", 0.44, 0.58, 0.56, 0.62, (0.45, 0.25, 0.10)),
             ("text", "BARISTA ONE", 0.5, 0.10, 0.6, 0.09, (0.95, 0.85, 0.65)), ("text", "espresso machine", 0.5, 0.88, 0.7, 0.07, (0.95, 0.85, 0.65))]
    return carton("coffeemachine", sx, sy, sz, c, front, side=print_ops(c, "15 bar", "milk frother", "", fg=(0.95, 0.85, 0.65)), tape=True)


@recipe("lightbulb_pack")
def _lightbulb_pack(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    front = print_ops(c, "LED 9W", "= 60 W   warm white", "2 pack", fg=(0.1, 0.1, 0.15), accent=(0.15, 0.30, 0.65),
                      picture=[("circle", 0.5, 0.62, 0.14, (0.98, 0.97, 0.90)), ("rect", 0.45, 0.70, 0.55, 0.80, (0.6, 0.6, 0.6))])
    out = carton("bulb", sx, sy, sz, c, front, seam=False)
    out.append(Part("tuck_flap", gbox(sx, sy, 0.003), (0, 0, hz - 0.0015), mat(dk(c, 0.92))))
    return out


@recipe("battery_pack")
def _battery_pack(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    card = label("battery_card", sx, sz, [("bg", c), ("rect", 0, 0, 1, 0.18, (0.08, 0.08, 0.08)),
                                         ("text", "VOLTMAX", 0.5, 0.09, 0.7, 0.12, (0.85, 0.65, 0.10)),
                                         ("text", "AA  x8", 0.5, 0.88, 0.5, 0.08, (0.08, 0.08, 0.08))])
    out = [Part("card", gbox(sx, 0.006, sz), (0, -hy + 0.003, 0), mat(c, rough=0.5)),
           panel_y("card_print", card, sx - 0.004, sz - 0.004, 0, 0, hy, -1),
           Part("blister", gbox(sx - 0.02, sy - 0.008, sz * 0.62), (0, 0.003, -0.02), mat((0.9, 0.95, 1.0), spec=0.9, rough=0.05), alpha=0.75),
           Part("hang_hole", gbox(0.03, 0.007, 0.008), (0, -hy + 0.003, hz - 0.015), mat((0.1, 0.1, 0.1)))]
    for i in range(4):
        x = -sx * 0.30 + i * sx * 0.20
        out += [Part(f"cell{i}", gcyl(0.007, sz * 0.5), (x, 0.002, -0.02), mat((0.08, 0.08, 0.08), spec=0.5)),
                Part(f"top{i}", gcyl(0.0071, sz * 0.12), (x, 0.002, -0.02 + sz * 0.19), mat((0.85, 0.60, 0.15), spec=0.6, metal=0.4))]
    out.append(Part("top_fill", gbox(sx, sy, 0.012), (0, 0, hz - 0.006), mat(c, rough=0.5)))
    return out


@recipe("toothpaste_box")
def _toothpaste_box(sx, sy, sz, c):
    front = [("bg", WHITE), ("rect", 0, 0, 0.35, 1, c), ("text", "FRESH", 0.175, 0.5, 0.30, 0.35, WHITE),
             ("text", "WHITENING TOOTHPASTE", 0.67, 0.35, 0.60, 0.20, c), ("text", "75 ml  fluoride", 0.67, 0.70, 0.45, 0.16, (0.3, 0.3, 0.35))]
    return carton("toothpaste", sx, sy, sz, c, front, side=[("bg", c), ("text", "75ml", 0.5, 0.5, 0.8, 0.3, WHITE)], seam=False)


@recipe("cereal_bar_box")
def _cereal_bar_box(sx, sy, sz, c):
    return carton("cerealbar", sx, sy, sz, c, print_ops(c, "OAT BARS", "honey & nuts", "6 x 25 g", fg=(0.25, 0.15, 0.05), accent=(0.75, 0.45, 0.15),
                  picture=[("rect", 0.25, 0.55, 0.75, 0.75, (0.80, 0.60, 0.30)), ("rect", 0.27, 0.57, 0.73, 0.73, (0.70, 0.50, 0.25))]))


@recipe("dishwasher_tabs")
def _dishwasher_tabs(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    out = carton("tabs", sx, sy, sz, c, print_ops(c, "SHINE ALL IN 1", "dishwasher tablets", "40 tabs", accent=(0.95, 0.80, 0.15),
                 picture=[("rect", 0.35, 0.55, 0.65, 0.78, WHITE), ("rect", 0.35, 0.55, 0.65, 0.63, (0.25, 0.55, 0.85)),
                          ("circle", 0.5, 0.70, 0.05, (0.85, 0.15, 0.15))]), seam=False)
    out += [Part("handle_slot", gbox(0.07, 0.02, 0.001), (0, 0, hz + 0.0004), mat(dk(c, 0.3))),
            Part("reseal", gbox(sx, 0.006, 0.001), (0, -sy * 0.3, hz + 0.0004), mat(dk(c, 0.7)))]
    return out


@recipe("water_bottle_pack")
def _water_bottle_pack(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    br = min(sx / 6, sy / 4) - 0.002
    body_h = sz * 0.68
    pet = mat((0.80, 0.90, 0.98), spec=0.9, rough=0.05)
    lab_ = label("waterpack_label", 2 * math.pi * br, 0.07, [("bg", WHITE), ("rect", 0, 0, 1, 0.3, c),
                                                            ("text", "ALPINE", 0.25, 0.62, 0.3, 0.4, dk(c, 0.6)),
                                                            ("text", "ALPINE", 0.75, 0.62, 0.3, 0.4, dk(c, 0.6))])
    out = [Part("film", gbox(sx, sy, body_h * 0.92), (0, 0, -hz + body_h * 0.46), mat((0.95, 0.97, 1.0), spec=0.9, rough=0.05), alpha=0.75),
           Part("film_top", gbox(sx - 0.02, sy - 0.02, 0.004), (0, 0, hz - 0.03), mat(c, spec=0.6, rough=0.2), alpha=0.35),
           Part("handle", gbox(0.12, 0.05, 0.003), (0, 0, hz - 0.001), mat(c, spec=0.5, rough=0.3))]
    for i in range(3):
        for j in range(2):
            x, y = -hx + sx * (i + 0.5) / 3, -hy + sy * (j + 0.5) / 2
            n = f"b{i}{j}"
            out += [Part(n, gcyl(br, body_h), (x, y, -hz + body_h / 2), pet, alpha=0.45),
                    Part(n + "_water", gcyl(br - 0.004, body_h - 0.02), (x, y, -hz + body_h / 2), mat(lt(c, 0.2), spec=0.6, rough=0.1), alpha=0.35),
                    Part(n + "_shoulder", gcyl(br * 0.7, sz * 0.14), (x, y, -hz + body_h + sz * 0.07), pet, alpha=0.4),
                    Part(n + "_neck", gcyl(0.014, sz * 0.10), (x, y, -hz + body_h + sz * 0.19), pet, alpha=0.3),
                    Part(n + "_cap", gcyl(0.016, sz * 0.06), (x, y, hz - sz * 0.03 - 0.002), mat(c, spec=0.4, rough=0.3)),
                    band(n + "_label", lab_, br, 0.07, -hz + body_h * 0.55)]
    return out


@recipe("milk_jug")
def _milk_jug(sx, sy, sz, c):
    return jug("milkjug", sx, sy, sz, c, [("bg", WHITE), ("rect", 0, 0, 1, 0.25, (0.85, 0.15, 0.15)),
                                          ("text", "WHOLE MILK", 0.5, 0.48, 0.85, 0.18, (0.85, 0.15, 0.15)),
                                          ("text", "1 GALLON  3.78 L", 0.5, 0.80, 0.8, 0.14, (0.2, 0.2, 0.3))], cap_c=(0.85, 0.15, 0.15))


@recipe("soda_bottle")
def _soda_bottle(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    body_h = sz * 0.62
    pet = mat((0.85, 0.90, 0.88), spec=0.9, rough=0.05)
    lab_ = label("cola_label", 2 * math.pi * r, 0.08, [("bg", (0.75, 0.05, 0.08)), ("text", "FIZZ COLA", 0.25, 0.45, 0.32, 0.45, WHITE),
                                                      ("text", "2 L", 0.75, 0.45, 0.12, 0.4, WHITE), ("rect", 0, 0.82, 1, 0.9, WHITE)])
    z0 = -hz
    out = [Part("bottle", gcyl(r, body_h), (0, 0, z0 + body_h / 2 + 0.01), pet, alpha=0.5),
           Part("cola", gcyl(r - 0.003, body_h + 0.03), (0, 0, z0 + body_h / 2 + 0.02), mat((0.18, 0.08, 0.04), spec=0.6, rough=0.2)),
           Part("shoulder", gsph(r), (0, 0, z0 + body_h + 0.01), mat(c, spec=0.8, rough=0.15), alpha=0.25),
           Part("neck", gcyl(0.014, sz - body_h - r - 0.02), (0, 0, hz - (sz - body_h - r - 0.02) / 2 - 0.014), pet, alpha=0.3),
           Part("cap", gcyl(0.016, 0.018), (0, 0, hz - 0.009), mat((0.75, 0.05, 0.08), spec=0.4)),
           band("label", lab_, r, 0.08, z0 + body_h * 0.55)]
    for k in range(5):                       # petaloid feet
        a = k * 2 * math.pi / 5
        out.append(Part(f"foot{k}", gsph(0.012), (r * 0.6 * math.cos(a), r * 0.6 * math.sin(a), z0 + 0.012), pet, alpha=0.4))
    return out


@recipe("hand_soap")
def _hand_soap(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz * 0.68
    lab_ = label("soap_label", sx - 0.01, body_h * 0.6, [("bg", WHITE), ("text", "LAVENDER", 0.5, 0.25, 0.85, 0.16, c),
                                                        ("text", "hand soap", 0.5, 0.48, 0.7, 0.12, (0.3, 0.3, 0.35)),
                                                        ("circle", 0.5, 0.75, 0.10, lt(c, 0.3))])
    z0 = -hz
    return [Part("body", gbox(sx, sy, body_h), (0, 0, z0 + body_h / 2), mat(c, spec=0.8, rough=0.15), alpha=0.2),
            Part("soap", gbox(sx - 0.006, sy - 0.006, body_h * 0.85), (0, 0, z0 + body_h * 0.43), mat(dk(c, 0.9), spec=0.6, rough=0.2)),
            Part("collar", gcyl(0.02, 0.015), (0, 0, z0 + body_h + 0.0075), mat(WHITE, spec=0.4)),
            Part("stem", gcyl(0.007, sz - body_h - 0.04), (0, 0, z0 + body_h + 0.015 + (sz - body_h - 0.04) / 2), mat(WHITE, spec=0.4)),
            Part("pump_head", gbox(0.03, 0.022, 0.018), (0, 0, hz - 0.016), mat(WHITE, spec=0.4)),
            Part("nozzle", gbox(0.04, 0.010, 0.010), (0.03, 0, hz - 0.012), mat(WHITE, spec=0.4)),
            Part("top_cover", gbox(sx, sy, 0.004), (0, 0, z0 + body_h - 0.002), mat(c, spec=0.6, rough=0.2)),
            panel_y("label", lab_, sx - 0.01, body_h * 0.6, 0, z0 + body_h * 0.45, hy, 1)]


@recipe("bleach_jug")
def _bleach_jug(sx, sy, sz, c):
    return jug("bleach", sx, sy, sz, c, [("bg", (0.15, 0.30, 0.65)), ("text", "ULTRA", 0.5, 0.2, 0.6, 0.15, WHITE),
                                         ("text", "BLEACH", 0.5, 0.45, 0.85, 0.22, WHITE), ("rect", 0, 0.70, 1, 1, (0.95, 0.75, 0.10)),
                                         ("text", "2.5 L  kills 99.9%", 0.5, 0.85, 0.85, 0.14, (0.1, 0.1, 0.2))], cap_c=(0.15, 0.30, 0.65))


@recipe("mouthwash")
def _mouthwash(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    body_h = sz * 0.78
    lab_ = label("mouthwash_label", sx - 0.01, body_h * 0.5, [("bg", WHITE), ("text", "COOL MINT", 0.5, 0.28, 0.85, 0.18, dk(c, 0.7)),
                                                             ("text", "mouthwash 500 ml", 0.5, 0.60, 0.85, 0.12, (0.3, 0.3, 0.35)),
                                                             ("rect", 0, 0.82, 1, 1, c)])
    z0 = -hz
    return [Part("bottle", gbox(sx, sy, body_h), (0, 0, z0 + body_h / 2), mat(c, spec=0.9, rough=0.05), alpha=0.35),
            Part("liquid", gbox(sx - 0.006, sy - 0.006, body_h - 0.02), (0, 0, z0 + body_h / 2 - 0.005), mat(lt(c, 0.15), spec=0.6, rough=0.1)),
            Part("shoulder", gbox(sx * 0.7, sy * 0.8, sz * 0.06), (0, 0, z0 + body_h + sz * 0.03), mat(c, spec=0.9, rough=0.05), alpha=0.3),
            Part("cap", gcyl(sy * 0.38, sz - body_h - sz * 0.06), (0, 0, hz - (sz - body_h - sz * 0.06) / 2), mat(dk(c, 0.75), spec=0.4)),
            panel_y("label", lab_, sx - 0.01, body_h * 0.5, 0, z0 + body_h * 0.45, hy, 1),
            panel_y("label_b", lab_, sx - 0.01, body_h * 0.5, 0, z0 + body_h * 0.45, hy, -1)]


@recipe("soup_can")
def _soup_can(sx, sy, sz, c):
    return tin("soup", sx, sz, c, [("bg", (0.80, 0.10, 0.10)), ("rect", 0, 0.55, 1, 1, WHITE),
                                   ("text", "TOMATO SOUP", 0.25, 0.28, 0.40, 0.22, WHITE), ("text", "TOMATO SOUP", 0.75, 0.28, 0.40, 0.22, WHITE),
                                   ("circle", 0.25, 0.78, 0.14, (0.85, 0.15, 0.10)), ("text", "400 g", 0.75, 0.78, 0.15, 0.16, (0.3, 0.3, 0.3))],
               lid_h=0.006, metal_lid=True)


@recipe("cookie_tin")
def _cookie_tin(sx, sy, sz, c):
    out = tin("cookies", sx, sz, c, [("bg", c), ("text", "BUTTER COOKIES", 0.25, 0.5, 0.42, 0.40, (0.95, 0.85, 0.50)),
                                     ("text", "BUTTER COOKIES", 0.75, 0.5, 0.42, 0.40, (0.95, 0.85, 0.50))],
              lid_h=0.025, body=c, metal_lid=True, lid_ring=False)
    r, hz = sx / 2, sz / 2
    out += [Part("lid_ring_gold", gcyl(r * 0.88, 0.0015), (0, 0, hz + 0.0003), mat((0.70, 0.58, 0.25), spec=0.6, metal=0.4)),
            Part("lid_centre", gcyl(r * 0.84, 0.0015), (0, 0, hz + 0.0008), mat(c, spec=0.5, metal=0.4, rough=0.3))]
    return out


@recipe("tennis_ball_can")
def _tennis_ball_can(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    ball = mat(c, rough=1.0)
    lab_ = label("tennis_label", 2 * math.pi * r, sz * 0.35, [("bg", (0.08, 0.08, 0.10)), ("text", "ACE PRO", 0.25, 0.4, 0.3, 0.35, c),
                                                             ("text", "3 balls", 0.75, 0.4, 0.2, 0.3, WHITE)])
    out = [Part("tube", gcyl(r, sz - 0.02), (0, 0, -0.01), mat((0.90, 0.95, 1.0), spec=0.9, rough=0.05), alpha=0.7),
           band("label", lab_, r, sz * 0.35, -hz + sz * 0.22),
           Part("lid", gcyl(r + 0.0005, 0.02), (0, 0, hz - 0.01), mat(c, spec=0.7, rough=0.1), alpha=0.45),
           Part("bottom", gcyl(r, 0.006), (0, 0, -hz + 0.003), mat(SILVER, metal=0.5, rough=0.3))]
    for k in range(3):
        z = -hz + 0.006 + (sz - 0.026) * (k + 0.5) / 3
        out.append(Part(f"ball{k}", gsph(r - 0.004), (0, 0, z), ball))
    return out


@recipe("popcorn_bucket")
def _popcorn_bucket(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    stripes = label("popcorn_wrap", 2 * math.pi * r, sz - 0.03, [("bg", WHITE)] + [
        ("rect", k / 12.0, 0, k / 12.0 + 1 / 24.0, 1, (0.85, 0.10, 0.10)) for k in range(12)] + [
        ("rect", 0.05, 0.35, 0.45, 0.65, (0.95, 0.85, 0.25)), ("text", "POPCORN", 0.25, 0.5, 0.36, 0.22, (0.85, 0.10, 0.10))])
    pop = mat(c, rough=1.0)
    out = [Part("bucket", gcyl(r * 0.9, sz - 0.03), (0, 0, -0.015), mat(WHITE, rough=0.7)),
           band("wrap", stripes, r * 0.9, sz - 0.03, -0.015),
           Part("rim", gcyl(r, 0.012), (0, 0, hz - 0.036), mat(WHITE, rough=0.7)),
           Part("heap", gcyl(r * 0.93, 0.02), (0, 0, hz - 0.024), pop)]
    for i in range(14):
        a, rr = i * 2.4, r * (0.15 + 0.6 * ((i * 37) % 10) / 10)
        out.append(Part(f"kernel{i}", gsph(0.012), (rr * math.cos(a), rr * math.sin(a), hz - 0.012), mat(lt(c, 0.25), rough=1.0)))
    return out


@recipe("oatmeal_canister")
def _oatmeal_canister(sx, sy, sz, c):
    return tin("oatmeal", sx, sz, c, [("bg", (0.95, 0.90, 0.78)), ("rect", 0, 0, 1, 0.15, c), ("rect", 0, 0.85, 1, 1, c),
                                      ("text", "OLD FARM", 0.25, 0.30, 0.36, 0.12, c), ("text", "ROLLED OATS", 0.25, 0.48, 0.40, 0.12, (0.3, 0.2, 0.1)),
                                      ("circle", 0.75, 0.50, 0.20, (0.85, 0.75, 0.55)), ("text", "1 kg", 0.25, 0.68, 0.15, 0.10, (0.3, 0.2, 0.1))],
               lid_h=0.022, body=(0.95, 0.90, 0.78))


@recipe("protein_tub")
def _protein_tub(sx, sy, sz, c):
    return tin("protein", sx, sz, c, [("bg", c), ("text", "WHEY", 0.25, 0.30, 0.30, 0.20, WHITE), ("text", "PROTEIN", 0.25, 0.50, 0.38, 0.13, (0.95, 0.75, 0.15)),
                                      ("text", "chocolate  2 kg", 0.25, 0.72, 0.36, 0.09, WHITE), ("rect", 0.55, 0.2, 0.95, 0.8, (0.95, 0.75, 0.15)),
                                      ("text", "24 g", 0.75, 0.5, 0.25, 0.2, c)], lid_h=0.035, body=c)


@recipe("pumpkin")
def _pumpkin(sx, sy, sz, c):
    r = sx / 2
    out = [Part("core", gsph(r * 0.86), (0, 0, -r * 0.1), mat(c, spec=0.3, rough=0.5))]
    for k in range(8):
        a = k * math.pi / 4
        out.append(Part(f"lobe{k}", gsph(r * 0.52), (r * 0.46 * math.cos(a), r * 0.46 * math.sin(a), -r * 0.12), mat(c if k % 2 else dk(c, 0.92), spec=0.3, rough=0.5)))
    out += [Part("stem", gcyl(0.014, 0.05), (0, 0, r * 0.72, 0.2, 0, 0), mat((0.35, 0.30, 0.15), rough=0.9)),
            Part("top", gsph(r * 0.55), (0, 0, r * 0.18), mat(c, spec=0.3, rough=0.5))]
    return out


@recipe("coconut")
def _coconut(sx, sy, sz, c):
    r = sx / 2
    fibre = mat(lt(c, 0.15), rough=1.0)
    out = [Part("shell", gsph(r), (0, 0, 0), mat(c, rough=1.0))]
    for k in range(10):
        a, el = k * 2.1, -0.6 + 0.13 * k
        x, y, z = math.cos(el) * math.cos(a), math.cos(el) * math.sin(a), math.sin(el)
        out.append(Part(f"fibre{k}", gbox(0.004, 0.03, 0.002), (x * r, y * r, z * r, 0, math.acos(z), math.atan2(y, x)), fibre))
    for k in range(3):                                   # the three "eyes" on the side
        a = k * 2 * math.pi / 3
        out.append(Part(f"eye{k}", gcyl(0.008, 0.004), (r * 0.97, 0.022 * math.cos(a), 0.022 * math.sin(a), 0, math.pi / 2, 0), mat((0.10, 0.06, 0.03))))
    return out


@recipe("orange_crate")
def _orange_crate(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    wood = mat((0.72, 0.58, 0.38), rough=0.95)
    tag = label("orange_tag", 0.14, 0.06, [("bg", (0.98, 0.95, 0.85)), ("text", "VALENCIA", 0.5, 0.35, 0.8, 0.35, (0.85, 0.40, 0.05)),
                                         ("text", "ORANGES 10 kg", 0.5, 0.75, 0.8, 0.28, (0.2, 0.3, 0.1))])
    out = [Part("floor", gbox(sx, sy, 0.015), (0, 0, -hz + 0.0075), wood),
           Part("fill", gbox(sx - 0.03, sy - 0.03, sz - 0.07), (0, 0, -hz + 0.015 + (sz - 0.07) / 2), mat(dk(c, 0.75), rough=0.8))]
    for s in (1, -1):
        for z in (-hz + 0.03, hz - 0.035):
            out.append(Part(f"side{s > 0}{z > 0}", gbox(sx, 0.012, 0.045), (0, s * (hy - 0.006), z), wood))
        out.append(Part(f"end{s > 0}", gbox(0.015, sy, sz - 0.01), (s * (hx - 0.0075), 0, -0.005), wood))
    nx, ny = 5, 4
    rr = min((sx - 0.03) / nx, (sy - 0.03) / ny) / 2
    for i in range(nx):
        for j in range(ny):
            x, y = -hx + 0.015 + (sx - 0.03) * (i + 0.5) / nx, -hy + 0.015 + (sy - 0.03) * (j + 0.5) / ny
            out.append(Part(f"orange{i}{j}", gsph(rr), (x, y, hz - rr), mat(c if (i + j) % 3 else lt(c, 0.08), spec=0.35, rough=0.5)))
    out.append(panel_y("tag", tag, 0.14, 0.04, 0, -0.01, hy, 1))
    return out


@recipe("cinder_block")
def _cinder_block(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    conc = dict(spec=0.02, rough=1.0)
    t = 0.03
    hole = mat(dk(c, 0.45), **conc)
    return [Part("wall_a", gbox(sx, t, sz), (0, hy - t / 2, 0), mat(c, **conc)),
            Part("wall_b", gbox(sx, t, sz), (0, -hy + t / 2, 0), mat(c, **conc)),
            Part("end_a", gbox(t, sy - 2 * t, sz), (hx - t / 2, 0, 0), mat(c, **conc)),
            Part("end_b", gbox(t, sy - 2 * t, sz), (-hx + t / 2, 0, 0), mat(c, **conc)),
            Part("web", gbox(t, sy - 2 * t, sz), (0, 0, 0), mat(c, **conc)),
            Part("hole_a", gbox(hx - 1.5 * t, sy - 2 * t, 0.004), (hx / 2, 0, hz - 0.03), hole),
            Part("hole_b", gbox(hx - 1.5 * t, sy - 2 * t, 0.004), (-hx / 2, 0, hz - 0.03), hole),
            Part("hole_fill", gbox(sx - 2 * t, sy - 2 * t, sz - 0.04), (0, 0, -0.02), mat(dk(c, 0.6), **conc))]


@recipe("car_battery")
def _car_battery(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lab_ = label("battery_label", sx - 0.02, sz * 0.55, [("bg", c), ("rect", 0, 0, 1, 0.25, (0.85, 0.65, 0.10)),
                                                       ("text", "POWERCELL", 0.5, 0.12, 0.6, 0.16, (0.1, 0.1, 0.1)),
                                                       ("text", "12V  70Ah  640A", 0.5, 0.50, 0.8, 0.16, WHITE),
                                                       ("rect", 0.1, 0.72, 0.9, 0.8, (0.85, 0.65, 0.10))])
    plastic = dict(spec=0.3, rough=0.5)
    return [Part("case", gbox(sx, sy, sz - 0.03), (0, 0, -0.015), mat(c, **plastic)),
            Part("lid", gbox(sx - 0.006, sy - 0.006, 0.02), (0, 0, hz - 0.03 + 0.0), mat(dk(c, 1.4), **plastic)),
            Part("handle", gbox(0.012, sy * 0.7, 0.008), (0, 0, hz - 0.016), mat((0.15, 0.15, 0.16), **plastic)),
            Part("term_pos", gcyl(0.011, 0.02), (hx - 0.035, hy - 0.035, hz - 0.01), mat((0.75, 0.62, 0.30), metal=0.5, rough=0.3)),
            Part("cap_pos", gcyl(0.016, 0.004), (hx - 0.035, hy - 0.035, hz - 0.018), mat((0.80, 0.10, 0.08))),
            Part("term_neg", gcyl(0.010, 0.02), (-hx + 0.035, hy - 0.035, hz - 0.01), mat((0.70, 0.70, 0.72), metal=0.5, rough=0.3)),
            panel_y("label", lab_, sx - 0.02, sz * 0.55, 0, -0.02, hy, 1)]


@recipe("potted_cactus")
def _potted_cactus(sx, sy, sz, c):
    r, hz = sx / 2, sz / 2
    clay = mat((0.60, 0.30, 0.15), spec=0.05, rough=0.95)
    pot_h = sz * 0.38
    plant = mat(c, spec=0.2, rough=0.7)
    spine = mat((0.92, 0.90, 0.80))
    out = [Part("pot", gcyl(r * 0.80, pot_h - 0.02), (0, 0, -hz + (pot_h - 0.02) / 2), clay),
           Part("rim", gcyl(r * 0.90, 0.025), (0, 0, -hz + pot_h - 0.0125), clay),
           Part("soil", gcyl(r * 0.84, 0.004), (0, 0, -hz + pot_h), mat((0.20, 0.13, 0.08), rough=1.0)),
           Part("pebbles", gcyl(r * 0.84, 0.002), (0, 0, -hz + pot_h + 0.003), mat((0.55, 0.52, 0.48), rough=1.0)),
           Part("stem", gcyl(r * 0.45, sz - pot_h - r * 0.45), (0, 0, -hz + pot_h + (sz - pot_h - r * 0.45) / 2), plant),
           Part("top", gsph(r * 0.45), (0, 0, hz - r * 0.45), plant),
           Part("arm_a", gcyl(r * 0.20, sz * 0.18), (r * 0.62, 0, 0.02), plant),
           Part("arm_a_join", gcyl(r * 0.18, r * 0.5), (r * 0.42, 0, -0.04, 0, math.pi / 2, 0), plant),
           Part("arm_a_tip", gsph(r * 0.20), (r * 0.62, 0, 0.02 + sz * 0.09), plant),
           Part("flower", gsph(r * 0.10), (0.01, 0, hz - 0.012), mat((0.95, 0.35, 0.55)))]
    for k in range(8):
        a = k * math.pi / 4
        out.append(Part(f"rib{k}", gbox(0.003, 0.003, sz * 0.4), (r * 0.45 * math.cos(a), r * 0.45 * math.sin(a), sz * 0.05), spine))
    return out


@recipe("teddy_bear")
def _teddy_bear(sx, sy, sz, c):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    fur = mat(c, rough=1.0)
    light = mat(lt(c, 0.35), rough=1.0)
    dark = mat((0.05, 0.04, 0.03), spec=0.6, rough=0.2)
    body_r = min(hx, hy) * 0.85
    head_r = body_r * 0.78
    zb = -hz + body_r + 0.01
    zh = hz - head_r - 0.02
    return [Part("body", gsph(body_r), (0, 0, zb), fur),
            Part("belly", gsph(body_r * 0.62), (0, body_r * 0.42, zb - 0.01), light),
            Part("leg_a", gsph(body_r * 0.42), (hx * 0.45, hy * 0.55, -hz + body_r * 0.42), fur),
            Part("leg_b", gsph(body_r * 0.42), (-hx * 0.45, hy * 0.55, -hz + body_r * 0.42), fur),
            Part("arm_a", gsph(body_r * 0.33), (hx * 0.80, hy * 0.25, zb + 0.02), fur),
            Part("arm_b", gsph(body_r * 0.33), (-hx * 0.80, hy * 0.25, zb + 0.02), fur),
            Part("head", gsph(head_r), (0, 0, zh), fur),
            Part("ear_a", gsph(head_r * 0.35), (head_r * 0.75, -0.01, zh + head_r * 0.75), fur),
            Part("ear_b", gsph(head_r * 0.35), (-head_r * 0.75, -0.01, zh + head_r * 0.75), fur),
            Part("muzzle", gsph(head_r * 0.42), (0, head_r * 0.75, zh - head_r * 0.2), light),
            Part("nose", gsph(head_r * 0.13), (0, head_r * 1.12, zh - head_r * 0.1), dark),
            Part("eye_a", gsph(head_r * 0.10), (head_r * 0.35, head_r * 0.88, zh + head_r * 0.25), dark),
            Part("eye_b", gsph(head_r * 0.10), (-head_r * 0.35, head_r * 0.88, zh + head_r * 0.25), dark),
            Part("bow", gbox(0.06, 0.015, 0.02), (0, body_r * 0.75, zb + body_r * 0.75), mat((0.80, 0.10, 0.15), spec=0.5))]


# ---------------------------------------------------------------------------- public
def has_recipe(ptype: Optional[str]) -> bool:
    return bool(ptype) and ptype in RECIPES


def product_parts(ptype: str, size: Tuple[float, float, float], rgb: RGB) -> List[Part]:
    sx, sy, sz = size
    return RECIPES[ptype](sx, sy, sz, tuple(rgb))


def product_visuals_xml(ptype: Optional[str], size: Tuple[float, float, float], rgb: RGB, fallback_geom: str,
                        yaw: float = 0.0) -> str:
    """<visual> elements of one product link. Unknown types (or a recipe that fails) get the plain
    catalogue shape in its colour, as before."""
    if has_recipe(ptype):
        try:
            return visuals_xml(product_parts(ptype, size, rgb), yaw)
        except Exception:                 # a broken look must never stop a product from spawning
            pass
    r, g, b = rgb
    pose = f"<pose>0 0 0 0 0 {yaw}</pose>" if yaw else ""
    return (f'<visual name="visual">{pose}<geometry>{fallback_geom}</geometry><material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse></material></visual>')
