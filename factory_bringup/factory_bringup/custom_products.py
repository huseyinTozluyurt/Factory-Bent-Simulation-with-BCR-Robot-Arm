#!/usr/bin/env python3
"""
Your own products (no ROS imports): anyone can add an object to the simulation, and it then
works like the built-in products of product_catalog.py - in the Order GUI, on the bents, in the
bins, for the OpenCV detectors, the colour classifier and the YOLO dataset / labs.

A product is either
  * a 3D model (mesh: .stl, .obj or .dae) - it is scaled to fit the size you give (or kept at
    its own size) and centred; its collision is the box (or cylinder) around it, or
  * a simple shape (box, cylinder, sphere) of your size and colour, optionally with a photo
    of the real product printed on its sides.
A photo also becomes the product's picture in the Order GUI.

Everything is kept in CUSTOM_DIR (shared by everyone using this workspace):
  custom_products.json        {"products": {type: {...}}}
  <type>/                     the copied mesh (+ .mtl / textures next to it) and photo
A product can be exported as one .zip and imported on another computer.

  python3 -m factory_bringup.custom_products add --name my_mug --label "Coffee mug" --mesh ~/mug.stl --size 0.12 0.09 0.10
  python3 -m factory_bringup.custom_products add --name juice_box --shape box --size 0.10 0.06 0.20 --color 0.6 0.3 0.1 --image juice.jpg
  python3 -m factory_bringup.custom_products list | remove NAME | export NAME FILE.zip | import FILE.zip
(or `ros2 run factory_bringup factory_products ...` after colcon build; or "Add your own product" in the Order GUI)

Rules that keep the vision code working (checked by validate()): size at most 0.48 x 0.34 x 0.42 m,
at least 6 cm high, a top of at least 60 cm2 and long side / short side <= 4.5. The colour (what the
overhead camera sees on top) is kept detector-safe: channels <= 0.6 (the hall light brightens ~1.65x),
not near black (dark belt) and not strongly green (the counting-gate marker).
"""

import argparse
import json
import math
import os
import re
import shutil
import struct
import sys
import time
import xml.etree.ElementTree as ET
import zipfile
from typing import Dict, List, Optional, Sequence, Tuple

CUSTOM_DIR = os.path.expanduser(os.environ.get("FACTORY_CUSTOM_PRODUCTS",
                                               "~/ros2_ws/src/factory_bent_project/factory_bringup/custom_products"))
CATALOG_FILE = "custom_products.json"
MESH_TYPES = (".stl", ".obj", ".dae")
IMAGE_TYPES = (".png", ".jpg", ".jpeg", ".bmp")
SHAPES = ("box", "cylinder", "sphere")
MAX_SIZE = (0.48, 0.34, 0.42)      # largest built-in product: the gripper, bins and spacing are made for this
MIN_HEIGHT = 0.06                  # the side camera needs 6 cm
MIN_TOP_AREA = 0.006               # the overhead detector's min_area_m2
MAX_ASPECT = 4.5                   # the overhead detector's max_aspect_ratio

Size = Tuple[float, float, float]
RGB = Tuple[float, float, float]


class ProductError(ValueError):
    pass


# ---------------------------------------------------------------------------- storage
def catalog_path() -> str:
    return os.path.join(CUSTOM_DIR, CATALOG_FILE)


def load() -> Dict[str, Dict]:
    """{type: entry} of every custom product (empty if there is none)."""
    try:
        with open(catalog_path()) as f:
            data = json.load(f).get("products", {})
    except (OSError, ValueError, AttributeError):
        return {}
    return {k: v for k, v in data.items() if isinstance(v, dict) and re.fullmatch(r"[a-z][a-z0-9_]{1,40}", k)}


def save(products: Dict[str, Dict]):
    os.makedirs(CUSTOM_DIR, exist_ok=True)
    tmp = catalog_path() + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"products": products}, f, indent=1, sort_keys=True)
    os.replace(tmp, catalog_path())          # nobody ever reads a half-written file


def catalog_mtime() -> float:
    try:
        return os.path.getmtime(catalog_path())
    except OSError:
        return 0.0


def entry_to_type(name: str, e: Dict) -> Dict:
    """The PRODUCT_TYPES entry of a custom product."""
    return {"size": tuple(e["size"]), "rgb": tuple(e["rgb"]), "shape": e.get("shape", "box"),
            "label": e.get("label", name), "custom": dict(e, dir=os.path.join(CUSTOM_DIR, name))}


# ---------------------------------------------------------------------------- checks
def slug(text: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "_", text.strip().lower()).strip("_")
    if s and not s[0].isalpha():
        s = "p_" + s
    return s[:40]


def validate_size(shape: str, size: Size) -> List[str]:
    """Problems that would stop the robot or the cameras from handling this size."""
    sx, sy, sz = size
    if shape in ("cylinder", "sphere"):
        sy = sx
    if shape == "sphere":
        sz = sx
    out = []
    for v, lim, axis in ((sx, MAX_SIZE[0], "x (length)"), (sy, MAX_SIZE[1], "y (width)"), (sz, MAX_SIZE[2], "z (height)")):
        if v > lim + 1e-9:
            out.append(f"{axis} {v:.3f} m is more than {lim} m")
        if v <= 0:
            out.append(f"{axis} must be positive")
    if sz < MIN_HEIGHT:
        out.append(f"height {sz:.3f} m is less than {MIN_HEIGHT} m (the side camera would not see it)")
    if sx * sy < MIN_TOP_AREA:
        out.append(f"top {sx * sy * 1e4:.0f} cm2 is less than {MIN_TOP_AREA * 1e4:.0f} cm2 (the overhead camera would not see it)")
    if min(sx, sy) > 0 and max(sx, sy) / min(sx, sy) > MAX_ASPECT:
        out.append(f"long side / short side {max(sx, sy) / min(sx, sy):.1f} is more than {MAX_ASPECT}")
    return out


def safe_color(rgb: Sequence[float]) -> Tuple[RGB, List[str]]:
    """The colour made detector-safe, and what was changed."""
    r, g, b = (max(0.0, min(1.0, float(v))) for v in rgb)
    notes = []
    m = max(r, g, b)
    if m > 0.6:
        k = 0.6 / m
        r, g, b = r * k, g * k, b * k
        notes.append("darkened so the hall light does not turn it white")
    if max(r, g, b) < 0.22:
        lift = 0.22 - max(r, g, b)
        r, g, b = r + lift, g + lift, b + lift
        notes.append("lightened: near-black tops vanish on the dark belt")
    if (g - r) > 0.14 and (g - b) > 0.14:
        g = max(r, b) + 0.14
        notes.append("less green: strong green is masked out as the counting-gate marker")
    return (round(r, 3), round(g, 3), round(b, 3)), notes


# ---------------------------------------------------------------------------- meshes
def _stl_points(path: str) -> List[Tuple[float, float, float]]:
    with open(path, "rb") as f:
        data = f.read()
    pts = []
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + n * 50 == len(data):                       # binary
            for i in range(n):
                off = 84 + i * 50 + 12
                v = struct.unpack("<9f", data[off:off + 36])
                pts += [v[0:3], v[3:6], v[6:9]]
            return pts
    for m in re.finditer(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data):
        pts.append(tuple(float(x) for x in m.groups()))
    return pts


def _obj_points(path: str) -> List[Tuple[float, float, float]]:
    pts = []
    with open(path, errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                pts.append((float(p[1]), float(p[2]), float(p[3])))
    return pts


def _dae_points(path: str) -> List[Tuple[float, float, float]]:
    root = ET.parse(path).getroot()
    ns = root.tag.split("}")[0] + "}" if root.tag.startswith("{") else ""
    unit = root.find(f"{ns}asset/{ns}unit")
    scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    up = root.find(f"{ns}asset/{ns}up_axis")
    y_up = up is not None and (up.text or "").strip().upper() == "Y_UP"
    pts = []
    for mesh in root.iter(f"{ns}mesh"):
        verts = mesh.find(f"{ns}vertices")
        if verts is None:
            continue
        src_id = None
        for inp in verts.findall(f"{ns}input"):
            if inp.get("semantic") == "POSITION":
                src_id = inp.get("source", "").lstrip("#")
        for src in mesh.findall(f"{ns}source"):
            if src.get("id") == src_id:
                arr = src.find(f"{ns}float_array")
                vals = [float(v) for v in (arr.text or "").split()]
                for i in range(0, len(vals) - 2, 3):
                    x, y, z = vals[i] * scale, vals[i + 1] * scale, vals[i + 2] * scale
                    pts.append((x, -z, y) if y_up else (x, y, z))      # Gazebo turns Y-up files to Z-up
    return pts


def mesh_bounds(path: str) -> Tuple[Size, Size]:
    """(min, max) corner of a mesh in metres as Gazebo will load it (units of .stl / .obj are taken as metres)."""
    ext = os.path.splitext(path)[1].lower()
    pts = {".stl": _stl_points, ".obj": _obj_points, ".dae": _dae_points}[ext](path)
    if not pts:
        raise ProductError(f"no vertices found in {path}")
    lo = tuple(min(p[i] for p in pts) for i in range(3))
    hi = tuple(max(p[i] for p in pts) for i in range(3))
    return lo, hi


def mesh_companions(path: str) -> List[str]:
    """Files next to a mesh that it needs: an .obj's .mtl files and their texture maps, a .dae's images."""
    folder = os.path.dirname(os.path.abspath(path))
    out: List[str] = []
    ext = os.path.splitext(path)[1].lower()
    try:
        if ext == ".obj":
            with open(path, errors="replace") as f:
                mtls = [ln.split(None, 1)[1].strip() for ln in f if ln.startswith("mtllib ")]
            for m in mtls:
                out.append(m)
                try:
                    with open(os.path.join(folder, m), errors="replace") as f:
                        out += [ln.split()[-1] for ln in f if ln.strip().split(" ")[0].lower().startswith("map_")]
                except OSError:
                    pass
        elif ext == ".dae":
            root = ET.parse(path).getroot()
            out += [(e.text or "").strip() for e in root.iter() if e.tag.endswith("init_from") and
                    (e.text or "").strip().lower().endswith(IMAGE_TYPES)]
    except (OSError, ET.ParseError):
        pass
    return [f for f in out if f and not os.path.isabs(f) and ".." not in f.replace("\\", "/").split("/")]


def _mtl_color(obj_path: str) -> Optional[RGB]:
    """Mean diffuse colour (Kd) of the .mtl next to an .obj, if any."""
    cols = []
    for f in os.listdir(os.path.dirname(obj_path) or "."):
        if f.lower().endswith(".mtl"):
            with open(os.path.join(os.path.dirname(obj_path) or ".", f), errors="replace") as fh:
                for line in fh:
                    if line.strip().startswith("Kd "):
                        cols.append(tuple(float(v) for v in line.split()[1:4]))
    if not cols:
        return None
    return tuple(sum(c[i] for c in cols) / len(cols) for i in range(3))


def image_color(path: str) -> Optional[RGB]:
    """Mean colour of the middle of a photo (its edges are often background)."""
    import cv2
    img = cv2.imread(path)
    if img is None:
        return None
    h, w = img.shape[:2]
    mid = img[h // 4: 3 * h // 4, w // 4: 3 * w // 4].reshape(-1, 3).mean(axis=0)
    return (mid[2] / 255.0, mid[1] / 255.0, mid[0] / 255.0)


# ---------------------------------------------------------------------------- add / remove / share
def builtin_types() -> set:
    from factory_bringup import product_catalog
    return {k for k, v in product_catalog.PRODUCT_TYPES.items() if "custom" not in v}


def add_product(name: str, label: str = "", shape: str = "box", size: Optional[Sequence[float]] = None,
                rgb: Optional[Sequence[float]] = None, mesh: str = "", image: str = "", recyclable: bool = False,
                keep_mesh_size: bool = False, author: str = "", replace: bool = False) -> Tuple[str, Dict, List[str]]:
    """Add (or with replace=True, update) a custom product. Returns (type, entry, notes)."""
    ptype = slug(name)
    if not ptype:
        raise ProductError("give the product a name")
    if ptype in builtin_types():
        raise ProductError(f"'{ptype}' is a built-in product; choose another name")
    products = load()
    if ptype in products and not replace:
        raise ProductError(f"a custom product '{ptype}' already exists (remove it first, or replace it)")
    if shape not in SHAPES:
        raise ProductError(f"shape must be one of {', '.join(SHAPES)}")
    mesh = os.path.expanduser(mesh) if mesh else ""
    image = os.path.expanduser(image) if image else ""
    if mesh and os.path.splitext(mesh)[1].lower() not in MESH_TYPES:
        raise ProductError(f"mesh must be {', '.join(MESH_TYPES)}")
    if mesh and not os.path.isfile(mesh):
        raise ProductError(f"mesh file not found: {mesh}")
    if image and (os.path.splitext(image)[1].lower() not in IMAGE_TYPES or not os.path.isfile(image)):
        raise ProductError(f"photo must be an existing {', '.join(IMAGE_TYPES)} file")
    notes: List[str] = []
    entry: Dict = {"label": label.strip() or name.strip(), "recyclable": bool(recyclable), "author": author,
                   "added": time.strftime("%Y-%m-%d %H:%M")}

    if mesh:
        lo, hi = mesh_bounds(mesh)
        ext = [hi[i] - lo[i] for i in range(3)]
        if min(ext) <= 0:
            raise ProductError("the mesh is flat (zero size along one axis)")
        if keep_mesh_size or not size:
            s = 1.0
        else:
            s = min(size[i] / ext[i] for i in range(3))              # fits inside the size given, keeps proportions
        fitted = tuple(round(ext[i] * s, 4) for i in range(3))
        centre = [(hi[i] + lo[i]) / 2 for i in range(3)]
        entry.update(mesh_scale=s, mesh_offset=[-centre[i] * s for i in range(3)])
        size = fitted
        if shape == "sphere":
            shape = "box"
        if shape == "cylinder":                                     # the cylinder round the mesh
            d = max(fitted[0], fitted[1])
            size = (d, d, fitted[2])
        notes.append(f"mesh scaled x{s:.3g} to {fitted[0]:.3f} x {fitted[1]:.3f} x {fitted[2]:.3f} m")
    if not size:
        raise ProductError("give the size (x y z in metres)")
    size = tuple(float(v) for v in size)
    if shape in ("cylinder", "sphere"):
        size = (size[0], size[0], size[0] if shape == "sphere" else size[2])
    problems = validate_size(shape, size)
    if problems:
        if mesh and keep_mesh_size:
            problems.append("if the model is in millimetres or another unit, give the size you want instead")
        raise ProductError("; ".join(problems))

    if rgb is None:
        rgb = (image_color(image) if image else None) or (_mtl_color(mesh) if mesh.lower().endswith(".obj") else None) \
            or (0.45, 0.45, 0.48)
        notes.append("colour taken from the " + ("photo" if image else "model" if mesh else "default"))
    rgb, cnotes = safe_color(rgb)
    notes += cnotes
    entry.update(shape=shape, size=list(size), rgb=list(rgb))

    # copy the files
    pdir = os.path.join(CUSTOM_DIR, ptype)
    if os.path.isdir(pdir) and replace:
        shutil.rmtree(pdir)
    os.makedirs(pdir, exist_ok=True)
    if mesh:
        src_dir = os.path.dirname(os.path.abspath(mesh))
        shutil.copy2(mesh, os.path.join(pdir, os.path.basename(mesh)))
        for f in mesh_companions(mesh):                              # .mtl and the textures the model uses
            src = os.path.join(src_dir, f)
            if os.path.isfile(src) and os.path.getsize(src) < 50e6:
                shutil.copy2(src, os.path.join(pdir, os.path.basename(f)))
        entry["mesh"] = os.path.basename(mesh)
    if image:
        dst = "photo" + os.path.splitext(image)[1].lower()
        shutil.copy2(image, os.path.join(pdir, dst))
        entry["image"] = dst
    products[ptype] = entry
    save(products)
    write_preview(ptype, entry)
    return ptype, entry, notes


def remove_product(name: str) -> bool:
    products = load()
    if name not in products:
        return False
    del products[name]
    save(products)
    shutil.rmtree(os.path.join(CUSTOM_DIR, name), ignore_errors=True)
    try:
        from factory_bringup.product_catalog import PRODUCT_IMAGE_DIR
        os.remove(os.path.join(os.path.expanduser(PRODUCT_IMAGE_DIR), f"{name}.png"))
    except OSError:
        pass
    return True


def export_product(name: str, zip_path: str) -> str:
    """One .zip with the product's entry and files, to give to someone else."""
    products = load()
    if name not in products:
        raise ProductError(f"no custom product '{name}'")
    zip_path = os.path.expanduser(zip_path)
    pdir = os.path.join(CUSTOM_DIR, name)
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("product.json", json.dumps({"type": name, "entry": products[name]}, indent=1))
        if os.path.isdir(pdir):
            for f in os.listdir(pdir):
                z.write(os.path.join(pdir, f), f"files/{f}")
    return zip_path


def import_product(zip_path: str, replace: bool = False) -> str:
    """Add the product of an exported .zip; returns its type."""
    with zipfile.ZipFile(os.path.expanduser(zip_path)) as z:
        meta = json.loads(z.read("product.json"))
        name, entry = slug(meta["type"]), meta["entry"]
        if name in builtin_types():
            raise ProductError(f"'{name}' is a built-in product")
        products = load()
        if name in products and not replace:
            raise ProductError(f"a custom product '{name}' already exists")
        problems = validate_size(entry.get("shape", "box"), tuple(entry["size"]))
        if problems:
            raise ProductError("; ".join(problems))
        entry["rgb"] = list(safe_color(entry["rgb"])[0])
        pdir = os.path.join(CUSTOM_DIR, name)
        shutil.rmtree(pdir, ignore_errors=True)
        os.makedirs(pdir)
        for info in z.infolist():
            base = os.path.basename(info.filename)
            if info.filename.startswith("files/") and base and not base.startswith("."):
                with open(os.path.join(pdir, base), "wb") as f:      # flat names only: no path tricks
                    f.write(z.read(info))
    products[name] = entry
    save(products)
    write_preview(name, entry)
    return name


def write_preview(ptype: str, entry: Dict):
    """The product's picture for the Order GUI: the photo, else a drawing of its shape and colour."""
    import cv2
    import numpy as np
    from factory_bringup.product_catalog import PRODUCT_IMAGE_DIR
    out = os.path.join(os.path.expanduser(PRODUCT_IMAGE_DIR), f"{ptype}.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    canvas = np.full((256, 256, 3), 179, np.uint8)                   # same grey as the Gazebo pictures
    img = None
    if entry.get("image"):
        img = cv2.imread(os.path.join(CUSTOM_DIR, ptype, entry["image"]))
    if img is not None:
        h, w = img.shape[:2]
        k = 236.0 / max(h, w)
        img = cv2.resize(img, (max(1, int(w * k)), max(1, int(h * k))), interpolation=cv2.INTER_AREA)
        y0, x0 = (256 - img.shape[0]) // 2, (256 - img.shape[1]) // 2
        canvas[y0:y0 + img.shape[0], x0:x0 + img.shape[1]] = img
    else:
        r, g, b = entry["rgb"]
        col = (int(b * 1.5 * 255) if b * 1.5 < 1 else 255, int(min(1, g * 1.5) * 255), int(min(1, r * 1.5) * 255))
        sx, sy, sz = entry["size"]
        k = 170.0 / max(sx, sz)
        w, h = int(sx * k), int(sz * k)
        if entry.get("shape") == "sphere":
            cv2.circle(canvas, (128, 128), w // 2, col, -1, cv2.LINE_AA)
        elif entry.get("shape") == "cylinder":
            cv2.rectangle(canvas, (128 - w // 2, 128 - h // 2), (128 + w // 2, 128 + h // 2), col, -1)
            cv2.ellipse(canvas, (128, 128 - h // 2), (w // 2, max(4, w // 8)), 0, 0, 360,
                        tuple(min(255, c + 30) for c in col), -1, cv2.LINE_AA)
        else:
            cv2.rectangle(canvas, (128 - w // 2, 128 - h // 2), (128 + w // 2, 128 + h // 2), col, -1)
        if entry.get("mesh"):
            cv2.putText(canvas, "3D model", (70, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 40, 40), 2, cv2.LINE_AA)
    cv2.imwrite(out, canvas)


# ---------------------------------------------------------------------------- the look (used by product_models)
def custom_visuals(ptype: str, t: Dict, yaw: float = 0.0) -> str:
    """<visual> elements of a custom product (mesh, or shape with the photo on its sides)."""
    from factory_bringup import product_models as M
    e = t["custom"]
    pdir = e.get("dir") or os.path.join(CUSTOM_DIR, ptype)
    sx, sy, sz = t["size"]
    c = tuple(t["rgb"])
    parts: List = []
    if e.get("mesh") and os.path.isfile(os.path.join(pdir, e["mesh"])):
        s = float(e.get("mesh_scale", 1.0))
        ox, oy, oz = e.get("mesh_offset", (0.0, 0.0, 0.0))
        geom = f"<mesh><uri>{os.path.join(pdir, e['mesh'])}</uri><scale>{s:.6g} {s:.6g} {s:.6g}</scale></mesh>"
        part = M.Part("mesh", geom, (ox, oy, oz), M.mat(c))
        part.mat = None                                          # keep the model's own materials
        parts.append(part)
        return M.visuals_xml(parts, yaw)
    photo = os.path.join(pdir, e["image"]) if e.get("image") else ""
    tex = None
    if photo and os.path.isfile(photo):
        tex = f"custom_{ptype}_photo"
        M.LABELS[tex] = (0, 0, [("file", photo, os.path.getmtime(photo))], t["shape"] == "cylinder")
    if t["shape"] == "sphere":
        parts.append(M.Part("body", M.gsph(sx / 2), (0, 0, 0), M.mat(c, spec=0.3, rough=0.5)))
    elif t["shape"] == "cylinder":
        parts.append(M.Part("body", M.gcyl(sx / 2, sz), (0, 0, 0), M.mat(c, spec=0.3, rough=0.5)))
        parts.append(M.Part("rim", M.gcyl(sx / 2 * 0.85, 0.0015), (0, 0, sz / 2 + 0.0003), M.mat(M.dk(c, 0.85))))
        if tex:
            parts.append(M.band("photo", tex, sx / 2, sz * 0.8, 0))
    else:
        parts.append(M.boxy("body", sx, sy, sz, 0, c, spec=0.25, rough=0.6))
        parts.append(M.Part("seam", M.gbox(sx, 0.003, 0.001), (0, 0, sz / 2 + 0.0004), M.mat(M.dk(c, 0.6))))
        if tex:
            parts.append(M.panel_y("photo_f", tex, sx - 0.004, sz - 0.004, 0, 0, sy / 2, 1))
            parts.append(M.panel_y("photo_b", tex, sx - 0.004, sz - 0.004, 0, 0, sy / 2, -1))
    return M.visuals_xml(parts, yaw)


# ---------------------------------------------------------------------------- command line
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="factory_products", description="Add your own products to the factory simulation.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("add", help="add a product (a 3D model or a simple shape)")
    a.add_argument("--name", required=True, help="short name, becomes the type, e.g. my_mug")
    a.add_argument("--label", default="", help="name shown in the GUI")
    a.add_argument("--shape", default="box", choices=SHAPES, help="collision shape (and look without a mesh)")
    a.add_argument("--size", type=float, nargs=3, metavar=("X", "Y", "Z"), help="metres (cylinder: diameter diameter height)")
    a.add_argument("--color", type=float, nargs=3, metavar=("R", "G", "B"), help="0..1, the colour seen from above")
    a.add_argument("--mesh", default="", help=".stl / .obj / .dae (textures and .mtl next to it are copied too)")
    a.add_argument("--keep-mesh-size", action="store_true", help="use the model's own size instead of --size")
    a.add_argument("--image", default="", help="photo of the product (.png / .jpg)")
    a.add_argument("--recyclable", action="store_true")
    a.add_argument("--author", default=os.environ.get("USER", ""))
    a.add_argument("--replace", action="store_true", help="overwrite a custom product of the same name")
    sub.add_parser("list", help="list the custom products")
    r = sub.add_parser("remove", help="remove a custom product")
    r.add_argument("name")
    e = sub.add_parser("export", help="write a product as one .zip to share")
    e.add_argument("name")
    e.add_argument("zip")
    i = sub.add_parser("import", help="add a product from an exported .zip")
    i.add_argument("zip")
    i.add_argument("--replace", action="store_true")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "add":
            ptype, entry, notes = add_product(args.name, args.label, args.shape, args.size, args.color, args.mesh, args.image,
                                              args.recyclable, args.keep_mesh_size, args.author, args.replace)
            print(f"Added '{ptype}': {entry['shape']} {entry['size']} colour {entry['rgb']}")
            for n in notes:
                print("  - " + n)
            print("Running nodes pick it up for new orders; restart detectors / labs to include it in their classes.")
        elif args.cmd == "list":
            products = load()
            if not products:
                print(f"No custom products yet ({catalog_path()}).")
            for k, v in sorted(products.items()):
                kind = f"mesh {v['mesh']}" if v.get("mesh") else v.get("shape", "box")
                print(f"{k:24s} {v.get('label', k):24s} {kind:22s} size {v['size']}  rgb {v['rgb']}"
                      f"{'  recyclable' if v.get('recyclable') else ''}")
        elif args.cmd == "remove":
            print("Removed." if remove_product(args.name) else f"No custom product '{args.name}'.")
        elif args.cmd == "export":
            print("Wrote " + export_product(args.name, args.zip))
        elif args.cmd == "import":
            print(f"Imported '{import_product(args.zip, args.replace)}'.")
    except (ProductError, OSError, KeyError, ValueError, zipfile.BadZipFile) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
