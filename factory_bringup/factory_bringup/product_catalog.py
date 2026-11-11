#!/usr/bin/env python3
"""
Product types that can ride the bents, and helpers to spawn an ordered row of them
(no ROS imports; used by the attacher and the Product Order GUI).

A spawned product is named  order_<type>_<number>,  e.g. order_paint_can_3, so its type
(size, colour, shape) can always be looked up from the name.
"""

import json
import math
import os
import re
from typing import Dict, List, Optional, Tuple

# type: size (x, y, z) in m (cylinder: diameter, diameter, height; sphere: diameter x3), RGB 0..1, shape, label
PRODUCT_TYPES: Dict[str, Dict] = {
    # the products of dual_circular_factory_conveyor.sdf
    "shipping_box_large": {"size": (0.34, 0.26, 0.26), "rgb": (0.45, 0.29, 0.13), "shape": "box", "label": "Shipping box (large)"},
    "milk_carton":        {"size": (0.16, 0.10, 0.34), "rgb": (0.97, 0.97, 0.98), "shape": "box", "label": "Milk carton"},
    "bottle_boxy":        {"size": (0.12, 0.12, 0.42), "rgb": (0.25, 0.60, 0.95), "shape": "box", "label": "Bottle (boxy)"},
    "sneaker_pair":       {"size": (0.36, 0.20, 0.12), "rgb": (0.12, 0.12, 0.15), "shape": "box", "label": "Sneaker pair"},
    "packet_pouch":       {"size": (0.18, 0.06, 0.24), "rgb": (0.88, 0.80, 0.15), "shape": "box", "label": "Packet pouch"},
    "shoe_box":           {"size": (0.34, 0.20, 0.14), "rgb": (0.14, 0.20, 0.45), "shape": "box", "label": "Shoe box"},
    "long_package":       {"size": (0.48, 0.18, 0.18), "rgb": (0.72, 0.20, 0.20), "shape": "box", "label": "Long package"},
    "pickup_box":         {"size": (0.34, 0.26, 0.26), "rgb": (0.90, 0.25, 0.10), "shape": "box", "label": "Pickup box"},
    # new types
    "cereal_box":         {"size": (0.20, 0.08, 0.30), "rgb": (0.95, 0.55, 0.10), "shape": "box", "label": "Cereal box"},
    "detergent_bottle":   {"size": (0.14, 0.10, 0.30), "rgb": (0.10, 0.70, 0.60), "shape": "box", "label": "Detergent bottle"},
    "soda_six_pack":      {"size": (0.24, 0.16, 0.13), "rgb": (0.85, 0.10, 0.15), "shape": "box", "label": "Soda six-pack"},
    "book_parcel":        {"size": (0.30, 0.22, 0.08), "rgb": (0.85, 0.75, 0.55), "shape": "box", "label": "Book parcel"},
    "toy_box":            {"size": (0.26, 0.26, 0.20), "rgb": (0.55, 0.30, 0.80), "shape": "box", "label": "Toy box"},
    "paint_can":          {"size": (0.17, 0.17, 0.19), "rgb": (0.75, 0.75, 0.78), "shape": "cylinder", "label": "Paint can"},
    "glass_jar":          {"size": (0.11, 0.11, 0.16), "rgb": (0.60, 0.85, 0.95), "shape": "cylinder", "label": "Glass jar"},
    "poster_tube":        {"size": (0.10, 0.10, 0.40), "rgb": (0.95, 0.85, 0.35), "shape": "cylinder", "label": "Poster tube (standing)"},
    # more types (the overhead camera needs a top of at least ~0.006 m2, the side camera 6 cm height)
    "pizza_box":          {"size": (0.34, 0.34, 0.08), "rgb": (0.82, 0.68, 0.45), "shape": "box", "label": "Pizza box"},
    "tissue_box":         {"size": (0.24, 0.12, 0.10), "rgb": (0.95, 0.60, 0.75), "shape": "box", "label": "Tissue box"},
    "gift_box":           {"size": (0.25, 0.25, 0.25), "rgb": (0.80, 0.05, 0.35), "shape": "box", "label": "Gift box"},
    "tool_case":          {"size": (0.40, 0.20, 0.12), "rgb": (0.95, 0.70, 0.05), "shape": "box", "label": "Tool case"},
    "laptop_box":         {"size": (0.40, 0.30, 0.08), "rgb": (0.70, 0.72, 0.76), "shape": "box", "label": "Laptop box"},
    "pet_food_bag":       {"size": (0.30, 0.12, 0.40), "rgb": (0.24, 0.42, 0.30), "shape": "box", "label": "Pet food bag"},
    "kettle_box":         {"size": (0.24, 0.24, 0.28), "rgb": (0.55, 0.75, 0.90), "shape": "box", "label": "Kettle box"},
    "electronics_box":    {"size": (0.30, 0.22, 0.12), "rgb": (0.15, 0.45, 0.50), "shape": "box", "label": "Electronics box"},
    "water_jug":          {"size": (0.20, 0.20, 0.32), "rgb": (0.30, 0.55, 0.95), "shape": "cylinder", "label": "Water jug"},
    "coffee_can":         {"size": (0.12, 0.12, 0.15), "rgb": (0.45, 0.25, 0.12), "shape": "cylinder", "label": "Coffee can"},
    "flower_pot":         {"size": (0.20, 0.20, 0.18), "rgb": (0.80, 0.40, 0.20), "shape": "cylinder", "label": "Flower pot"},
    "candle_jar":         {"size": (0.11, 0.11, 0.12), "rgb": (0.96, 0.90, 0.75), "shape": "cylinder", "label": "Candle jar"},
    "basketball":         {"size": (0.24, 0.24, 0.24), "rgb": (0.90, 0.45, 0.10), "shape": "sphere", "label": "Basketball"},
    "football":           {"size": (0.22, 0.22, 0.22), "rgb": (0.96, 0.96, 0.96), "shape": "sphere", "label": "Football"},
    # realistic grocery / household products. The colour is what the overhead camera sees on top;
    # channels stay <= 0.6 because the hall light makes colours ~1.65x brighter (lighter ones turn white),
    # and nothing may be strongly green (the overhead detector drops the green counting gate)
    "wine_bottle":        {"size": (0.10, 0.10, 0.34), "rgb": (0.12, 0.26, 0.16), "shape": "cylinder", "label": "Wine bottle"},
    "olive_oil_bottle":   {"size": (0.09, 0.09, 0.30), "rgb": (0.45, 0.42, 0.10), "shape": "box", "label": "Olive oil bottle"},
    "ice_cream_tub":      {"size": (0.17, 0.17, 0.12), "rgb": (0.30, 0.55, 0.48), "shape": "cylinder", "label": "Ice cream tub"},
    "egg_carton":         {"size": (0.31, 0.12, 0.075), "rgb": (0.45, 0.40, 0.55), "shape": "box", "label": "Egg carton"},
    "motor_oil_jug":      {"size": (0.20, 0.11, 0.28), "rgb": (0.40, 0.06, 0.08), "shape": "box", "label": "Motor oil jug"},
    "cooler_box":         {"size": (0.44, 0.28, 0.30), "rgb": (0.04, 0.40, 0.55), "shape": "box", "label": "Cooler box"},
    "spray_can":          {"size": (0.10, 0.10, 0.24), "rgb": (0.48, 0.58, 0.10), "shape": "cylinder", "label": "Spray paint can"},
    "coffee_bean_sack":   {"size": (0.34, 0.22, 0.14), "rgb": (0.50, 0.40, 0.25), "shape": "box", "label": "Coffee bean sack"},
    "shampoo_bottle":     {"size": (0.11, 0.07, 0.24), "rgb": (0.60, 0.22, 0.35), "shape": "box", "label": "Shampoo bottle"},
    "bowling_ball":       {"size": (0.22, 0.22, 0.22), "rgb": (0.25, 0.08, 0.40), "shape": "sphere", "label": "Bowling ball"},
    # more grocery, household and DIY products (same colour rules; tops must not be near black either,
    # or the overhead contour detector cannot tell them from the dark belt)
    "juice_carton":       {"size": (0.10, 0.07, 0.24), "rgb": (0.60, 0.32, 0.04), "shape": "box", "label": "Orange juice carton"},
    "tea_box":            {"size": (0.18, 0.08, 0.10), "rgb": (0.35, 0.12, 0.25), "shape": "box", "label": "Tea box"},
    "printer_paper":      {"size": (0.30, 0.21, 0.065), "rgb": (0.20, 0.35, 0.55), "shape": "box", "label": "Printer paper ream"},
    "phone_box":          {"size": (0.17, 0.09, 0.065), "rgb": (0.60, 0.42, 0.38), "shape": "box", "label": "Smartphone box"},
    "diaper_pack":        {"size": (0.40, 0.22, 0.30), "rgb": (0.35, 0.50, 0.60), "shape": "box", "label": "Diaper pack"},
    "beer_crate":         {"size": (0.40, 0.30, 0.28), "rgb": (0.52, 0.38, 0.05), "shape": "box", "label": "Beer crate (20 bottles)"},
    "bread_loaf":         {"size": (0.30, 0.13, 0.12), "rgb": (0.60, 0.50, 0.25), "shape": "box", "label": "Bread loaf"},
    "watermelon":         {"size": (0.26, 0.26, 0.26), "rgb": (0.18, 0.30, 0.16), "shape": "sphere", "label": "Watermelon"},
    "chips_tube":         {"size": (0.09, 0.09, 0.25), "rgb": (0.10, 0.30, 0.30), "shape": "cylinder", "label": "Chips tube"},
    "paint_bucket":       {"size": (0.30, 0.30, 0.32), "rgb": (0.30, 0.38, 0.45), "shape": "cylinder", "label": "Paint bucket (15 L)"},
    "honey_jar":          {"size": (0.10, 0.10, 0.13), "rgb": (0.40, 0.30, 0.05), "shape": "cylinder", "label": "Honey jar"},
    "ketchup_bottle":     {"size": (0.10, 0.065, 0.20), "rgb": (0.60, 0.05, 0.05), "shape": "box", "label": "Ketchup bottle"},
    "car_tire":           {"size": (0.40, 0.40, 0.16), "rgb": (0.48, 0.48, 0.52), "shape": "cylinder", "label": "Car wheel (lying)"},
    "traffic_cone":       {"size": (0.30, 0.30, 0.40), "rgb": (0.60, 0.22, 0.02), "shape": "cylinder", "label": "Traffic cone"},
    "fire_extinguisher":  {"size": (0.14, 0.14, 0.42), "rgb": (0.55, 0.04, 0.06), "shape": "cylinder", "label": "Fire extinguisher"},
    "cabbage":            {"size": (0.20, 0.20, 0.20), "rgb": (0.35, 0.45, 0.30), "shape": "sphere", "label": "Cabbage"},
    "microwave_box":      {"size": (0.48, 0.34, 0.30), "rgb": (0.42, 0.32, 0.20), "shape": "box", "label": "Microwave box"},
    "first_aid_kit":      {"size": (0.26, 0.18, 0.09), "rgb": (0.15, 0.35, 0.28), "shape": "box", "label": "First aid kit"},
    "globe":              {"size": (0.30, 0.30, 0.30), "rgb": (0.08, 0.25, 0.55), "shape": "sphere", "label": "Desk globe"},
    "wooden_crate":       {"size": (0.40, 0.30, 0.25), "rgb": (0.55, 0.40, 0.22), "shape": "box", "label": "Wooden crate"},
    "baby_formula_tin":   {"size": (0.13, 0.13, 0.18), "rgb": (0.58, 0.40, 0.55), "shape": "cylinder", "label": "Baby formula tin"},
    "kettlebell":         {"size": (0.20, 0.20, 0.26), "rgb": (0.25, 0.25, 0.27), "shape": "cylinder", "label": "Kettlebell (8 kg)"},
    # to 100: packaged food, household, electronics, DIY and fresh produce (same colour rules)
    "pasta_box":          {"size": (0.20, 0.06, 0.12), "rgb": (0.10, 0.20, 0.50), "shape": "box", "label": "Pasta box"},
    "cracker_box":        {"size": (0.22, 0.07, 0.16), "rgb": (0.55, 0.15, 0.08), "shape": "box", "label": "Cracker box"},
    "sugar_bag":          {"size": (0.15, 0.09, 0.20), "rgb": (0.45, 0.48, 0.58), "shape": "box", "label": "Sugar bag"},
    "flour_bag":          {"size": (0.18, 0.10, 0.28), "rgb": (0.58, 0.55, 0.48), "shape": "box", "label": "Flour bag"},
    "rice_bag":           {"size": (0.22, 0.08, 0.30), "rgb": (0.50, 0.10, 0.15), "shape": "box", "label": "Rice bag"},
    "cat_litter_box":     {"size": (0.30, 0.20, 0.32), "rgb": (0.35, 0.25, 0.45), "shape": "box", "label": "Cat litter box"},
    "headphones_box":     {"size": (0.22, 0.20, 0.09), "rgb": (0.10, 0.18, 0.22), "shape": "box", "label": "Headphones box"},
    "tv_box":             {"size": (0.48, 0.12, 0.32), "rgb": (0.18, 0.18, 0.20), "shape": "box", "label": "TV box"},
    "blender_box":        {"size": (0.24, 0.24, 0.40), "rgb": (0.50, 0.10, 0.30), "shape": "box", "label": "Blender box"},
    "vacuum_box":         {"size": (0.34, 0.30, 0.42), "rgb": (0.55, 0.45, 0.05), "shape": "box", "label": "Vacuum cleaner box"},
    "board_game_box":     {"size": (0.40, 0.27, 0.08), "rgb": (0.10, 0.30, 0.20), "shape": "box", "label": "Board game"},
    "puzzle_box":         {"size": (0.34, 0.24, 0.06), "rgb": (0.45, 0.30, 0.55), "shape": "box", "label": "Jigsaw puzzle"},
    "drone_box":          {"size": (0.32, 0.24, 0.12), "rgb": (0.30, 0.32, 0.34), "shape": "box", "label": "Drone box"},
    "coffee_machine_box": {"size": (0.36, 0.30, 0.40), "rgb": (0.25, 0.15, 0.10), "shape": "box", "label": "Coffee machine box"},
    "lightbulb_pack":     {"size": (0.12, 0.07, 0.14), "rgb": (0.60, 0.55, 0.20), "shape": "box", "label": "LED bulb pack"},
    "battery_pack":       {"size": (0.14, 0.05, 0.20), "rgb": (0.40, 0.40, 0.05), "shape": "box", "label": "Battery blister pack"},
    "toothpaste_box":     {"size": (0.20, 0.05, 0.065), "rgb": (0.55, 0.20, 0.25), "shape": "box", "label": "Toothpaste box"},
    "cereal_bar_box":     {"size": (0.18, 0.06, 0.10), "rgb": (0.45, 0.55, 0.15), "shape": "box", "label": "Cereal bar box"},
    "dishwasher_tabs":    {"size": (0.22, 0.12, 0.12), "rgb": (0.10, 0.35, 0.45), "shape": "box", "label": "Dishwasher tablets"},
    "water_bottle_pack":  {"size": (0.27, 0.18, 0.33), "rgb": (0.30, 0.45, 0.58), "shape": "box", "label": "Water 6-pack"},
    "milk_jug":           {"size": (0.16, 0.16, 0.26), "rgb": (0.55, 0.56, 0.58), "shape": "box", "label": "Milk jug (1 gal)"},
    "soda_bottle":        {"size": (0.11, 0.11, 0.33), "rgb": (0.22, 0.10, 0.06), "shape": "cylinder", "label": "Soda bottle (2 L)"},
    "hand_soap":          {"size": (0.09, 0.07, 0.20), "rgb": (0.55, 0.30, 0.45), "shape": "box", "label": "Hand soap pump"},
    "bleach_jug":         {"size": (0.16, 0.10, 0.28), "rgb": (0.50, 0.52, 0.58), "shape": "box", "label": "Bleach jug"},
    "mouthwash":          {"size": (0.10, 0.07, 0.24), "rgb": (0.05, 0.35, 0.40), "shape": "box", "label": "Mouthwash bottle"},
    "soup_can":           {"size": (0.09, 0.09, 0.12), "rgb": (0.52, 0.52, 0.50), "shape": "cylinder", "label": "Soup can"},
    "cookie_tin":         {"size": (0.20, 0.20, 0.08), "rgb": (0.12, 0.22, 0.45), "shape": "cylinder", "label": "Cookie tin"},
    "tennis_ball_can":    {"size": (0.09, 0.09, 0.26), "rgb": (0.58, 0.60, 0.20), "shape": "cylinder", "label": "Tennis ball can"},
    "popcorn_bucket":     {"size": (0.20, 0.20, 0.22), "rgb": (0.60, 0.55, 0.35), "shape": "cylinder", "label": "Popcorn bucket"},
    "oatmeal_canister":   {"size": (0.13, 0.13, 0.24), "rgb": (0.45, 0.20, 0.10), "shape": "cylinder", "label": "Oatmeal canister"},
    "protein_tub":        {"size": (0.15, 0.15, 0.24), "rgb": (0.40, 0.15, 0.45), "shape": "cylinder", "label": "Protein powder tub"},
    "pumpkin":            {"size": (0.28, 0.28, 0.28), "rgb": (0.60, 0.28, 0.04), "shape": "sphere", "label": "Pumpkin"},
    "coconut":            {"size": (0.14, 0.14, 0.14), "rgb": (0.30, 0.20, 0.12), "shape": "sphere", "label": "Coconut"},
    "orange_crate":       {"size": (0.40, 0.30, 0.18), "rgb": (0.60, 0.38, 0.08), "shape": "box", "label": "Crate of oranges"},
    "cinder_block":       {"size": (0.39, 0.19, 0.19), "rgb": (0.42, 0.42, 0.40), "shape": "box", "label": "Cinder block"},
    "car_battery":        {"size": (0.28, 0.18, 0.19), "rgb": (0.38, 0.38, 0.42), "shape": "box", "label": "Car battery"},
    "potted_cactus":      {"size": (0.16, 0.16, 0.30), "rgb": (0.28, 0.40, 0.30), "shape": "cylinder", "label": "Potted cactus"},
    "teddy_bear":         {"size": (0.20, 0.16, 0.26), "rgb": (0.45, 0.30, 0.18), "shape": "box", "label": "Teddy bear"},
}

ORDER_PREFIX = "order_"

# Products that users added themselves (custom_products.py: Order GUI "Add your own product" or
# `python3 -m factory_bringup.custom_products add ...`) join PRODUCT_TYPES and RECYCLABLE here, and
# again whenever their file changes (refresh_custom_products), so running nodes pick up new ones.
_custom_mtime = -1.0


def refresh_custom_products(force: bool = False) -> bool:
    """Load the custom products into PRODUCT_TYPES / RECYCLABLE if their file changed; True if it did."""
    global _custom_mtime
    from factory_bringup import custom_products
    mtime = custom_products.catalog_mtime()
    if mtime == _custom_mtime and not force:
        return False
    _custom_mtime = mtime
    for k in [k for k, v in PRODUCT_TYPES.items() if "custom" in v]:
        del PRODUCT_TYPES[k]
        RECYCLABLE.discard(k)
    for k, e in custom_products.load().items():
        if k in PRODUCT_TYPES:                 # never shadow a built-in product
            continue
        try:
            PRODUCT_TYPES[k] = custom_products.entry_to_type(k, e)
        except (KeyError, TypeError, ValueError):
            continue
        if e.get("recyclable"):
            RECYCLABLE.add(k)
    try:
        from factory_bringup.product_models import PRODUCT_SIZES
        PRODUCT_SIZES.clear()
    except ImportError:
        pass
    return True


def type_of(model_name: Optional[str]) -> Optional[str]:
    """Product type of a model name: order_<type>_<n>, product_<type>, or a world name like pickup_box."""
    if not model_name:
        return None
    if not _known(model_name):
        refresh_custom_products()              # maybe a product someone just added
    m = re.fullmatch(rf"{ORDER_PREFIX}(.+)_(\d+)", model_name)
    if m and m.group(1) in PRODUCT_TYPES:
        return m.group(1)
    if model_name.startswith("product_") and model_name[len("product_"):] in PRODUCT_TYPES:
        return model_name[len("product_"):]
    return model_name if model_name in PRODUCT_TYPES else None


def _known(model_name: str) -> bool:
    m = re.fullmatch(rf"{ORDER_PREFIX}(.+)_(\d+)", model_name)
    name = m.group(1) if m else model_name[len("product_"):] if model_name.startswith("product_") else model_name
    return name in PRODUCT_TYPES


def geometry_xml(shape: str, size: Tuple[float, float, float]) -> str:
    sx, sy, sz = size
    if shape == "cylinder":
        return f"<cylinder><radius>{sx / 2.0}</radius><length>{sz}</length></cylinder>"
    if shape == "sphere":
        return f"<sphere><radius>{sx / 2.0}</radius></sphere>"
    return f"<box><size>{sx} {sy} {sz}</size></box>"


def product_visuals(ptype: Optional[str], size: Optional[Tuple[float, float, float]] = None,
                    rgb: Optional[Tuple[float, float, float]] = None, shape: Optional[str] = None, yaw: float = 0.0) -> str:
    """<visual> elements of a product: its realistic multi-part look with printed labels
    (product_models.py) inside the catalogue size; the plain shape for anything else."""
    from factory_bringup.product_models import PRODUCT_SIZES, product_visuals_xml
    if not PRODUCT_SIZES:
        PRODUCT_SIZES.update({k: (v["size"], v["rgb"]) for k, v in PRODUCT_TYPES.items() if "custom" not in v})
    t = PRODUCT_TYPES.get(ptype or "", {})
    if "custom" in t:
        from factory_bringup.custom_products import custom_visuals
        try:
            return custom_visuals(ptype, dict(t, size=size or t["size"], rgb=rgb or t["rgb"]), yaw)
        except Exception:                       # a broken custom look must never stop a spawn
            pass
    size = size or t.get("size", (0.3, 0.3, 0.3))
    rgb = rgb or t.get("rgb", (0.9, 0.2, 0.1))
    geom = geometry_xml(shape or t.get("shape", "box"), size)
    return product_visuals_xml(ptype if t else None, size, rgb, geom, yaw)


def product_sdf(name: str, ptype: str, twist: Optional[Tuple[float, float]] = None, gravity: bool = False,
                mass: float = 0.2) -> str:
    """SDF of one product model; with twist it orbits like the world's products (VelocityControl).
    The collision is the simple catalogue shape; the visuals are the product's realistic look."""
    t = PRODUCT_TYPES[ptype]
    geom = geometry_xml(t["shape"], t["size"])
    plugin = ""
    if twist is not None:
        plugin = (f'<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">'
                  f'<initial_linear>{twist[0]} 0 0</initial_linear><initial_angular>0 0 {twist[1]}</initial_angular></plugin>')
    return (f'<?xml version="1.0" ?><sdf version="1.8"><model name="{name}"><static>false</static>{plugin}'
            f'<link name="link"><gravity>{"true" if gravity else "false"}</gravity>'
            f'<velocity_decay><linear>0</linear><angular>0</angular></velocity_decay>'
            f'<inertial><mass>{mass}</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial>'
            f'<collision name="collision"><geometry>{geom}</geometry></collision>'
            f'{product_visuals(ptype)}</link></model></sdf>')


def max_items(spacing: float, start_offset: float, radius: float) -> int:
    """How many products fit on the ring at this spacing (keeps one spacing free before the first)."""
    circ = 2 * math.pi * radius
    return max(0, int((circ - start_offset - spacing) // spacing) + 1) if spacing > 0 else 0


def order_poses(n: int, spacing: float, start_offset: float, center: Tuple[float, float], radius: float,
                ref_angle: float = 0.0, ccw: bool = True) -> List[Tuple[float, float, float]]:
    """
    (x, y, yaw) for n products on a ring, equal arc distance `spacing` apart. Item 0 is
    `start_offset` m before ref_angle (the catch station) against the travel direction,
    so it arrives first; item 1 is `spacing` behind it, and so on.
    """
    out = []
    sign = 1.0 if ccw else -1.0
    for i in range(n):
        a = ref_angle - sign * (start_offset + i * spacing) / radius
        x, y = center[0] + radius * math.cos(a), center[1] + radius * math.sin(a)
        out.append((x, y, a + sign * math.pi / 2))      # heading along the travel direction
    return out


# Photo booth for the product pictures in the order GUI (capture_product_images): a camera
# in the world (<model name="product_photo_camera">) away from the bents, looking down at
# the products from the front. Each product is put on the camera's axis, closer or farther
# so that it fills about half of the picture.
PHOTO_CAMERA_POSE = (0.0, 11.0, 1.6, 0.0, 0.5, 1.5708)    # x y z roll pitch yaw, same as the world file
PHOTO_CAMERA_HFOV = 0.6
PHOTO_PRODUCT_YAW = 0.5                                    # turned so two sides show
PRODUCT_IMAGE_DIR = "~/ros2_ws/src/factory_bent_project/factory_bringup/product_images"   # <type>.png


def photo_product_pose(ptype: str, fill: float = 0.55) -> Tuple[float, float, float, float]:
    """(x, y, z, yaw) of a product in front of the photo camera."""
    x, y, z, _, pitch, yaw = PHOTO_CAMERA_POSE
    size = max(PRODUCT_TYPES[ptype]["size"])
    d = max(0.45, size / (fill * 2.0 * math.tan(PHOTO_CAMERA_HFOV / 2.0)))
    dx, dy, dz = math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), -math.sin(pitch)
    return x + d * dx, y + d * dy, z + d * dz, PHOTO_PRODUCT_YAW


# Where the arm takes each product type: "bent2" (put on the second bent, the default),
# "recycling" or "trash" (dropped into the bin of that name next to the arm, see the world's
# <model name="recycling_bin"> / "trash_bin"). Chosen per product in the order GUI and kept
# in DESTINATIONS_FILE as {"destinations": {type: "recycling" | "trash"}}; vision_pick_place
# reads the file again for every pick, so a change counts from the next object on.
DESTINATIONS = ("bent2", "recycling", "trash")
DESTINATIONS_FILE = "~/.ros/factory_destinations.json"
BINS = {   # name: (x, y) of the bin centre in the world, same as the world file
    "recycling": (0.0, 2.2),
    "trash": (0.0, -2.2),
}
BIN_SIZE = (0.9, 0.9, 0.95)          # outer x, y, height (open at the top)
# suggestion for the GUI button "Recyclables to recycling": cardboard, paper, plastic, glass, metal
RECYCLABLE = {
    "shipping_box_large", "milk_carton", "bottle_boxy", "shoe_box", "long_package", "pickup_box",
    "cereal_box", "detergent_bottle", "soda_six_pack", "book_parcel", "toy_box", "glass_jar",
    "poster_tube", "pizza_box", "tissue_box", "gift_box", "laptop_box", "kettle_box",
    "electronics_box", "water_jug", "coffee_can",
    "wine_bottle", "olive_oil_bottle", "egg_carton", "spray_can", "shampoo_bottle",
    "juice_carton", "tea_box", "printer_paper", "phone_box", "beer_crate", "honey_jar", "ketchup_bottle",
    "microwave_box", "baby_formula_tin",
    "pasta_box", "cracker_box", "sugar_bag", "flour_bag", "headphones_box", "tv_box", "blender_box", "vacuum_box",
    "board_game_box", "puzzle_box", "drone_box", "coffee_machine_box", "lightbulb_pack", "toothpaste_box",
    "cereal_bar_box", "dishwasher_tabs", "water_bottle_pack", "milk_jug", "soda_bottle", "hand_soap", "bleach_jug",
    "mouthwash", "soup_can", "cookie_tin", "oatmeal_canister", "protein_tub",
}


refresh_custom_products()


def load_destinations(path: str = DESTINATIONS_FILE) -> Dict[str, str]:
    """{type: "recycling" | "trash"} from the file; types not in it go to bent 2."""
    try:
        with open(os.path.expanduser(path)) as f:
            d = json.load(f).get("destinations", {})
    except (OSError, ValueError, AttributeError):
        return {}
    return {t: v for t, v in d.items() if t in PRODUCT_TYPES and v in ("recycling", "trash")}


def save_destinations(dest: Dict[str, str], path: str = DESTINATIONS_FILE):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    clean = {t: v for t, v in sorted(dest.items()) if v in ("recycling", "trash")}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"destinations": clean}, f, indent=1)
    os.replace(tmp, path)                 # the arm never reads a half-written file


def destination_of(model_name: Optional[str], dest: Optional[Dict[str, str]] = None) -> str:
    """Where the arm takes this model: "bent2", "recycling" or "trash"."""
    t = type_of(model_name)
    d = load_destinations() if dest is None else dest
    return d.get(t, "bent2") if t else "bent2"


# ---------------------------------------------------------------------------- worlds
# All worlds share the arm, the cameras, the catch station (-2.58, 0), the place point and the bins;
# what differs is listed here by Gazebo world name and set on every node at start-up
# (apply_world_layout). run_factory.sh --world ... exports the name in FACTORY_WORLD_NAME, and
# factory_full.launch.py sets it from the world file.
# A straight belt is handled as a ring of LINE_RADIUS m: over its 20 m it is straight to 5 mm,
# so every ring formula (order rows, belt twist, the camera's search lane) still holds.
WORLD_ENV = "FACTORY_WORLD_NAME"
DUAL_WORLD = "dual_circular_factory_world"
WORLD_FILES = {   # run_factory.sh --world <key>: (file in my_factory_worlds/worlds, Gazebo world name)
    "dual": ("dual_circular_factory_conveyor.sdf", DUAL_WORLD),
    "circular": ("circular_factory_conveyor.sdf", "circular_factory_world"),
    "straight": ("factory_conveyor.sdf", "factory_world"),
    # experimental two-conveyor worlds (make_exp_worlds.py); circular -> circular is the dual world
    "straight-circular": ("exp_straight_circular.sdf", "exp_straight_circular_world"),
    "circular-straight": ("exp_circular_straight.sdf", "exp_circular_straight_world"),
    "straight-straight": ("exp_straight_straight.sdf", "exp_straight_straight_world"),
}
LINE_RADIUS = 10000.0
STATION_X = -2.58
PLACE_LINE_X = 2.22    # straight bent 2 of the experimental worlds: x of its product lane
_NO_BENT2 = {"has_bent2": False, "ring2_center_x": 1.0e6, "ring2_moving": False}
_LINE1 = dict(            # bent 1 straight: x = STATION_X, belt y -15.4 .. 5.4, products move +y
    ring1_center_x=STATION_X - LINE_RADIUS, ring1_center_y=0.0, ring1_radius=LINE_RADIUS,
    product_path_radius=LINE_RADIUS,
    ring_inner_radius=LINE_RADIUS - 0.32, ring_outer_radius=LINE_RADIUS + 0.33,
    bent1_path_length=15.0,          # belt before the catch station (it starts at y = -15.4)
    bent1_end=True, bent1_end_y=5.4,  # products past the end drop into the collection box
    collection_box_x=-2.58, collection_box_y=8.6, collection_box_floor_z=0.135,
)
_LINE2 = dict(            # bent 2 straight: x = PLACE_LINE_X, belt y 15.4 .. -5.4, products move -y
    ring2_center_x=PLACE_LINE_X + LINE_RADIUS, ring2_center_y=0.0,
    ring2_inner_radius=LINE_RADIUS - 0.4, ring2_outer_radius=LINE_RADIUS + 0.4,   # release onto the belt
    ring2_product_radius=LINE_RADIUS,
    ring2_spawn_reference_angle=-math.pi + 0.336 / LINE_RADIUS,                   # (2.22, -0.34), as on the ring
    ring2_lane_inner_radius=LINE_RADIUS - 0.32, ring2_lane_outer_radius=LINE_RADIUS + 0.33,  # camera lane
    place_x=PLACE_LINE_X, place_y=-0.3736, place_yaw=-math.pi / 2,
    bent2_path_length=15.0,          # belt before the place point (it starts at y = 15.4)
    bent2_end=True, bent2_end_y=-5.4,  # products past the end drop into collection box 2
    collection_box2_x=PLACE_LINE_X, collection_box2_y=-8.6, collection_box2_floor_z=0.135,
)
WORLD_LAYOUTS: Dict[str, Dict] = {
    DUAL_WORLD: {},
    "circular_factory_world": dict(_NO_BENT2),
    "factory_world": dict(_NO_BENT2, **_LINE1),
    "exp_straight_circular_world": dict(_LINE1, ring2_lane_inner_radius=3.10, ring2_lane_outer_radius=3.75),
    "exp_circular_straight_world": dict(_LINE2),
    "exp_straight_straight_world": dict(_LINE1, **_LINE2),
}


def world_name(default: str = DUAL_WORLD) -> str:
    """Gazebo world name from $FACTORY_WORLD_NAME, else default."""
    return os.environ.get(WORLD_ENV) or default


def world_has_bent2(name: Optional[str] = None) -> bool:
    return WORLD_LAYOUTS.get(name or world_name(), {}).get("has_bent2", True)


def bent1_capacity(spacing: float, start_offset: float, name: Optional[str] = None) -> int:
    """How many products one order can put on bent 1 of this world (max_items, and for a straight
    belt only what fits before the catch station)."""
    layout = WORLD_LAYOUTS.get(name or world_name(), {})
    n = max_items(spacing, start_offset, layout.get("ring1_radius", 3.42))
    length = layout.get("bent1_path_length", 0.0)
    if length > 0 and spacing > 0:
        n = min(n, max(0, int((length - start_offset) // spacing) + 1))
    return n


def bent2_capacity(spacing: float, start_offset: float, name: Optional[str] = None) -> int:
    """How many products one order can put on bent 2 of this world (as bent1_capacity)."""
    layout = WORLD_LAYOUTS.get(name or world_name(), {})
    n = max_items(spacing, start_offset, layout.get("ring2_product_radius", 3.42))
    length = layout.get("bent2_path_length", 0.0)
    if length > 0 and spacing > 0:
        n = min(n, max(0, int((length - start_offset) // spacing) + 1))
    return n


def apply_world_layout(node) -> str:
    """Set the parameters of this node that the world's layout changes; returns the world name.
    The name is $FACTORY_WORLD_NAME if set, else the node's world_name parameter."""
    from rclpy.parameter import Parameter
    name = os.environ.get(WORLD_ENV, "")
    if node.has_parameter("world_name"):
        if name:
            node.set_parameters([Parameter("world_name", value=name)])
        name = str(node.get_parameter("world_name").value)
    name = name or DUAL_WORLD
    layout = WORLD_LAYOUTS.get(name)
    if layout is None:
        node.get_logger().warn(f"Unknown world '{name}': using the dual circular world's layout.")
        return name
    params = [Parameter(k, value=v) for k, v in layout.items() if node.has_parameter(k)]
    if params:
        node.set_parameters(params)
    node.get_logger().info(f"World '{name}'" + (f": set {', '.join(p.name for p in params)}" if params else ""))
    return name

