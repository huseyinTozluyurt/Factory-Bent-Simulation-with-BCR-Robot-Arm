#!/usr/bin/env python3
"""
Product Order GUI (PyQt5) - choose which products go on the first bent, in which order.

Left: every product type of product_catalog.py (the world's products and the new ones),
shown with its picture from the simulation (product_images/<type>.png, taken in Gazebo by
capture_product_images; button "Take pictures in Gazebo"), and a large preview with size
and shape of the selected one. Add types to the order on the right, sort them with
Up / Down, then "Spawn order" puts them on bent 1 in that order, the same distance
apart along the belt; the first one arrives at the catch station first. The vision
pick and place then picks them one by one as usual. The "Bent 2" tab does the same on
the second bent (the first one arrives at the release point first).

"Simulation": one belt speed for both bents (m/s), the simulation speed (x real time),
a run time after which the arm and both bents stop (0 = no limit) and the arm movement
(motion_type of vision_pick_place: auto, cycle, random or one of the six types); "Apply"
sends them to the attacher on /factory_settings and the movement to vision_pick_place.

ROS side (served by bcr_endpoint_object_attacher):
  publishes   /spawn_order          std_msgs/String JSON {"bent": 1|2, "items": [type, ...], "spacing", "start_offset", "clear"}
              /factory_settings     std_msgs/String JSON {"belt_speed", "real_time_factor", "run_minutes"}
  subscribes  /spawn_order/result   std_msgs/String JSON {"ok", "message"}
  calls       /stop_bent1 /start_bent1 /stop_bent2 /start_bent2  (std_srvs/Trigger)
              /vision_pick_place/set_parameters  (motion_type)

Look: dark factory theme (STYLE below), a header with a status light (green = the attacher
is listening), search box and shape filter for the products, numbered order lists that can
also be sorted by drag and drop, coloured conveyor buttons and a time-stamped log.

Bins: every product type goes to bent 2 unless "Goes to" (under the selected product) says
recycling or trash; then the arm drops it into that bin next to it. The choice is saved at
once in ~/.ros/factory_destinations.json (product_catalog.DESTINATIONS_FILE), which
vision_pick_place reads for every pick; the pictures get a badge (R = recycling, T = trash).

Run:  ros2 run factory_bringup order_gui

Selection mode (used by run_factory.sh before anything else starts; no ROS needed):
      ros2 run factory_bringup order_gui --select --output ~/.ros/factory_order.json
  The window opens with the last selection; "Start simulation" writes the order to the file
  and exits with code 0 (closing the window exits with 3 = cancelled). The attacher then
  spawns that file's order when Gazebo is up (startup_order_file).
"""

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

from PyQt5.QtCore import QProcess, QSize, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QIcon, QPainter, QPixmap
from PyQt5.QtWidgets import (QAbstractItemView, QApplication, QCheckBox, QColorDialog, QComboBox, QDialog,
                             QDialogButtonBox, QDoubleSpinBox, QFileDialog, QFormLayout, QFrame, QGroupBox,
                             QHBoxLayout, QLabel, QLineEdit, QListView, QListWidget, QListWidgetItem, QMessageBox,
                             QPlainTextEdit, QPushButton, QRadioButton, QSpinBox, QStyle, QTabWidget, QVBoxLayout,
                             QWidget)

from factory_bringup import custom_products
from factory_bringup.product_catalog import (PRODUCT_IMAGE_DIR, PRODUCT_TYPES, RECYCLABLE, bent1_capacity,
                                             bent2_capacity, load_destinations, refresh_custom_products,
                                             save_destinations, world_has_bent2)

RING1_RADIUS = 3.42   # must match ring1_radius of the attacher
DEFAULT_BELT_SPEED = 0.2394   # m/s, the world file's speed (3.42 m * 0.07 rad/s)

# Arm movements (motion_type of vision_pick_place, see MOTION_TYPES there): value, text
MOTION_CHOICES = [
    ("auto", "Auto - by object height, all types in turn"),
    ("cycle", "Cycle - each type in order"),
    ("random", "Random - a different type each time"),
    ("standard", "Standard - swing left, back the same way"),
    ("mirror", "Mirror - swing right, back the same way"),
    ("high_carry", "High carry - lift high, slow swing"),
    ("low_fast", "Low fast - small lift, quick swing"),
    ("full_turn", "Full turn - keep turning left, one full turn"),
    ("reverse_return", "Reverse return - go right, come back left"),
]

# Dark factory theme. Buttons take a role with setProperty("role", ...): primary (amber),
# go (green), stop (red); panels are QFrame#card, headings QLabel#h2.
STYLE = """
QWidget { background: #1b2029; color: #e6e9ef; font-size: 10pt; }
QFrame#card, QGroupBox { background: #242b37; border: 1px solid #323b4b; border-radius: 10px; }
QGroupBox { margin-top: 14px; padding: 10px 8px 8px 8px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #f5a623; }
QFrame#card QLabel, QGroupBox QLabel, QCheckBox { background: transparent; }
QLabel#title { font-size: 17pt; font-weight: bold; color: #ffffff; background: transparent; }
QLabel#subtitle { color: #9aa4b6; background: transparent; }
QLabel#h2 { font-size: 11pt; font-weight: bold; color: #f5a623; }
QLabel#muted { color: #9aa4b6; }
QLabel#preview { background: #ffffff; border: 2px solid #3a4456; border-radius: 10px; }
QLabel#chip { border-radius: 11px; padding: 3px 12px; font-weight: bold; }
QFrame#header { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #2b3446, stop:1 #1b2029);
                border-bottom: 3px solid #f5a623; }
QListWidget { background: #1e2430; border: 1px solid #323b4b; border-radius: 8px; padding: 4px; outline: 0; }
QListWidget::item { border-radius: 8px; padding: 4px; color: #e6e9ef; }
QListWidget::item:hover { background: #2f3850; }
QListWidget::item:selected { background: #f5a623; color: #1b2029; }
QPushButton { background: #333d50; border: 1px solid #435069; border-radius: 7px; padding: 6px 12px; }
QPushButton:hover { background: #3f4b63; }
QPushButton:pressed { background: #2a3242; }
QPushButton:disabled { background: #262c38; color: #5d6678; border-color: #2f3644; }
QPushButton[role="primary"] { background: #f5a623; color: #1b2029; border: none; font-weight: bold;
                              font-size: 11pt; padding: 9px 14px; }
QPushButton[role="primary"]:hover { background: #ffbb45; }
QPushButton[role="primary"]:disabled { background: #5b4a2b; color: #9a8a6a; }
QPushButton[role="go"] { background: #1f7a4d; border-color: #27a065; }
QPushButton[role="go"]:hover { background: #26935c; }
QPushButton[role="stop"] { background: #8e2b2b; border-color: #b83a3a; }
QPushButton[role="stop"]:hover { background: #a83434; }
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background: #1e2430; border: 1px solid #3a4456;
                              border-radius: 6px; padding: 4px 6px; selection-background-color: #f5a623; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus { border-color: #f5a623; }
QComboBox QAbstractItemView { background: #242b37; selection-background-color: #f5a623; selection-color: #1b2029; }
QTabWidget::pane { border: 1px solid #323b4b; border-radius: 8px; top: -1px; background: #1e2430; }
QTabBar::tab { background: #2a3240; color: #9aa4b6; padding: 7px 14px; border-top-left-radius: 8px;
               border-top-right-radius: 8px; margin-right: 3px; }
QTabBar::tab:selected { background: #f5a623; color: #1b2029; font-weight: bold; }
QPlainTextEdit { background: #12161d; color: #b9c2d3; border: 1px solid #323b4b; border-radius: 8px;
                 font-family: monospace; font-size: 9pt; }
QCheckBox::indicator { width: 16px; height: 16px; }
QToolTip { background: #242b37; color: #e6e9ef; border: 1px solid #f5a623; }
QScrollBar:vertical { background: transparent; width: 10px; }
QScrollBar::handle:vertical { background: #3a4456; border-radius: 5px; min-height: 30px; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
"""


def card(*widgets_or_layouts, title: str = "") -> QFrame:
    """A rounded panel holding the given widgets / layouts one under the other."""
    frame = QFrame()
    frame.setObjectName("card")
    box = QVBoxLayout(frame)
    box.setContentsMargins(12, 10, 12, 12)
    if title:
        head = QLabel(title)
        head.setObjectName("h2")
        box.addWidget(head)
    for w in widgets_or_layouts:
        if isinstance(w, QWidget):
            box.addWidget(w)
        else:
            box.addLayout(w)
    return frame


def button(text: str, role: str = "", icon=None) -> QPushButton:
    b = QPushButton(text)
    if role:
        b.setProperty("role", role)
    if icon is not None:
        b.setIcon(b.style().standardIcon(icon))
    b.setCursor(Qt.PointingHandCursor)
    return b


def image_dirs():
    """Where product pictures are looked for, first match wins."""
    dirs = [os.environ.get("FACTORY_PRODUCT_IMAGES", ""), os.path.expanduser(PRODUCT_IMAGE_DIR)]
    try:
        from ament_index_python.packages import get_package_share_directory
        dirs.append(os.path.join(get_package_share_directory("factory_bringup"), "product_images"))
    except Exception:
        pass
    return [d for d in dirs if d]


_pixmaps = {}


def picture(ptype: str) -> QPixmap:
    """The product's picture from the simulation, or a plain colour square when there is none yet."""
    if ptype not in _pixmaps:
        pix = QPixmap()
        for d in image_dirs():
            path = os.path.join(d, ptype + ".png")
            if os.path.isfile(path) and pix.load(path):
                break
        if pix.isNull():
            r, g, b = PRODUCT_TYPES[ptype]["rgb"]
            pix = QPixmap(64, 64)
            pix.fill(QColor(int(r * 255), int(g * 255), int(b * 255)))
        _pixmaps[ptype] = pix
    return _pixmaps[ptype]


_destinations = load_destinations()      # {type: "recycling" | "trash"}; others go to bent 2
DEST_TEXT = {"bent2": "Bent 2 (place on the belt)", "recycling": "Recycling bin", "trash": "Trash bin"}
# single circular / straight world ($FACTORY_WORLD_NAME): no bent 2, products not for a bin stay on the belt
HAS_BENT2 = world_has_bent2()
if not HAS_BENT2:
    DEST_TEXT["bent2"] = "Stays on the belt (not picked)"
BADGE = {"recycling": ("R", "#1f9d55"), "trash": ("T", "#4a4f5a")}


def swatch(ptype: str) -> QIcon:
    """The product's picture, with a badge when it goes to a bin (R = recycling, T = trash)."""
    dest = _destinations.get(ptype)
    if dest not in BADGE:
        return QIcon(picture(ptype))
    pix = picture(ptype).scaled(96, 96, Qt.KeepAspectRatio, Qt.SmoothTransformation)
    letter, colour = BADGE[dest]
    p = QPainter(pix)
    p.setRenderHint(QPainter.Antialiasing)
    p.setPen(QColor("#ffffff"))
    p.setBrush(QColor(colour))
    d = 34
    p.drawEllipse(pix.width() - d - 2, 2, d, d)
    f = QFont()
    f.setBold(True)
    f.setPixelSize(20)
    p.setFont(f)
    p.drawText(pix.width() - d - 2, 2, d, d, Qt.AlignCenter, letter)
    p.end()
    return QIcon(pix)


def describe(ptype: str) -> str:
    t = PRODUCT_TYPES[ptype]
    sx, sy, sz = t["size"]
    size = f"d {sx * 100:.0f} x h {sz * 100:.0f} cm" if t["shape"] == "cylinder" else \
        f"{sx * 100:.0f} x {sy * 100:.0f} x {sz * 100:.0f} cm"
    return f"{t['label']}   ({t['shape']}, {size})"


class AddProductDialog(QDialog):
    """"Add your own product": a 3D model (.stl / .obj / .dae) or a simple shape with a photo.
    Saved by custom_products.add_product into the shared custom product folder."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add your own product")
        self.setMinimumWidth(680)
        self.result_type = None
        self.name = QLineEdit()
        self.name.setPlaceholderText("e.g. coffee_mug  (letters, numbers, _)")
        self.label = QLineEdit()
        self.label.setPlaceholderText("name shown in the list, e.g. Coffee mug")
        self.use_mesh = QRadioButton("3D model file (.stl, .obj, .dae)")
        self.use_shape = QRadioButton("Simple shape (box, cylinder, sphere)")
        self.use_mesh.setChecked(True)
        self.mesh = QLineEdit()
        self.mesh.setPlaceholderText("model file; .mtl and textures next to it are taken too")
        browse_mesh = button("Browse ...")
        browse_mesh.clicked.connect(lambda: self.browse(self.mesh, "3D models (*.stl *.STL *.obj *.OBJ *.dae *.DAE)"))
        self.keep_size = QCheckBox("Use the model's own size (model in metres)")
        self.shape = QComboBox()
        self.shape.addItems(list(custom_products.SHAPES))
        self.shape.setToolTip("Collision shape the robot grips; for a 3D model the box or cylinder around it")
        self.size = []
        size_row = QHBoxLayout()
        for axis, value in (("x", 0.20), ("y", 0.12), ("z", 0.20)):
            sp = QDoubleSpinBox()
            sp.setRange(0.01, 1.0)
            sp.setDecimals(3)
            sp.setSingleStep(0.01)
            sp.setValue(value)
            sp.setSuffix(" m")
            sp.setPrefix(axis + "  ")
            sp.valueChanged.connect(self.check)
            self.size.append(sp)
            size_row.addWidget(sp)
        self.photo = QLineEdit()
        self.photo.setPlaceholderText("optional photo of the real product (.png / .jpg)")
        browse_photo = button("Browse ...")
        browse_photo.clicked.connect(lambda: self.browse(self.photo, "Pictures (*.png *.jpg *.jpeg *.bmp)"))
        self.auto_color = QCheckBox("Colour from the photo / model")
        self.auto_color.setChecked(True)
        self.color = (0.45, 0.45, 0.48)
        self.color_btn = button("Choose colour ...")
        self.color_btn.clicked.connect(self.pick_color)
        self.recyclable = QCheckBox("Recyclable (cardboard, paper, plastic, glass, metal)")
        self.author = QLineEdit(os.environ.get("USER", ""))
        self.problems = QLabel()
        self.problems.setWordWrap(True)
        mesh_row = QHBoxLayout()
        mesh_row.addWidget(self.mesh, 1)
        mesh_row.addWidget(browse_mesh)
        photo_row = QHBoxLayout()
        photo_row.addWidget(self.photo, 1)
        photo_row.addWidget(browse_photo)
        color_row = QHBoxLayout()
        color_row.addWidget(self.auto_color)
        color_row.addWidget(self.color_btn)
        kind = QHBoxLayout()
        kind.addWidget(self.use_mesh)
        kind.addWidget(self.use_shape)
        form = QFormLayout()
        form.addRow("Name", self.name)
        form.addRow("Shown as", self.label)
        form.addRow("Product is", kind)
        form.addRow("3D model", mesh_row)
        form.addRow("", self.keep_size)
        form.addRow("Shape", self.shape)
        form.addRow("Size", size_row)
        form.addRow("Photo", photo_row)
        form.addRow("Colour", color_row)
        form.addRow("", self.recyclable)
        form.addRow("Added by", self.author)
        hint = QLabel("The size is what the robot grips (a 3D model is scaled to fit inside it). At most "
                      f"{custom_products.MAX_SIZE[0] * 100:.0f} x {custom_products.MAX_SIZE[1] * 100:.0f} x "
                      f"{custom_products.MAX_SIZE[2] * 100:.0f} cm, at least {custom_products.MIN_HEIGHT * 100:.0f} cm "
                      "high. The colour is what the overhead camera sees on top; it is adjusted if the "
                      "detectors could not see it.")
        hint.setObjectName("muted")
        hint.setWordWrap(True)
        buttons = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.save)
        buttons.rejected.connect(self.reject)
        box = QVBoxLayout(self)
        box.addLayout(form)
        box.addWidget(hint)
        box.addWidget(self.problems)
        box.addWidget(buttons)
        for w in (self.use_mesh, self.use_shape, self.keep_size, self.auto_color):
            w.toggled.connect(self.check)
        self.shape.currentIndexChanged.connect(self.check)
        self.name.textChanged.connect(self.check)
        self.check()

    def browse(self, field: QLineEdit, filt: str):
        path, _ = QFileDialog.getOpenFileName(self, "Choose a file", os.path.expanduser("~"), filt)
        if path:
            field.setText(path)
            if field is self.mesh and not self.name.text():
                self.name.setText(custom_products.slug(os.path.splitext(os.path.basename(path))[0]))
            self.check()

    def pick_color(self):
        c = QColorDialog.getColor(QColor(*[int(v * 255) for v in self.color]), self, "Colour seen from above")
        if c.isValid():
            self.color = (c.redF(), c.greenF(), c.blueF())
            self.auto_color.setChecked(False)
            self.check()

    def values(self):
        return [sp.value() for sp in self.size]

    def check(self, *_):
        mesh = self.use_mesh.isChecked()
        for w in (self.mesh, self.keep_size):
            w.setEnabled(mesh)
        for sp in self.size:
            sp.setEnabled(not (mesh and self.keep_size.isChecked()))
        self.size[1].setEnabled(self.size[1].isEnabled() and self.shape.currentText() == "box")
        self.size[2].setEnabled(self.size[2].isEnabled() and self.shape.currentText() != "sphere")
        self.color_btn.setEnabled(True)
        r, g, b = self.color
        self.color_btn.setStyleSheet("" if self.auto_color.isChecked() else
                                     f"background: rgb({int(r * 255)},{int(g * 255)},{int(b * 255)});")
        notes = []
        if not mesh:
            notes += custom_products.validate_size(self.shape.currentText(), tuple(self.values()))
        else:
            notes += ([] if self.mesh.text() or not self.name.text() else ["choose the 3D model file"])
        if not self.auto_color.isChecked():
            notes += custom_products.safe_color(self.color)[1]
        self.problems.setText("<span style='color:#f5a623'>" + "<br>".join(notes) + "</span>" if notes else "")

    def save(self):
        mesh = self.use_mesh.isChecked()
        try:
            ptype, entry, notes = custom_products.add_product(
                self.name.text(), self.label.text(), self.shape.currentText(),
                None if (mesh and self.keep_size.isChecked()) else self.values(),
                None if self.auto_color.isChecked() else self.color,
                self.mesh.text().strip() if mesh else "", self.photo.text().strip(), self.recyclable.isChecked(),
                mesh and self.keep_size.isChecked(), self.author.text().strip())
        except (custom_products.ProductError, OSError, ValueError) as exc:
            QMessageBox.warning(self, "Cannot add the product", str(exc))
            return
        self.result_type, self.notes = ptype, notes
        self.accept()


class OrderNode(Node):
    def __init__(self):
        super().__init__("product_order_gui")
        self.order_pub = self.create_publisher(String, "/spawn_order", 10)
        self.settings_pub = self.create_publisher(String, "/factory_settings", 10)
        self.results = []
        self.create_subscription(String, "/spawn_order/result", lambda m: self.results.append(m.data), 10)
        self.trigger_clients = {name: self.create_client(Trigger, name)
                                for name in ("/stop_bent1", "/start_bent1", "/stop_bent2", "/start_bent2")}
        self.vision_params = self.create_client(SetParameters, "/vision_pick_place/set_parameters")
        self.pending = []    # (service name, future)

    def set_motion(self, motion: str) -> bool:
        """Set motion_type of vision_pick_place; it is used from the next pick on."""
        if not self.vision_params.service_is_ready():
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter(name="motion_type", value=ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=motion))]
        self.pending.append(("arm movement " + motion, self.vision_params.call_async(req)))
        return True

    def call(self, name: str) -> bool:
        client = self.trigger_clients[name]
        if not client.service_is_ready():
            return False
        self.pending.append((name, client.call_async(Trigger.Request())))
        return True

    def finished_calls(self):
        done = [(n, f) for n, f in self.pending if f.done()]
        self.pending = [(n, f) for n, f in self.pending if not f.done()]
        return done


class OrderWindow(QWidget):
    def __init__(self, node: OrderNode, select_output: str = ""):
        super().__init__()
        self.node = node
        self.select_output = select_output      # selection mode: write the order here and close
        self.started = False
        self.setWindowTitle("Choose the products, then start the simulation" if select_output
                            else "Product Order - bent 1")
        self.resize(1280, 960)
        self.setStyleSheet(STYLE)

        # header: title, what this window does, status light
        header = QFrame()
        header.setObjectName("header")
        title = QLabel("Factory Pick & Place")
        title.setObjectName("title")
        subtitle = QLabel(("Choose the products for both bents, then start the simulation" if HAS_BENT2 else
                           "Choose the products for the belt, then start the simulation") if select_output else
                          "Order products onto the bents and control the conveyors while the simulation runs")
        subtitle.setObjectName("subtitle")
        titles = QVBoxLayout()
        titles.setSpacing(0)
        titles.addWidget(title)
        titles.addWidget(subtitle)
        self.chip = QLabel()
        self.chip.setObjectName("chip")
        head_row = QHBoxLayout(header)
        head_row.setContentsMargins(18, 12, 18, 12)
        head_row.addLayout(titles, 1)
        head_row.addWidget(self.chip, 0, Qt.AlignVCenter)
        self.set_status("setup" if select_output else "waiting")
        self.last_status_check = 0.0

        # product types: picture grid + preview of the selected one
        self.types = QListWidget()
        self.types.setViewMode(QListView.IconMode)
        self.types.setIconSize(QSize(92, 92))
        self.types.setGridSize(QSize(122, 138))
        self.types.setSpacing(6)
        self.types.setUniformItemSizes(True)
        self.types.setMinimumHeight(250)
        self.types.setResizeMode(QListView.Adjust)
        self.types.setMovement(QListView.Static)
        self.types.setWordWrap(True)
        for ptype in PRODUCT_TYPES:
            item = QListWidgetItem(swatch(ptype), PRODUCT_TYPES[ptype]["label"])
            item.setData(Qt.UserRole, ptype)
            item.setToolTip(describe(ptype))
            self.types.addItem(item)
        self.types.itemDoubleClicked.connect(lambda _: self.add_selected())
        self.types.currentItemChanged.connect(lambda *_: self.show_preview())
        self.search = QLineEdit()
        self.search.setPlaceholderText("Search products ...")
        self.search.setClearButtonEnabled(True)
        self.search.textChanged.connect(self.filter_types)
        self.shape_filter = QComboBox()
        self.shape_filter.addItems(["All shapes", "box", "cylinder", "sphere", "Own products"])
        self.shape_filter.currentIndexChanged.connect(self.filter_types)
        self.shown_types = QLabel()
        self.shown_types.setObjectName("muted")
        self.preview = QLabel()
        self.preview.setObjectName("preview")
        self.preview.setFixedSize(150, 150)
        self.preview.setAlignment(Qt.AlignCenter)
        self.preview_text = QLabel("Select a product")
        self.preview_text.setWordWrap(True)
        self.preview_text.setTextFormat(Qt.RichText)
        reload_pics = button("Reload pictures", icon=QStyle.SP_BrowserReload)
        reload_pics.clicked.connect(self.reload_pictures)
        self.capture = button("Take pictures in Gazebo", icon=QStyle.SP_DialogSaveButton)
        self.capture.setToolTip("Runs capture_product_images: each product is shown to the photo camera "
                                "in the simulation and saved as its picture (Gazebo must be running).")
        self.capture.clicked.connect(self.capture_pictures)
        self.capture_proc = None
        info = QVBoxLayout()
        info.addWidget(self.preview_text)
        info.addStretch(1)
        info.addWidget(reload_pics)
        info.addWidget(self.capture)
        preview_row = QHBoxLayout()
        preview_row.addWidget(self.preview)
        preview_row.addLayout(info, 1)
        # where the arm takes this product type
        self.dest = QComboBox()
        for value in ("bent2", "recycling", "trash"):
            self.dest.addItem(DEST_TEXT[value], value)
        self.dest.setToolTip("Where the arm takes every product of this type (from the next pick on)")
        self.dest.currentIndexChanged.connect(self.on_dest_changed)
        sort_bins = button("Auto-sort into bins")
        sort_bins.setToolTip("Cardboard, paper, plastic, glass and metal products to the recycling bin, "
                             "the others to the trash bin")
        sort_bins.clicked.connect(self.sort_into_bins)
        all_bent2 = button("All to bent 2" if HAS_BENT2 else "None to the bins")
        all_bent2.clicked.connect(lambda: self.set_all_destinations({}))
        dest_row = QHBoxLayout()
        dest_row.addWidget(QLabel("Goes to"))
        dest_row.addWidget(self.dest, 1)
        dest_row.addWidget(sort_bins)
        dest_row.addWidget(all_bent2)
        self.count = QSpinBox()
        self.count.setRange(1, 20)
        self.count.setPrefix("x ")
        self.count.setFixedWidth(70)
        add = button("Add to order", "go", QStyle.SP_ArrowRight)
        add.clicked.connect(self.add_selected)
        add_all = button("Add one of each")
        add_all.clicked.connect(self.add_one_of_each)
        filters = QHBoxLayout()
        filters.addWidget(self.search, 1)
        filters.addWidget(self.shape_filter)
        row = QHBoxLayout()
        row.addWidget(self.count)
        row.addWidget(add, 1)
        row.addWidget(add_all)
        catalog = card(filters, self.shown_types, self.types,
                       title=f"Products  ({len(PRODUCT_TYPES)} types, double-click to add)")
        catalog.layout().setStretchFactor(self.types, 1)
        # your own products: add (3D model or shape + photo), share as .zip, remove
        own = QHBoxLayout()
        for text, slot, icon, tip in (
                ("Add your own product ...", self.add_custom, QStyle.SP_FileDialogNewFolder,
                 "Add a 3D model (.stl / .obj / .dae) or a shape with a photo; it can then be ordered, "
                 "sorted into the bins and detected like the other products"),
                ("Import .zip ...", self.import_custom, QStyle.SP_DialogOpenButton, "Add a product someone exported"),
                ("Export ...", self.export_custom, QStyle.SP_DialogSaveButton, "Save the selected own product as a .zip to share"),
                ("Remove", self.remove_custom, QStyle.SP_TrashIcon, "Delete the selected own product")):
            b = button(text, "go" if text.startswith("Add") else "", icon)
            b.setToolTip(tip)
            b.clicked.connect(slot)
            own.addWidget(b, 2 if text.startswith("Add") else 1)
        details = card(preview_row, dest_row, row, own, title="Selected product")
        left = QVBoxLayout()
        left.addWidget(catalog, 1)
        left.addWidget(details)

        # the orders: one list per bent, spacing / offset / clear kept per bent
        self.orders = {1: QListWidget(), 2: QListWidget()}
        self.bent_opts = {1: {"spacing": 1.0, "start_offset": 1.5, "clear": True},
                          2: {"spacing": 1.5, "start_offset": 1.0, "clear": True}}
        self.shown_bent = 1
        self.tabs = QTabWidget()
        self.tabs.addTab(self.orders[1], "Bent 1 - pick")
        if HAS_BENT2:
            self.tabs.addTab(self.orders[2], "Bent 2 - place")
        self.tabs.setTabToolTip(0, "Bent 1: the robot picks the products here")
        if HAS_BENT2:
            self.tabs.setTabToolTip(1, "Bent 2: the robot places the products here")
        self.tabs.currentChanged.connect(self.on_tab_changed)
        for lst in self.orders.values():
            lst.setMinimumHeight(160)
            lst.setDragDropMode(QAbstractItemView.InternalMove)      # sort by drag and drop too
            lst.setDefaultDropAction(Qt.MoveAction)
            lst.model().rowsMoved.connect(lambda *_: self.number_orders())
        buttons = QHBoxLayout()
        for text, slot, icon, role in (("Up", lambda: self.move(-1), QStyle.SP_ArrowUp, ""),
                                       ("Down", lambda: self.move(1), QStyle.SP_ArrowDown, ""),
                                       ("Remove", self.remove_selected, QStyle.SP_DialogCancelButton, ""),
                                       ("Clear", self.clear_order, QStyle.SP_TrashIcon, "stop")):
            b = button(text, role, icon)
            b.clicked.connect(slot)
            buttons.addWidget(b)
        self.spacing = QDoubleSpinBox()
        self.spacing.setRange(0.5, 5.0)
        self.spacing.setSingleStep(0.1)
        self.spacing.setValue(1.0)
        self.spacing.setSuffix(" m")
        self.offset = QDoubleSpinBox()
        self.offset.setRange(0.5, 10.0)
        self.offset.setSingleStep(0.1)
        self.offset.setValue(1.5)
        self.offset.setSuffix(" m")
        self.clear_first = QCheckBox("Remove the products on bent 1 first")
        self.clear_first.setChecked(True)
        self.capacity = QLabel()
        self.capacity.setWordWrap(True)
        form = QFormLayout()
        form.addRow("Distance between products", self.spacing)
        self.offset_label = QLabel("First product before the station")
        form.addRow(self.offset_label, self.offset)
        form.addRow(self.clear_first)
        form.addRow(self.capacity)
        self.spawn = button("Spawn order on bent 1", "primary", QStyle.SP_MediaPlay)
        self.spawn.clicked.connect(self.spawn_order)
        hint = QLabel("Top of the list arrives first. Drag items or use Up / Down to sort.")
        hint.setObjectName("muted")
        orders_card = card(hint, self.tabs, buttons, form, self.spawn, title="Orders")
        orders_card.layout().setStretchFactor(self.tabs, 1)
        right = QVBoxLayout()
        right.addWidget(orders_card, 1)

        # conveyors
        bents = QGroupBox("Conveyors")
        grid = QHBoxLayout(bents)
        conveyor_buttons = (
                ("Stop bent 1", ["/stop_bent1"], "stop", QStyle.SP_MediaStop),
                ("Start bent 1", ["/start_bent1"], "go", QStyle.SP_MediaPlay),
                ("Stop bent 2", ["/stop_bent2"], "stop", QStyle.SP_MediaStop),
                ("Start bent 2", ["/start_bent2"], "go", QStyle.SP_MediaPlay),
                ("Stop both", ["/stop_bent1", "/stop_bent2"], "stop", QStyle.SP_BrowserStop),
                ("Start both", ["/start_bent1", "/start_bent2"], "go", QStyle.SP_MediaSeekForward))
        if not HAS_BENT2:
            conveyor_buttons = (("Stop belt", ["/stop_bent1"], "stop", QStyle.SP_MediaStop),
                                ("Start belt", ["/start_bent1"], "go", QStyle.SP_MediaPlay))
        for text, services, role, icon in conveyor_buttons:
            b = button(text, role, icon)
            b.clicked.connect(lambda _, ss=services: [self.call(s) for s in ss])
            grid.addWidget(b)

        sim = QGroupBox("Simulation")
        sim_form = QFormLayout(sim)
        self.belt_speed = QDoubleSpinBox()
        self.belt_speed.setRange(0.05, 1.0)
        self.belt_speed.setSingleStep(0.01)
        self.belt_speed.setDecimals(4)
        self.belt_speed.setValue(DEFAULT_BELT_SPEED)
        self.belt_speed.setSuffix(" m/s")
        self.sim_speed = QDoubleSpinBox()
        self.sim_speed.setRange(0.1, 5.0)
        self.sim_speed.setSingleStep(0.1)
        self.sim_speed.setValue(1.0)
        self.sim_speed.setSuffix(" x real time")
        self.run_minutes = QSpinBox()
        self.run_minutes.setRange(0, 600)
        self.run_minutes.setSuffix(" min")
        self.run_minutes.setSpecialValueText("no limit")
        self.motion = QComboBox()
        for value, text in MOTION_CHOICES:
            self.motion.addItem(text, value)
        self.motion.setToolTip("How the arm moves from bent 1 to bent 2 and back. Auto uses all six "
                               "movements (tall and flat objects get their own one).")
        self.apply_settings_button = button("Apply settings", "go", QStyle.SP_DialogApplyButton)
        self.apply_settings_button.clicked.connect(self.send_settings)
        sim_form.addRow("Belt speed, both bents" if HAS_BENT2 else "Belt speed", self.belt_speed)
        sim_form.addRow("Simulation speed", self.sim_speed)
        sim_form.addRow("Run time (then the arm and bents stop)", self.run_minutes)
        sim_form.addRow("Arm movement", self.motion)
        sim_form.addRow(self.apply_settings_button)
        right.addWidget(sim)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(200)
        self.log.setFixedHeight(72)

        top = QHBoxLayout()
        top.setSpacing(12)
        top.addLayout(left, 11)
        top.addLayout(right, 10)
        body = QVBoxLayout()
        body.setContentsMargins(14, 10, 14, 14)
        body.setSpacing(10)
        body.addLayout(top, 1)
        body.addWidget(bents)
        body.addWidget(card(self.log, title="Messages"))
        main = QVBoxLayout(self)
        main.setContentsMargins(0, 0, 0, 0)
        main.setSpacing(0)
        main.addWidget(header)
        main.addLayout(body, 1)

        if select_output:
            bents.hide()
            self.capture.hide()                 # needs Gazebo, which is not running yet
            self.apply_settings_button.hide()   # the settings go into the start file
            self.spawn.setText("Start simulation with these orders")
            world = button("Start simulation with the world's own products", icon=QStyle.SP_DirHomeIcon)
            world.clicked.connect(self.start_with_world_products)
            right.addWidget(world)
            self.load_selection(select_output)

        self.spacing.valueChanged.connect(self.update_capacity)
        self.offset.valueChanged.connect(self.update_capacity)
        self.update_capacity()
        for lst in self.orders.values():
            lst.setIconSize(QSize(36, 36))
        self.filter_types()
        self.types.setCurrentRow(0)

        if self.node is not None:
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.spin_ros)
            self.timer.start(50)

    # selection mode ---------------------------------------------------------------
    def load_selection(self, path: str):
        try:
            with open(os.path.expanduser(path)) as f:
                order = json.load(f)
        except (OSError, ValueError):
            self.say("Choose the products for bent 1, then press Start simulation.")
            return
        per_bent = {1: order.get("bent1", {}), 2: order.get("bent2", {})} if ("bent1" in order or "bent2" in order) \
            else {1: order, 2: {}}                       # older files: one bent-1 order
        for bent, o in per_bent.items():
            for ptype in o.get("items", []):
                if ptype in PRODUCT_TYPES:
                    item = QListWidgetItem(swatch(ptype), PRODUCT_TYPES[ptype]["label"])
                    item.setData(Qt.UserRole, ptype)
                    self.orders[bent].addItem(item)
            opts = self.bent_opts[bent]
            opts["spacing"] = float(o.get("spacing", opts["spacing"]))
            opts["start_offset"] = float(o.get("start_offset", opts["start_offset"]))
            opts["clear"] = bool(o.get("clear", opts["clear"]))
        self.number_orders()
        s = order.get("settings", {})
        self.belt_speed.setValue(float(s.get("belt_speed", self.belt_speed.value())))
        self.sim_speed.setValue(float(s.get("real_time_factor", self.sim_speed.value())))
        self.run_minutes.setValue(int(s.get("run_minutes", self.run_minutes.value())))
        i = self.motion.findData(s.get("motion_type", "auto"))
        self.motion.setCurrentIndex(max(0, i))
        self.show_bent_opts()
        self.say(f"Last selection loaded from {path}. Change it or press Start simulation.")

    def write_selection(self, order):
        path = os.path.expanduser(self.select_output)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w") as f:
            json.dump(order, f, indent=1)
        self.started = True
        self.close()

    def start_with_world_products(self):
        self.write_selection({"bent1": {"items": []}, "bent2": {"items": []}, "settings": self.settings()})

    # bents and settings ----------------------------------------------------------
    @property
    def order(self) -> QListWidget:
        """The order list of the bent whose tab is shown."""
        return self.orders[self.shown_bent]

    def keep_bent_opts(self):
        self.bent_opts[self.shown_bent] = {"spacing": self.spacing.value(), "start_offset": self.offset.value(),
                                           "clear": self.clear_first.isChecked()}

    def show_bent_opts(self):
        opts = self.bent_opts[self.shown_bent]
        self.spacing.setValue(opts["spacing"])
        self.offset.setValue(opts["start_offset"])
        self.clear_first.setChecked(opts["clear"])
        bent = self.shown_bent
        self.clear_first.setText(f"Remove the objects on bent {bent} first")
        self.offset_label.setText("First product before the station" if bent == 1 else
                                  "First product before the release point")
        if not self.select_output:
            self.spawn.setText(f"Spawn order on bent {bent}")
        self.update_capacity()

    def on_tab_changed(self, index: int):
        self.keep_bent_opts()
        self.shown_bent = index + 1
        self.show_bent_opts()

    def order_for(self, bent: int):
        if bent == self.shown_bent:
            self.keep_bent_opts()
        lst = self.orders[bent]
        return dict(self.bent_opts[bent], bent=bent,
                    items=[lst.item(i).data(Qt.UserRole) for i in range(lst.count())])

    def settings(self):
        return {"belt_speed": round(self.belt_speed.value(), 4), "real_time_factor": round(self.sim_speed.value(), 3),
                "run_minutes": self.run_minutes.value(), "motion_type": self.motion.currentData()}

    def send_settings(self):
        motion = self.motion.currentData()
        if not self.node.set_motion(motion):
            self.say("vision_pick_place is not running: arm movement not changed.")
        if self.node.count_subscribers("/factory_settings") == 0:
            self.say("Nobody listens on /factory_settings: is the attacher running (restarted after the update)?")
            return
        self.node.settings_pub.publish(String(data=json.dumps(self.settings())))
        self.say("Sent settings: " + json.dumps(self.settings()))

    # pictures -------------------------------------------------------------------
    def show_preview(self):
        item = self.types.currentItem()
        if item is None:
            return
        ptype = item.data(Qt.UserRole)
        self.preview.setPixmap(picture(ptype).scaled(146, 146, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        t = PRODUCT_TYPES[ptype]
        sx, sy, sz = t["size"]
        size = {"box": f"{sx * 100:.0f} x {sy * 100:.0f} x {sz * 100:.0f} cm",
                "cylinder": f"diameter {sx * 100:.0f} cm, height {sz * 100:.0f} cm",
                "sphere": f"diameter {sx * 100:.0f} cm"}[t["shape"]]
        self.preview_text.setText(f"<span style='font-size:13pt; font-weight:bold; color:#ffffff'>{t['label']}</span>"
                                  f"<br><span style='color:#9aa4b6'>type</span> {ptype}"
                                  f"<br><span style='color:#9aa4b6'>shape</span> {t['shape']}"
                                  f"<br><span style='color:#9aa4b6'>size</span> {size}"
                                  f"<br><span style='color:#9aa4b6'>goes to</span> "
                                  f"{DEST_TEXT[_destinations.get(ptype, 'bent2')]}")
        self._showing = True                    # setting the box is not a change by the user
        self.dest.setCurrentIndex(max(0, self.dest.findData(_destinations.get(ptype, "bent2"))))
        self._showing = False

    def filter_types(self, *_):
        """Show only the products whose name matches the search text and the chosen shape."""
        text = self.search.text().strip().lower()
        shape = self.shape_filter.currentText()
        shown = 0
        for i in range(self.types.count()):
            item = self.types.item(i)
            ptype = item.data(Qt.UserRole)
            t = PRODUCT_TYPES[ptype]
            ok = (not text or text in ptype.lower() or text in t["label"].lower()) and \
                 (shape == "All shapes" or t["shape"] == shape or (shape == "Own products" and "custom" in t))
            item.setHidden(not ok)
            shown += ok
        self.shown_types.setText(f"{shown} of {len(PRODUCT_TYPES)} shown")

    # bins -----------------------------------------------------------------------
    def on_dest_changed(self, *_):
        item = self.types.currentItem()
        if item is None or getattr(self, "_showing", False):
            return
        ptype, dest = item.data(Qt.UserRole), self.dest.currentData()
        new = dict(_destinations)
        if dest == "bent2":
            new.pop(ptype, None)
        else:
            new[ptype] = dest
        self.set_all_destinations(new, f"{PRODUCT_TYPES[ptype]['label']} now goes to: {DEST_TEXT[dest]}")

    def sort_into_bins(self):
        self.set_all_destinations({t: ("recycling" if t in RECYCLABLE else "trash") for t in PRODUCT_TYPES},
                                  f"{len(RECYCLABLE)} recyclable types to the recycling bin, "
                                  f"{len(PRODUCT_TYPES) - len(RECYCLABLE)} to the trash bin")

    def set_all_destinations(self, new, text: str = "every product goes to bent 2"):
        _destinations.clear()
        _destinations.update(new)
        try:
            save_destinations(_destinations)
        except OSError as exc:
            self.say(f"Could not save the bin choices: {exc}")
            return
        self.refresh_icons()
        self.show_preview()
        self.say(f"Bins: {text} (used from the next pick).")

    def refresh_icons(self):
        for i in range(self.types.count()):
            item = self.types.item(i)
            item.setIcon(swatch(item.data(Qt.UserRole)))
        for lst in self.orders.values():
            for i in range(lst.count()):
                item = lst.item(i)
                item.setIcon(swatch(item.data(Qt.UserRole)))

    def reload_pictures(self):
        _pixmaps.clear()
        for i in range(self.types.count()):
            item = self.types.item(i)
            item.setIcon(swatch(item.data(Qt.UserRole)))
        for lst in self.orders.values():
            for i in range(lst.count()):
                item = lst.item(i)
                item.setIcon(swatch(item.data(Qt.UserRole)))
        self.show_preview()
        self.say("Pictures reloaded from " + " | ".join(image_dirs()))

    def capture_pictures(self):
        if self.capture_proc is not None:
            return
        self.capture_proc = QProcess(self)
        self.capture_proc.setProcessChannelMode(QProcess.MergedChannels)
        self.capture_proc.readyReadStandardOutput.connect(
            lambda: [self.say(line) for line in bytes(self.capture_proc.readAllStandardOutput()).decode(
                errors="replace").splitlines() if line.strip()])
        self.capture_proc.finished.connect(self.capture_finished)
        self.capture.setEnabled(False)
        self.say("Taking pictures in Gazebo (about 2 s per product) ...")
        self.capture_proc.start("ros2", ["run", "factory_bringup", "capture_product_images"])

    def capture_finished(self, *_):
        self.capture_proc = None
        self.capture.setEnabled(True)
        self.reload_pictures()

    # your own products ----------------------------------------------------------
    def reload_types(self, select: str = ""):
        """Rebuild the product grid (after own products were added or removed)."""
        refresh_custom_products(force=True)
        self.types.clear()
        for ptype in PRODUCT_TYPES:
            _pixmaps.pop(ptype, None)
            item = QListWidgetItem(swatch(ptype), PRODUCT_TYPES[ptype]["label"])
            item.setData(Qt.UserRole, ptype)
            item.setToolTip(describe(ptype) + ("   [own product]" if "custom" in PRODUCT_TYPES[ptype] else ""))
            self.types.addItem(item)
            if ptype == select:
                self.types.setCurrentItem(item)
        for lst in self.orders.values():                 # drop order items whose type is gone
            for i in reversed(range(lst.count())):
                if lst.item(i).data(Qt.UserRole) not in PRODUCT_TYPES:
                    lst.takeItem(i)
        self.number_orders()
        self.filter_types()

    def selected_custom(self):
        item = self.types.currentItem()
        ptype = item.data(Qt.UserRole) if item else None
        if not ptype or "custom" not in PRODUCT_TYPES.get(ptype, {}):
            self.say("Select one of your own products first (filter: Own products).")
            return None
        return ptype

    def add_custom(self):
        dlg = AddProductDialog(self)
        dlg.setStyleSheet(STYLE)
        if dlg.exec_() != QDialog.Accepted or not dlg.result_type:
            return
        self.reload_types(dlg.result_type)
        self.say(f"Added your product '{dlg.result_type}'" + (": " + "; ".join(dlg.notes) if dlg.notes else "") +
                 ". It can be ordered now; restart detectors / vision labs to add it to their classes.")

    def import_custom(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import a product", os.path.expanduser("~"), "Product (*.zip)")
        if not path:
            return
        try:
            ptype = custom_products.import_product(path)
        except (custom_products.ProductError, OSError, ValueError, KeyError) as exc:
            QMessageBox.warning(self, "Cannot import", str(exc))
            return
        self.reload_types(ptype)
        self.say(f"Imported product '{ptype}' from {path}.")

    def export_custom(self):
        ptype = self.selected_custom()
        if not ptype:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export product", os.path.expanduser(f"~/{ptype}.zip"), "Product (*.zip)")
        if path:
            try:
                self.say("Exported to " + custom_products.export_product(ptype, path))
            except (custom_products.ProductError, OSError) as exc:
                QMessageBox.warning(self, "Cannot export", str(exc))

    def remove_custom(self):
        ptype = self.selected_custom()
        if not ptype:
            return
        if QMessageBox.question(self, "Remove product", f"Delete your product '{ptype}' and its files?") != QMessageBox.Yes:
            return
        custom_products.remove_product(ptype)
        if ptype in _destinations:
            new = dict(_destinations)
            new.pop(ptype)
            self.set_all_destinations(new, f"{ptype} removed")
        self.reload_types()
        self.say(f"Removed '{ptype}'.")

    # order list ---------------------------------------------------------------
    def add_type(self, ptype: str, n: int = 1):
        for _ in range(n):
            item = QListWidgetItem(swatch(ptype), PRODUCT_TYPES[ptype]["label"])
            item.setData(Qt.UserRole, ptype)
            self.order.addItem(item)
        self.number_orders()
        self.update_capacity()

    def add_selected(self):
        item = self.types.currentItem()
        if item is not None:
            self.add_type(item.data(Qt.UserRole), self.count.value())

    def add_one_of_each(self):
        for ptype in PRODUCT_TYPES:
            self.add_type(ptype)

    def move(self, step: int):
        row = self.order.currentRow()
        if row < 0 or not 0 <= row + step < self.order.count():
            return
        item = self.order.takeItem(row)
        self.order.insertItem(row + step, item)
        self.order.setCurrentRow(row + step)
        self.number_orders()

    def remove_selected(self):
        row = self.order.currentRow()
        if row >= 0:
            self.order.takeItem(row)
        self.number_orders()
        self.update_capacity()

    def clear_order(self):
        self.order.clear()
        self.update_capacity()

    def number_orders(self):
        """Show the position in front of every order item (1 = arrives first)."""
        for lst in self.orders.values():
            for i in range(lst.count()):
                item = lst.item(i)
                item.setText(f"{i + 1:>2}.  {PRODUCT_TYPES[item.data(Qt.UserRole)]['label']}")

    def items(self):
        return [self.order.item(i).data(Qt.UserRole) for i in range(self.order.count())]

    def update_capacity(self):
        cap = (bent1_capacity(self.spacing.value(), self.offset.value()) if self.shown_bent == 1
               else bent2_capacity(self.spacing.value(), self.offset.value()))
        n = self.order.count()
        self.capacity.setText(f"{n} in the order for bent {self.shown_bent}, at most {cap} fit at this distance"
                              + ("  - too many, remove some or shorten the distance" if n > cap else ""))
        self.capacity.setStyleSheet("color: #ff6b6b; font-weight: bold;" if n > cap else "color: #9aa4b6;")
        for b in ((1, 2) if HAS_BENT2 else (1,)):
            self.tabs.setTabText(b - 1, f"Bent {b} - {'pick' if b == 1 else 'place'}  ({self.orders[b].count()})")
        if self.select_output:
            opts = dict(self.bent_opts)
            opts[self.shown_bent] = {"spacing": self.spacing.value(), "start_offset": self.offset.value()}
            fits = all(self.orders[b].count() <= (bent1_capacity(o["spacing"], o["start_offset"]) if b == 1
                                                  else bent2_capacity(o["spacing"], o["start_offset"]))
                       for b, o in opts.items())
            self.spawn.setEnabled(fits and any(lst.count() for lst in self.orders.values()))
        else:
            self.spawn.setEnabled(0 < n <= cap)

    # ROS ------------------------------------------------------------------------
    def say(self, text: str):
        self.log.appendPlainText(time.strftime("%H:%M:%S  ") + text)

    def set_status(self, state: str):
        """Header light: setup (selection before start), online (attacher listening), waiting."""
        text, bg, fg = {"setup": ("SETUP", "#3d8bfd", "#ffffff"),
                        "online": ("ATTACHER ONLINE", "#27a065", "#ffffff"),
                        "waiting": ("WAITING FOR SIMULATION", "#5d6678", "#e6e9ef")}[state]
        self.chip.setText("\u25CF  " + text)
        self.chip.setStyleSheet(f"background: {bg}; color: {fg};")

    def spawn_order(self):
        if self.select_output:
            self.write_selection({"bent1": self.order_for(1), "bent2": self.order_for(2), "settings": self.settings()})
            return
        order = self.order_for(self.shown_bent)
        if self.node.count_subscribers("/spawn_order") == 0:
            self.say("Nobody listens on /spawn_order: is bcr_endpoint_object_attacher running (rebuilt)?")
            return
        self.node.order_pub.publish(String(data=json.dumps(order)))
        self.say(f"Sent order of {len(order['items'])} products for bent {order['bent']}, "
                 f"{order['spacing']:.2f} m apart ...")

    def call(self, service: str):
        if not self.node.call(service):
            self.say(f"{service} is not available (is the attacher running?)")

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)
        if time.monotonic() - self.last_status_check > 1.0:
            self.last_status_check = time.monotonic()
            self.set_status("online" if self.node.count_subscribers("/spawn_order") > 0 else "waiting")
        for text in self.node.results:
            try:
                res = json.loads(text)
                self.say(("OK: " if res.get("ok") else "FAILED: ") + res.get("message", ""))
            except ValueError:
                self.say(text)
        self.node.results.clear()
        for name, future in self.node.finished_calls():
            res = future.result()
            if res is not None and hasattr(res, "results"):          # SetParameters
                ok = all(r.successful for r in res.results)
                self.say(f"{name}: " + ("set, used from the next pick" if ok else
                                        "refused: " + "; ".join(r.reason for r in res.results)))
            else:
                self.say(f"{name}: {res.message if res else 'no answer'}")


def main(args=None):
    argv = sys.argv[1:]
    if "--select" in argv:
        out = argv[argv.index("--output") + 1] if "--output" in argv else "~/.ros/factory_order.json"
        app = QApplication(sys.argv[:1])
        win = OrderWindow(None, select_output=out)
        win.show()
        app.exec_()
        sys.exit(0 if win.started else 3)

    rclpy.init(args=args)
    node = OrderNode()
    app = QApplication(sys.argv)
    win = OrderWindow(node)
    win.show()
    try:
        code = app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
