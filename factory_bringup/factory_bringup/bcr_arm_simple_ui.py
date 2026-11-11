#!/usr/bin/env python3
"""
BCR Arm Factory GUI - V14 Endpoint Attachment Fixed UI

Fixes over your V13 file:
  - Window/title version corrected to V14.
  - target_model default is pickup_box, not pickup_box_1.
  - attach_offset_frame combo is placed correctly in the grid.
  - attach_offset defaults are correct for endpoint attacher V2:
      frame=world, x=0.0, y=0.0, z=-0.08, keep_object_world_flat=True.
  - Endpoint attach/release buttons log each step clearly.
  - Hybrid attach no longer depends on a physical finger gripper working.
  - Gripper commands are treated as optional because your current URDF only has gripper_tip.
  - Sequence attach/release calls endpoint services directly.

Required:
  Terminal 1:
    ros2 launch factory_bringup factory_with_bcr_arm.launch.py

  Terminal 2:
    ros2 launch factory_bringup bcr_endpoint_object_attacher.launch.py

  Terminal 3:
    ros2 launch factory_bringup bcr_arm_simple_ui.launch.py
"""

import sys
from typing import Callable, Dict, List, Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QPushButton,
    QDoubleSpinBox,
    QTextEdit,
    QListWidget,
    QListWidgetItem,
    QGroupBox,
    QTabWidget,
    QSlider,
    QMessageBox,
    QLineEdit,
    QCheckBox,
    QComboBox,
)


DEFAULT_CONTROLLER_TOPIC = "/joint_trajectory_controller/joint_trajectory"
DEFAULT_GRIPPER_TOPIC = "/gripper_position_controller/commands"
DEFAULT_JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]

ENDPOINT_ATTACH_SERVICE = "/attach_object_to_endpoint"
ENDPOINT_RELEASE_SERVICE = "/release_object_from_endpoint"
ENDPOINT_MOVE_PICK_SERVICE = "/move_object_to_endpoint_pick"
ENDPOINT_PARAM_SERVICE = "/bcr_endpoint_object_attacher/set_parameters"

# Optional only in your current endpoint-only URDF.
# These commands are harmless if no gripper controller exists, but the actual picking is /attach_object_to_endpoint.
GRIPPER_OPEN = [0.04, 0.04]
GRIPPER_HALF = [0.02, 0.02]
GRIPPER_CLOSE = [0.0, 0.0]

PRESETS = {
    "Home / Zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "Safe Observation": [0.0, 0.45, 0.0, -0.65, 0.0, 0.35, 0.0],

    # Your recent useful manual pose range was around:
    # [1.300, -0.750, 0.000, -0.800, 0.000, -1.200, 0.000]
    "Pick Approach": [1.30, -0.50, 0.0, -0.80, 0.0, -1.20, 0.0],
    "Pick Contact": [1.30, -0.75, 0.0, -0.80, 0.0, -1.20, 0.0],
    "Lift": [1.30, -0.50, 0.0, -0.80, 0.0, -1.20, 0.0],
    "Place Approach": [0.65, -0.50, 0.0, -0.80, 0.0, -1.20, 0.0],
    "Place Contact": [0.65, -0.75, 0.0, -0.80, 0.0, -1.20, 0.0],
}


def make_param_msg(name: str, value) -> ParameterMsg:
    msg = ParameterMsg()
    msg.name = name

    pv = ParameterValue()

    if isinstance(value, bool):
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(value)
    elif isinstance(value, int):
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)
    elif isinstance(value, float):
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
    else:
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(value)

    msg.value = pv
    return msg


class BCRArmEndpointNode(Node):
    def __init__(self):
        super().__init__("bcr_arm_endpoint_attachment_gui_node")

        self.current_joint_positions: Dict[str, float] = {}
        self.last_joint_state_time: Optional[float] = None

        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            DEFAULT_CONTROLLER_TOPIC,
            10,
        )

        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            DEFAULT_GRIPPER_TOPIC,
            10,
        )

        self.attach_client = self.create_client(Trigger, ENDPOINT_ATTACH_SERVICE)
        self.release_client = self.create_client(Trigger, ENDPOINT_RELEASE_SERVICE)
        self.move_pick_client = self.create_client(Trigger, ENDPOINT_MOVE_PICK_SERVICE)
        self.param_client = self.create_client(SetParameters, ENDPOINT_PARAM_SERVICE)

    def joint_state_callback(self, msg: JointState):
        self.last_joint_state_time = self.get_clock().now().nanoseconds / 1e9
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = float(pos)

    def current_arm_vector(self) -> List[float]:
        return [
            float(self.current_joint_positions.get(name, 0.0))
            for name in DEFAULT_JOINT_NAMES
        ]

    def publish_arm_goal(self, positions: List[float], duration_sec: float = 2.0):
        if len(positions) != 7:
            raise ValueError(f"Expected 7 joint positions, got {len(positions)}")

        traj = JointTrajectory()
        traj.joint_names = list(DEFAULT_JOINT_NAMES)

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in positions]

        sec = int(duration_sec)
        point.time_from_start.sec = sec
        point.time_from_start.nanosec = int((duration_sec - sec) * 1e9)

        traj.points.append(point)
        self.trajectory_pub.publish(traj)

    def publish_gripper(self, values: List[float]):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        self.gripper_pub.publish(msg)

    def services_ready(self) -> Dict[str, bool]:
        return {
            "attach": self.attach_client.service_is_ready(),
            "release": self.release_client.service_is_ready(),
            "move_pick": self.move_pick_client.service_is_ready(),
            "params": self.param_client.service_is_ready(),
        }

    def _trigger_async(self, client, name: str):
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.05)

        if not client.service_is_ready():
            raise RuntimeError(f"{name} is not available. Launch bcr_endpoint_object_attacher first.")

        return client.call_async(Trigger.Request())

    def attach_async(self):
        return self._trigger_async(self.attach_client, ENDPOINT_ATTACH_SERVICE)

    def release_async(self):
        return self._trigger_async(self.release_client, ENDPOINT_RELEASE_SERVICE)

    def move_pick_async(self):
        return self._trigger_async(self.move_pick_client, ENDPOINT_MOVE_PICK_SERVICE)

    def set_params_async(self, params: Dict[str, object]):
        if not self.param_client.service_is_ready():
            self.param_client.wait_for_service(timeout_sec=0.05)

        if not self.param_client.service_is_ready():
            raise RuntimeError(f"{ENDPOINT_PARAM_SERVICE} is not available. Launch bcr_endpoint_object_attacher first.")

        req = SetParameters.Request()
        req.parameters = [make_param_msg(k, v) for k, v in params.items()]
        return self.param_client.call_async(req)


class BCRArmEndpointGui(QWidget):
    def __init__(self, node: BCRArmEndpointNode):
        super().__init__()

        self.node = node
        self.setWindowTitle("BCR Arm Factory GUI V14 - Endpoint Attachment Fixed")
        self.resize(1280, 860)

        self.joint_spins: List[QDoubleSpinBox] = []
        self.joint_sliders: List[QSlider] = []

        self.saved_waypoints: Dict[str, List[float]] = {
            k: list(v) for k, v in PRESETS.items()
        }

        self.sequence_steps: List[tuple] = []
        self.sequence_index = 0
        self.sequence_running = False

        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.0))
        self.ros_timer.start(30)

        self.status_timer = QTimer(self)
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(300)

        self.sequence_timer = QTimer(self)
        self.sequence_timer.timeout.connect(self.run_next_sequence_step)

        self.build_ui()

    def build_ui(self):
        main = QVBoxLayout()
        self.setLayout(main)

        title = QLabel("BCR Arm Factory Controller V14 - Endpoint Attachment Fixed")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 20px; font-weight: bold; padding: 8px;")
        main.addWidget(title)

        self.status_label = QLabel("Status: starting...")
        main.addWidget(self.status_label)

        tabs = QTabWidget()
        main.addWidget(tabs)

        tabs.addTab(self.build_arm_tab(), "Arm Control")
        tabs.addTab(self.build_endpoint_tab(), "Endpoint Attach")
        tabs.addTab(self.build_gripper_tab(), "Optional Gripper")
        tabs.addTab(self.build_sequence_tab(), "Factory Transfer Sequence")
        tabs.addTab(self.build_log_tab(), "Log")

        self.log("V14 endpoint attachment GUI started.")
        self.log("Launch bcr_endpoint_object_attacher.launch.py before using endpoint attach buttons.")
        self.log("Important: current URDF has endpoint tool frame only. Object pickup is done by /attach_object_to_endpoint.")
        self.log("Default: target_model=pickup_box, attach_offset_frame=world, offset=(0,0,-0.08), keep_flat=True.")

    def build_arm_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        group = QGroupBox("Arm Presets and Waypoints")
        h = QHBoxLayout()
        group.setLayout(h)

        self.preset_list = QListWidget()
        for name in self.saved_waypoints:
            self.preset_list.addItem(QListWidgetItem(name))
        h.addWidget(self.preset_list, 1)

        buttons = QVBoxLayout()
        for text, cb in [
            ("MOVE TO SELECTED PRESET", self.on_move_selected_preset),
            ("LOAD SELECTED INTO SLIDERS", self.on_load_selected_preset),
            ("SAVE CURRENT ARM AS SELECTED PRESET", self.on_save_current_as_selected_preset),
            ("LOAD CURRENT ARM INTO SLIDERS", lambda: self.set_manual_positions(self.node.current_arm_vector())),
        ]:
            b = QPushButton(text)
            b.clicked.connect(cb)
            buttons.addWidget(b)

        home = QPushButton("HOME / ZERO")
        home.clicked.connect(lambda: self.move_arm(PRESETS["Home / Zero"], 2.0, "Home / Zero"))
        buttons.addWidget(home)

        buttons.addStretch()
        h.addLayout(buttons, 1)
        layout.addWidget(group)

        manual = QGroupBox("Manual Joint Control")
        grid = QGridLayout()
        manual.setLayout(grid)

        for i, joint in enumerate(DEFAULT_JOINT_NAMES):
            spin = QDoubleSpinBox()
            spin.setRange(-3.14, 3.14)
            spin.setSingleStep(0.01)
            spin.setDecimals(3)

            slider = QSlider(Qt.Horizontal)
            slider.setRange(-314, 314)

            spin.valueChanged.connect(lambda value, s=slider: s.setValue(int(value * 100)))
            slider.valueChanged.connect(lambda value, sp=spin: sp.setValue(value / 100.0))

            self.joint_spins.append(spin)
            self.joint_sliders.append(slider)

            grid.addWidget(QLabel(joint), i, 0)
            grid.addWidget(slider, i, 1)
            grid.addWidget(spin, i, 2)

        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.2, 15.0)
        self.duration_spin.setSingleStep(0.1)
        self.duration_spin.setValue(2.0)

        grid.addWidget(QLabel("Move duration sec:"), 7, 0)
        grid.addWidget(self.duration_spin, 7, 1)

        send = QPushButton("SEND MANUAL ARM GOAL")
        send.clicked.connect(lambda: self.move_arm(self.get_manual_positions(), self.duration_spin.value(), "Manual"))
        grid.addWidget(send, 7, 2)

        layout.addWidget(manual)
        return tab

    def build_endpoint_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        params = QGroupBox("Endpoint Attacher Parameters")
        grid = QGridLayout()
        params.setLayout(grid)

        self.target_model_edit = QLineEdit("pickup_box")
        self.world_name_edit = QLineEdit("dual_circular_factory_world")
        self.base_frame_edit = QLineEdit("base_link")
        self.endpoint_frame_edit = QLineEdit("gripper_tip")

        self.attach_offset_frame_combo = QComboBox()
        self.attach_offset_frame_combo.addItems(["world", "endpoint"])
        self.attach_offset_frame_combo.setCurrentText("world")

        self.attach_x_spin = self.make_pose_spin(0.0, -2, 2, 0.01)
        self.attach_y_spin = self.make_pose_spin(0.0, -2, 2, 0.01)
        self.attach_z_spin = self.make_pose_spin(0.0, -2, 2, 0.01)

        self.keep_flat_checkbox = QCheckBox("keep_object_world_flat")
        self.keep_flat_checkbox.setChecked(True)

        row = 0
        grid.addWidget(QLabel("target_model:"), row, 0)
        grid.addWidget(self.target_model_edit, row, 1)
        grid.addWidget(QLabel("world_name:"), row, 2)
        grid.addWidget(self.world_name_edit, row, 3)

        row += 1
        grid.addWidget(QLabel("base_frame:"), row, 0)
        grid.addWidget(self.base_frame_edit, row, 1)
        grid.addWidget(QLabel("endpoint_frame:"), row, 2)
        grid.addWidget(self.endpoint_frame_edit, row, 3)

        row += 1
        grid.addWidget(QLabel("attach_offset_frame:"), row, 0)
        grid.addWidget(self.attach_offset_frame_combo, row, 1)

        row += 1
        grid.addWidget(QLabel("attach_offset_x:"), row, 0)
        grid.addWidget(self.attach_x_spin, row, 1)
        grid.addWidget(QLabel("attach_offset_y:"), row, 2)
        grid.addWidget(self.attach_y_spin, row, 3)

        row += 1
        grid.addWidget(QLabel("attach_offset_z:"), row, 0)
        grid.addWidget(self.attach_z_spin, row, 1)
        grid.addWidget(self.keep_flat_checkbox, row, 2, 1, 2)

        row += 1
        apply = QPushButton("APPLY ENDPOINT SETTINGS")
        apply.clicked.connect(self.on_apply_endpoint_settings)
        grid.addWidget(apply, row, 0, 1, 4)

        layout.addWidget(params)

        pick = QGroupBox("Endpoint Pick Test Pose")
        pg = QGridLayout()
        pick.setLayout(pg)

        self.pick_x_spin = self.make_pose_spin(-2.175)
        self.pick_y_spin = self.make_pose_spin(1.195)
        self.pick_z_spin = self.make_pose_spin(2.006)

        for c, (name, widget) in enumerate([
            ("pick_x", self.pick_x_spin),
            ("pick_y", self.pick_y_spin),
            ("pick_z", self.pick_z_spin),
        ]):
            pg.addWidget(QLabel(name + ":"), 0, c * 2)
            pg.addWidget(widget, 0, c * 2 + 1)

        b1 = QPushButton("APPLY PICK POSE")
        b1.clicked.connect(self.on_apply_pick_pose)

        b2 = QPushButton("MOVE OBJECT TO ENDPOINT PICK")
        b2.clicked.connect(self.on_move_object_to_endpoint_pick)

        pg.addWidget(b1, 1, 0, 1, 3)
        pg.addWidget(b2, 1, 3, 1, 3)
        layout.addWidget(pick)

        release = QGroupBox("Release / Place Target")
        rg = QGridLayout()
        release.setLayout(rg)

        self.exact_release_checkbox = QCheckBox("exact_release")
        self.release_x_spin = self.make_pose_spin(-2.175)
        self.release_y_spin = self.make_pose_spin(1.195)
        self.release_z_spin = self.make_pose_spin(1.90)

        rg.addWidget(self.exact_release_checkbox, 0, 0, 1, 2)

        for c, (name, widget) in enumerate([
            ("release_x", self.release_x_spin),
            ("release_y", self.release_y_spin),
            ("release_z", self.release_z_spin),
        ]):
            rg.addWidget(QLabel(name + ":"), 1, c * 2)
            rg.addWidget(widget, 1, c * 2 + 1)

        rb = QPushButton("APPLY RELEASE SETTINGS")
        rb.clicked.connect(self.on_apply_release_settings)
        rg.addWidget(rb, 2, 0, 1, 6)

        layout.addWidget(release)

        actions = QGroupBox("Endpoint Object Actions")
        ag = QGridLayout()
        actions.setLayout(ag)

        action_buttons = [
            ("ATTACH OBJECT TO ENDPOINT", self.on_attach_endpoint),
            ("RELEASE OBJECT FROM ENDPOINT", self.on_release_endpoint),
            ("HYBRID ENDPOINT ATTACH", self.on_hybrid_close_attach),
            ("HYBRID ENDPOINT RELEASE", self.on_hybrid_release_open),
        ]

        for i, (text, cb) in enumerate(action_buttons):
            b = QPushButton(text)
            b.clicked.connect(cb)
            ag.addWidget(b, i // 2, i % 2)

        layout.addWidget(actions)

        note = QLabel(
            "Endpoint mode: the object's top face snaps to endpoint_frame (top of the gripper) "
            "and follows it; release drops it from that exact pose. attach_offset is extra fine-tuning. "
            "Your current URDF has gripper_tip only, so ATTACH OBJECT TO ENDPOINT is the real pick action."
        )
        note.setWordWrap(True)
        note.setStyleSheet("font-weight: bold;")
        layout.addWidget(note)

        layout.addStretch()
        return tab

    def build_gripper_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        group = QGroupBox("Optional Gripper Publisher")
        grid = QGridLayout()
        group.setLayout(grid)

        for i, (text, values) in enumerate([
            ("OPEN GRIPPER OPTIONAL", GRIPPER_OPEN),
            ("HALF CLOSE OPTIONAL", GRIPPER_HALF),
            ("CLOSE GRIPPER OPTIONAL", GRIPPER_CLOSE),
        ]):
            b = QPushButton(text)
            b.clicked.connect(lambda checked=False, v=values, t=text: self.send_gripper(v, t))
            grid.addWidget(b, 0, i)

        self.custom_left_spin = self.make_gripper_spin(0.04)
        self.custom_right_spin = self.make_gripper_spin(0.04)

        grid.addWidget(QLabel("left_finger_joint:"), 1, 0)
        grid.addWidget(self.custom_left_spin, 1, 1)
        grid.addWidget(QLabel("right_finger_joint:"), 2, 0)
        grid.addWidget(self.custom_right_spin, 2, 1)

        custom = QPushButton("SEND CUSTOM GRIPPER OPTIONAL")
        custom.clicked.connect(lambda: self.send_gripper(
            [self.custom_left_spin.value(), self.custom_right_spin.value()],
            "CUSTOM OPTIONAL",
        ))
        grid.addWidget(custom, 2, 2)

        layout.addWidget(group)

        info = QLabel(
            "Note: your current bcr_arm xacro only defines gripper_tip, not left/right finger joints. "
            "So this tab is optional. Picking is done by endpoint attach service."
        )
        info.setWordWrap(True)
        layout.addWidget(info)
        layout.addStretch()
        return tab

    def build_sequence_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        waypoint_group = QGroupBox("Save / Use Factory Transfer Waypoints")
        grid = QGridLayout()
        waypoint_group.setLayout(grid)

        labels = [
            ("Pick Approach", "Move near factory belt 1 object."),
            ("Pick Contact", "Endpoint near object."),
            ("Lift", "Lift after endpoint attach."),
            ("Place Approach", "Move above factory belt 2."),
            ("Place Contact", "Lower to release on factory belt 2."),
        ]

        for row, (name, description) in enumerate(labels):
            save = QPushButton(f"SAVE CURRENT AS {name.upper()}")
            save.clicked.connect(lambda checked=False, n=name: self.save_current_waypoint(n))

            move = QPushButton(f"MOVE TO {name.upper()}")
            move.clicked.connect(lambda checked=False, n=name: self.move_waypoint(n))

            grid.addWidget(QLabel(name), row, 0)
            grid.addWidget(QLabel(description), row, 1)
            grid.addWidget(save, row, 2)
            grid.addWidget(move, row, 3)

        layout.addWidget(waypoint_group)

        seq = QGroupBox("Endpoint Attachment Pick and Place Sequence")
        sg = QGridLayout()
        seq.setLayout(sg)

        self.step_duration_spin = self.make_time_spin(2.0)
        self.wait_after_attach_spin = self.make_time_spin(0.5)
        self.wait_after_release_spin = self.make_time_spin(0.6)

        for r, (label, widget) in enumerate([
            ("Arm step duration sec:", self.step_duration_spin),
            ("Wait after attach sec:", self.wait_after_attach_spin),
            ("Wait after release sec:", self.wait_after_release_spin),
        ]):
            sg.addWidget(QLabel(label), r, 0)
            sg.addWidget(widget, r, 1)

        run = QPushButton("RUN ENDPOINT FACTORY TRANSFER")
        run.clicked.connect(self.on_run_sequence)

        stop = QPushButton("STOP SEQUENCE")
        stop.clicked.connect(self.on_stop_sequence)

        sg.addWidget(run, 3, 0)
        sg.addWidget(stop, 3, 1)

        layout.addWidget(seq)
        layout.addStretch()
        return tab

    def build_log_tab(self):
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)

        clear = QPushButton("CLEAR LOG")
        clear.clicked.connect(self.log_text.clear)
        layout.addWidget(clear)

        return tab

    def make_pose_spin(self, value, minimum=-10.0, maximum=10.0, step=0.01):
        s = QDoubleSpinBox()
        s.setRange(minimum, maximum)
        s.setSingleStep(step)
        s.setDecimals(3)
        s.setValue(value)
        return s

    def make_gripper_spin(self, value):
        s = QDoubleSpinBox()
        s.setRange(0.0, 0.04)
        s.setSingleStep(0.002)
        s.setDecimals(3)
        s.setValue(value)
        return s

    def make_time_spin(self, value):
        s = QDoubleSpinBox()
        s.setRange(0.2, 10.0)
        s.setSingleStep(0.1)
        s.setValue(value)
        return s

    def update_status(self):
        services = self.node.services_ready()
        self.status_label.setText(
            f"joint_states={len(self.node.current_joint_positions)}/7 | "
            f"endpoint services attach={services['attach']}, release={services['release']}, "
            f"move_pick={services['move_pick']}, params={services['params']} | "
            f"sequence={'RUNNING' if self.sequence_running else 'idle'}"
        )

    def log(self, text: str):
        self.log_text.append(text)

    def get_manual_positions(self):
        return [s.value() for s in self.joint_spins]

    def set_manual_positions(self, positions):
        for spin, value in zip(self.joint_spins, positions):
            spin.setValue(float(value))

    def move_arm(self, positions, duration, label):
        try:
            self.node.publish_arm_goal(positions, duration)
            self.log(f"ARM GOAL {label}: {['%.3f' % p for p in positions]} duration={duration:.1f}s")
        except Exception as exc:
            QMessageBox.warning(self, "Arm command failed", str(exc))
            self.log(f"ERROR arm command failed: {exc}")

    def send_gripper(self, values, label):
        try:
            self.node.publish_gripper(values)
            self.log(f"{label}: {values}")
        except Exception as exc:
            self.log(f"OPTIONAL gripper command failed: {exc}")

    def endpoint_base_params(self):
        return {
            "target_model": self.target_model_edit.text().strip() or "pickup_box",
            "world_name": self.world_name_edit.text().strip() or "dual_circular_factory_world",
            "base_frame": self.base_frame_edit.text().strip() or "base_link",
            "endpoint_frame": self.endpoint_frame_edit.text().strip() or "gripper_tip",
            "attach_offset_frame": self.attach_offset_frame_combo.currentText(),
            "attach_offset_x": self.attach_x_spin.value(),
            "attach_offset_y": self.attach_y_spin.value(),
            "attach_offset_z": self.attach_z_spin.value(),
            "attach_rpy_roll": 0.0,
            "attach_rpy_pitch": 0.0,
            "attach_rpy_yaw": 0.0,
            "keep_object_world_flat": self.keep_flat_checkbox.isChecked(),
        }

    def pick_params(self):
        return {
            "pick_x": self.pick_x_spin.value(),
            "pick_y": self.pick_y_spin.value(),
            "pick_z": self.pick_z_spin.value(),
            "pick_qx": 0.0,
            "pick_qy": 0.0,
            "pick_qz": 0.0,
            "pick_qw": 1.0,
        }

    def release_params(self):
        return {
            "exact_release": self.exact_release_checkbox.isChecked(),
            "release_x": self.release_x_spin.value(),
            "release_y": self.release_y_spin.value(),
            "release_z": self.release_z_spin.value(),
            "release_qx": 0.0,
            "release_qy": 0.0,
            "release_qz": 0.0,
            "release_qw": 1.0,
        }

    def set_params_then(self, params, callback: Optional[Callable] = None):
        self.log(f"SETTING PARAMS: {params}")

        try:
            fut = self.node.set_params_async(params)
        except Exception as exc:
            QMessageBox.warning(self, "Endpoint attacher not ready", str(exc))
            self.log(f"ERROR params call failed before send: {exc}")
            return

        def done(f):
            try:
                result = f.result()
                ok = all(r.successful for r in result.results)
                reasons = [r.reason for r in result.results if not r.successful]
                self.log(f"PARAMS {'OK' if ok else 'FAILED'}: {params} {reasons}")

                if ok and callback:
                    callback()

            except Exception as exc:
                self.log(f"ERROR parameter service callback failed: {exc}")

        fut.add_done_callback(done)

    def trigger(self, future_factory, label):
        self.log(f"CALLING {label}...")

        try:
            fut = future_factory()
        except Exception as exc:
            QMessageBox.warning(self, "Endpoint service not ready", str(exc))
            self.log(f"ERROR {label} failed before send: {exc}")
            return

        def done(f):
            try:
                r = f.result()
                self.log(f"{label}: success={r.success}, message={r.message}")
            except Exception as exc:
                self.log(f"ERROR {label} callback failed: {exc}")

        fut.add_done_callback(done)

    def on_apply_endpoint_settings(self):
        self.set_params_then(
            self.endpoint_base_params(),
            lambda: self.log("Endpoint settings applied."),
        )

    def on_apply_pick_pose(self):
        p = self.endpoint_base_params()
        p.update(self.pick_params())
        self.set_params_then(p, lambda: self.log("Pick pose applied."))

    def on_apply_release_settings(self):
        p = self.endpoint_base_params()
        p.update(self.release_params())
        self.set_params_then(p, lambda: self.log("Release settings applied."))

    def on_move_object_to_endpoint_pick(self):
        p = self.endpoint_base_params()
        p.update(self.pick_params())
        self.set_params_then(
            p,
            lambda: self.trigger(self.node.move_pick_async, "MOVE_OBJECT_TO_ENDPOINT_PICK"),
        )

    def on_attach_endpoint(self):
        self.set_params_then(
            self.endpoint_base_params(),
            lambda: self.trigger(self.node.attach_async, "ATTACH_OBJECT_TO_ENDPOINT"),
        )

    def on_release_endpoint(self):
        p = self.endpoint_base_params()
        p.update(self.release_params())
        self.set_params_then(
            p,
            lambda: self.trigger(self.node.release_async, "RELEASE_OBJECT_FROM_ENDPOINT"),
        )

    def on_hybrid_close_attach(self):
        # Current endpoint-only URDF has no finger controller.
        # This publish is optional; real pickup is the attach service below.
        self.send_gripper(GRIPPER_CLOSE, "OPTIONAL CLOSE before endpoint attach")
        QTimer.singleShot(300, self.on_attach_endpoint)

    def on_hybrid_release_open(self):
        def release_then_open():
            try:
                fut = self.node.release_async()
            except Exception as exc:
                QMessageBox.warning(self, "Endpoint release not ready", str(exc))
                self.log(f"ERROR release failed before send: {exc}")
                return

            def done(f):
                try:
                    r = f.result()
                    self.log(f"HYBRID_RELEASE: success={r.success}, message={r.message}")
                    QTimer.singleShot(300, lambda: self.send_gripper(GRIPPER_OPEN, "OPTIONAL OPEN after endpoint release"))
                except Exception as exc:
                    self.log(f"ERROR HYBRID_RELEASE callback failed: {exc}")

            fut.add_done_callback(done)

        p = self.endpoint_base_params()
        p.update(self.release_params())
        self.set_params_then(p, release_then_open)

    def on_move_selected_preset(self):
        item = self.preset_list.currentItem()
        if item is None:
            QMessageBox.information(self, "Select preset", "Select a preset first.")
            return
        self.move_arm(self.saved_waypoints[item.text()], self.duration_spin.value(), item.text())

    def on_load_selected_preset(self):
        item = self.preset_list.currentItem()
        if item is None:
            QMessageBox.information(self, "Select preset", "Select a preset first.")
            return
        self.set_manual_positions(self.saved_waypoints[item.text()])
        self.log(f"Loaded preset: {item.text()}")

    def on_save_current_as_selected_preset(self):
        item = self.preset_list.currentItem()
        if item is None:
            QMessageBox.information(self, "Select preset", "Select a preset first.")
            return
        self.save_current_waypoint(item.text())

    def save_current_waypoint(self, name):
        pos = self.node.current_arm_vector()
        self.saved_waypoints[name] = pos
        self.set_manual_positions(pos)
        self.log(f"SAVED waypoint {name}: {['%.3f' % p for p in pos]}")

    def move_waypoint(self, name):
        self.move_arm(self.saved_waypoints[name], self.step_duration_spin.value(), name)

    def on_run_sequence(self):
        p = self.endpoint_base_params()
        p.update(self.release_params())

        self.sequence_steps = [
            ("arm", self.saved_waypoints["Pick Approach"], "Move to Pick Approach", self.step_duration_spin.value()),
            ("arm", self.saved_waypoints["Pick Contact"], "Move to Pick Contact", self.step_duration_spin.value()),
            ("attach", None, "Attach object to endpoint", self.wait_after_attach_spin.value()),
            ("arm", self.saved_waypoints["Lift"], "Lift object", self.step_duration_spin.value()),
            ("arm", self.saved_waypoints["Place Approach"], "Move to Place Approach", self.step_duration_spin.value()),
            ("arm", self.saved_waypoints["Place Contact"], "Move to Place Contact", self.step_duration_spin.value()),
            ("release", None, "Release object", self.wait_after_release_spin.value()),
        ]

        self.sequence_index = 0
        self.sequence_running = True
        self.log("START endpoint factory transfer sequence.")
        self.set_params_then(p, self.run_next_sequence_step)

    def on_stop_sequence(self):
        self.sequence_running = False
        self.sequence_timer.stop()
        self.log("STOPPED sequence.")

    def run_next_sequence_step(self):
        if not self.sequence_running:
            return

        if self.sequence_index >= len(self.sequence_steps):
            self.sequence_running = False
            self.sequence_timer.stop()
            self.log("DONE endpoint factory transfer sequence.")
            return

        kind, payload, label, wait = self.sequence_steps[self.sequence_index]
        self.sequence_index += 1

        self.log(f"SEQ {self.sequence_index}/{len(self.sequence_steps)}: {label}")

        if kind == "arm":
            self.node.publish_arm_goal(payload, wait)
        elif kind == "attach":
            self.trigger(self.node.attach_async, "SEQ_ATTACH")
        elif kind == "release":
            self.trigger(self.node.release_async, "SEQ_RELEASE")

        self.sequence_timer.start(int(max(0.1, wait) * 1000))

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.status_timer.stop()
        self.sequence_timer.stop()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = BCRArmEndpointNode()

    app = QApplication(sys.argv)
    gui = BCRArmEndpointGui(node)
    gui.show()

    try:
        code = app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(code)


if __name__ == "__main__":
    main()