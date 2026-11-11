from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="factory_bringup",
            executable="bcr_fake_gripper",
            name="bcr_fake_gripper",
            output="screen",
            emulate_tty=True,
            parameters=[{
                # Gazebo world and object
                "world_name": "dual_circular_factory_world",
                "target_model": "pickup_box",

                # TF frames
                "base_frame": "base_link",
                "tool_frame": "gripper_tip",

                # Optional base offset if Gazebo and TF origins differ
                "base_x": 0.0,
                "base_y": 0.0,
                "base_z": 0.0,

                # Stable hold values from your successful tests
                "debug_mode": True,
                "preserve_relative_pose": False,
                "snap_on_pick": True,
                "flat_object_orientation": False,

                "hold_offset_x": -0.12,
                "hold_offset_y": 0.0,
                "hold_offset_z": -0.08,
                "hold_rpy_roll": 0.0,
                "hold_rpy_pitch": 0.0,
                "hold_rpy_yaw": 0.0,

                # Development default: ignore distance rejection.
                # For final demo, set debug_mode=false and max_pick_distance=0.5.
                "max_pick_distance": 0.0,

                # Pick-zone location.
                # Tune pick_x/y/z to the red-rectangle gripper/conveyor pick point.
                "pick_x": 0.57,
                "pick_y": 0.0,
                "pick_z": 3.70,
                "pick_qx": 0.0,
                "pick_qy": 0.0,
                "pick_qz": 0.0,
                "pick_qw": 1.0,

                # For repeatable tests, you can set this true from GUI/terminal.
                # For realistic conveyor picking, keep it false.
                "auto_move_object_to_pick_zone": False,
                "use_pick_zone_for_distance": False,
                "pick_zone_max_distance": 0.35,

                # Precise placement default.
                "exact_place_on_release": True,
                "place_x": 0.57,
                "place_y": 0.0,
                "place_z": 3.70,
                "place_qx": 0.0,
                "place_qy": 0.0,
                "place_qz": 0.0,
                "place_qw": 1.0,

                # Precise place settle behavior
                "place_settle_repeats": 3,
                "place_settle_sleep_sec": 0.05,

                # Runtime settings
                "update_hz": 25.0,
                "gz_timeout_ms": 300,
                "verbose_debug": True,
            }],
        )
    ])