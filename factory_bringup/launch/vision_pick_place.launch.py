from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    auto_start = LaunchConfiguration("auto_start")
    loop = LaunchConfiguration("loop")
    dry_run = LaunchConfiguration("dry_run")
    mode = LaunchConfiguration("mode")
    show_window = LaunchConfiguration("show_window")
    use_side_camera = LaunchConfiguration("use_side_camera")
    use_release_camera = LaunchConfiguration("use_release_camera")
    wait_at = LaunchConfiguration("wait_at")
    motion_variety = LaunchConfiguration("motion_variety")
    motion_type = LaunchConfiguration("motion_type")
    run_detector = LaunchConfiguration("run_detector")
    run_controller = LaunchConfiguration("run_controller")

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="station",
                              description="station: wait over ring 1 and catch passing products; "
                                          "pick_zone: pick a still object near the arm."),
        DeclareLaunchArgument("auto_start", default_value="true",
                              description="Start as soon as the nodes are up (false: wait for /vision_pick_place/start)."),
        DeclareLaunchArgument("loop", default_value="true",
                              description="Keep catching objects until /vision_pick_place/stop."),
        DeclareLaunchArgument("show_window", default_value="true",
                              description="Open the live 'Object Detection' window."),
        DeclareLaunchArgument("use_side_camera", default_value="true",
                              description="Cross-check with the side camera, stop the product and come down onto it "
                                          "(false: attach from the hover point on the overhead camera alone)."),
        DeclareLaunchArgument("wait_at", default_value="home",
                              description="home: wait at the start position, stop the first bent when an object is confirmed "
                                          "and come to catch it; station: hover over the first bent (previous behaviour)."),
        DeclareLaunchArgument("motion_type", default_value="auto",
                              description="Movement type per cycle (MOTION_TYPES in vision_pick_place.py): auto (by object "
                                          "height), cycle, random, or one of standard, mirror, high_carry, low_fast, "
                                          "full_turn, reverse_return."),
        DeclareLaunchArgument("motion_variety", default_value="true",
                              description="false = always the 'standard' movement type."),
        DeclareLaunchArgument("use_release_camera", default_value="true",
                              description="Wait until the release zone on the second bent is empty before putting an object down."),
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="Detect and solve IK, log the joint goals, but do not move the arm."),
        # the vision lab (vision_lab run) brings its own detector and controller: run_vision_lab.sh sim
        # starts the rest with both false
        DeclareLaunchArgument("run_detector", default_value="true",
                              description="Start ring_object_detector (false: the vision lab's detector replaces it)."),
        DeclareLaunchArgument("run_controller", default_value="true",
                              description="Start vision_pick_place (false: the vision lab drives the arm)."),

        Node(
            package="factory_bringup",
            executable="ring_object_detector",
            name="ring_object_detector",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(run_detector),
            parameters=[{
                "use_sim_time": True,
                "image_topic": "/overhead_camera/image",
                "show_window": ParameterValue(show_window, value_type=bool),

                # <model name="overhead_camera"> in dual_circular_factory_conveyor.sdf
                "camera_x": 0.0,
                "camera_y": 0.0,
                "camera_z": 13.0,
                "camera_pitch": 1.5708,
                "camera_hfov": 1.55,

                # ring 1 = left bent (pick), ring 2 = right bent (place); belt r = 2.95..4.05, search only the product lane
                "ring1_center_x": -6.0,
                "ring2_center_x": 6.0,
                "ring_inner_radius": 3.10,
                "ring_outer_radius": 3.75,
                "belt_z": 1.375,
                "object_top_z": 1.60,

                "gray_margin": 20,
                "canny_low": 15,
                "canny_high": 45,
                "min_area_m2": 0.006,
                "max_area_m2": 0.40,
            }],
        ),

        # side camera (<model name="side_camera"> in the world) into ROS 2
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="side_camera_bridge",
            output="screen",
            arguments=["/side_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            condition=IfCondition(use_side_camera),
        ),

        Node(
            package="factory_bringup",
            executable="side_camera_checker",
            name="side_camera_checker",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(use_side_camera),
            parameters=[{
                "use_sim_time": True,
                "image_topic": "/side_camera/image",
                "show_window": ParameterValue(show_window, value_type=bool),
                # <model name="side_camera"> pose and fov
                "camera_x": -1.50,
                "camera_y": 0.0,
                "camera_z": 1.80,
                "camera_pitch": 0.17,
                "camera_yaw": 3.14159,
                "camera_hfov": 1.10,
                # station plane and the band of belt watched there
                "plane_x": -2.58,
                "band_y_min": -0.70,
                "band_y_max": 0.70,
                "belt_z": 1.42,
                "max_top_z": 1.95,
            }],
        ),

        # release camera (<model name="release_camera"> in the world) and its zone check
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="release_camera_bridge",
            output="screen",
            arguments=["/release_camera/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            condition=IfCondition(use_release_camera),
        ),

        Node(
            package="factory_bringup",
            executable="release_zone_monitor",
            name="release_zone_monitor",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(use_release_camera),
            parameters=[{
                "use_sim_time": True,
                "show_window": ParameterValue(show_window, value_type=bool),
                "camera_x": 2.2171,
                "camera_y": -0.3736,
                "camera_z": 4.5,
                "camera_pitch": 1.5708,
                "camera_hfov": 1.0,
                # the zone = place slot 0 below
                "zone_x": 2.2171,
                "zone_y": -0.3736,
                "zone_radius": 0.40,
                "diff_threshold": 30,
            }],
        ),

        Node(
            package="factory_bringup",
            executable="vision_pick_place",
            name="vision_pick_place",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(run_controller),
            parameters=[{
                "use_sim_time": True,
                "mode": mode,
                "auto_start": ParameterValue(auto_start, value_type=bool),
                "loop": ParameterValue(loop, value_type=bool),
                "dry_run": ParameterValue(dry_run, value_type=bool),

                # base_* / object_height / endpoint_frame are read from the attacher at start.
                "sync_from_attacher": True,

                # station mode: hover over the product path (r = 3.42 around (-6, 0)) nearest the arm
                "station_x": -2.58,
                "station_y": 0.0,
                "hover_z": 2.0,
                "roi_radius": 0.30,
                "catch_latency": 0.35,      # attach delay the vision prediction compensates for
                "track_coast_time": 5.0,    # products are hidden under the arm for up to ~4 s

                # two-camera cross-check, stop the product, come down onto its top, attach
                "use_side_camera": ParameterValue(use_side_camera, value_type=bool),
                "catch_window": 0.25,
                "cross_check_tol": 0.20,
                "side_confirm_timeout": 2.0,
                "descend_gap": 0.01,        # gripper tip 1 cm above the product top when it grips
                "descend_duration": 0.8,

                # carry: small lift, then one smooth swing round to ring 2 (same arm shape both sides)
                "lift_height": 0.50,            # up from the grab before swinging
                "lift_max_z": 2.35,             # but the gripper tip no higher than this (reach limit ~2.4 m)
                "swing_direction": "left",      # left = counter-clockwise seen from above; or "right"
                "carry_swing_duration": 3.0,
                "release_settle_sec": 0.6,

                "pick_zone_x": -2.45,
                "pick_zone_y": 0.0,
                "pick_zone_radius": 0.9,
                "pick_surface_z": 1.375,
                "stable_frames": 5,
                "stable_tolerance": 0.03,

                # slot 0 on ring 2: object put down resting on the belt here, with this yaw
                "place_x": 2.2171,
                "place_y": -0.3736,
                "place_yaw": -1.2781,
                "place_exact": True,
                "place_slots": 1,               # ring 2 moves: always the same release zone
                "use_release_camera": ParameterValue(use_release_camera, value_type=bool),
                "wait_at": wait_at,
                "motion_variety": ParameterValue(motion_variety, value_type=bool),
                "motion_type": motion_type,
                "place_wait_offset": 0.8,       # arm waits this far before the zone until it is empty
                "place_slot_spacing": 0.3,
                "place_surface_z": 1.375,
                "place_tip_height": 0.47,
                "place_drop_clearance": 0.03,

                "approach_height": 0.25,
                "ik_tolerance": 0.05,
                "transit_pose": [0.0, 0.45, 0.0, -0.65, 0.0, 0.35, 0.0],
                "pick_seed": [1.30, -0.75, 0.0, -0.80, 0.0, -1.20, 0.0],
                # arm speed: seconds per short move / per swing between the bents
                "move_duration": 1.0,
                "swing_duration": 2.0,
                "settle_timeout_sec": 1.5,
            }],
        ),
    ])
