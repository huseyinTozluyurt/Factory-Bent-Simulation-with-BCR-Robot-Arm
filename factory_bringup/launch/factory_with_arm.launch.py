import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    factory_bringup_pkg = get_package_share_directory("factory_bringup")
    arm_pkg = get_package_share_directory("rk_demo")

    # Go from:
    # install/factory_bringup/share/factory_bringup
    # back to source path is not reliable after install.
    # So for now we use your source world path directly:
    world_path = os.path.expanduser(
        "~/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/dual_circular_factory_conveyor.sdf"
    )

    arm_xacro = os.path.join(
        arm_pkg,
        "urdf",
        "robot_arm.urdf.xacro"
    )

    gz_resource_path = (
        os.path.dirname(factory_bringup_pkg) + ":" +
        os.path.dirname(arm_pkg) + ":" +
        os.path.expanduser("~/ros2_ws/src/factory_bent_project/my_factory_worlds") + ":" +
        os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    )

    robot_description = ParameterValue(
        Command(["xacro ", arm_xacro]),
        value_type=str
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_resource_path
        ),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py"
                )
            ),
            launch_arguments={
                "gz_args": f"-r {world_path}"
            }.items()
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": True,
                }
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    output="screen",
                    arguments=[
                        "-name", "factory_robot_arm",
                        "-topic", "robot_description",
                        "-x", "0.0",
                        "-y", "0.0",
                        "-z", "0.20",
                        "-R", "0.0",
                        "-P", "0.0",
                        "-Y", "0.0",
                    ],
                )
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen",
                ),

                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_trajectory_controller"],
                    output="screen",
                ),
            ],
        ),
    ])