from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="factory_bringup",
            executable="bcr_arm_simple_ui",
            name="bcr_arm_simple_ui",
            output="screen",
            emulate_tty=True,
        )
    ])