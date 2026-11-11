from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    factory_world_path = LaunchConfiguration('factory_world_path')
    use_moveit = LaunchConfiguration('use_moveit')

    declare_factory_world_path = DeclareLaunchArgument(
        'factory_world_path',
        default_value='/home/huseyin/ros2_ws/src/factory_bent_project/my_factory_worlds/worlds/dual_circular_factory_conveyor.sdf',
        description='Absolute path to the dual circular factory conveyor SDF world.'
    )

    declare_use_moveit = DeclareLaunchArgument(
        'use_moveit',
        default_value='false',
        description='Use BCR Arm MoveIt2 launch instead of Gazebo-only launch.'
    )

    # Gazebo + BCR Arm launch.
    # This launch supports world_path according to the BCR Arm README.
    bcr_arm_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bcr_arm_gazebo'),
                'launch',
                'bcr_arm.gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world_path': factory_world_path
        }.items()
    )

    # Bridge your overhead Gazebo camera into ROS2.
    # Your report uses /overhead_camera/image as the ROS2 image topic.
    camera_bridge = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_factory_world_path,
        declare_use_moveit,
        bcr_arm_gazebo_launch,
        camera_bridge,
    ])
