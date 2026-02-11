from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    world = LaunchConfiguration("world")

    gazebo_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="/home/sangam/Documents/Acad/sem-4/IRPP/PROJ/scan_project/gazebo_worlds/complex_grid_map.world",
            description="Gazebo world file",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={"gz_args": ["-r ", world]}.items(),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="pose_tf_bridge",
            output="screen",
            arguments=[
                "/world/complex_grid_map/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="set_pose_bridge",
            output="screen",
            arguments=[
                "/world/complex_grid_map/set_pose@ros_gz_interfaces/srv/SetEntityPose"
            ],
        ),
        Node(
            package="ros_humans_ros2",
            executable="move_humans",
            name="move_humans",
            output="screen",
            parameters=[{"rate": 10.0, "world": "complex_grid_map"}],
        ),
        Node(
            package="ros_humans_ros2",
            executable="publish_human_pose",
            name="human_pose_publisher",
            output="screen",
            parameters=[{"frame_id": "world"}],
        ),
    ])
