from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    timeout = LaunchConfiguration("timeout")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/HFRobotRTK"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("timeout", default_value="0.1"),
        DeclareLaunchArgument("frame_id", default_value="gps"),
        Node(
            package="handsfree_rtk",
            executable="cpp_driver_um98x",
            name="handsfree_rtk",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": baudrate,
                "timeout": timeout,
                "frame_id": frame_id,
            }],
        ),
    ])
