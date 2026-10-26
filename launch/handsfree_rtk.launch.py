from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_raw_log_dir = str(Path(__file__).resolve().parents[1] / "logs")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    frame_id = LaunchConfiguration("frame_id")
    heading_frame_id = LaunchConfiguration("heading_frame_id")
    raw_log_dir = LaunchConfiguration("raw_log_dir")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/HFRobotRTK"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("frame_id", default_value="gps"),
        DeclareLaunchArgument("heading_frame_id", default_value="map"),
        DeclareLaunchArgument("raw_log_dir", default_value=default_raw_log_dir),
        Node(
            package="handsfree_rtk",
            executable="cpp_driver_um98x",
            name="handsfree_rtk",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": baudrate,
                "frame_id": frame_id,
                "heading_frame_id": heading_frame_id,
                "raw_log_dir": raw_log_dir,
            }],
        ),
    ])
