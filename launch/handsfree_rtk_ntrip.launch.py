from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    timeout = LaunchConfiguration("timeout")
    frame_id = LaunchConfiguration("frame_id")
    ntrip_server = LaunchConfiguration("ntrip_server")
    ntrip_port = LaunchConfiguration("ntrip_port")
    ntrip_username = LaunchConfiguration("ntrip_username")
    ntrip_password = LaunchConfiguration("ntrip_password")
    ntrip_mountpoint = LaunchConfiguration("ntrip_mountpoint")
    gga_period = LaunchConfiguration("gga_period")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/HFRobotRTK"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("timeout", default_value="1.0"),
        DeclareLaunchArgument("frame_id", default_value="gps"),
        DeclareLaunchArgument("ntrip_server", default_value="120.253.239.161"),
        DeclareLaunchArgument("ntrip_port", default_value="8002"),
        DeclareLaunchArgument("ntrip_username", default_value="ctea952"),
        DeclareLaunchArgument("ntrip_password", default_value="cm286070"),
        DeclareLaunchArgument("ntrip_mountpoint", default_value="RTCM33_GRCE"),
        DeclareLaunchArgument("gga_period", default_value="3.0"),
        Node(
            package="handsfree_rtk",
            executable="cpp_driver_um98x_ntrip",
            name="handsfree_rtk_ntrip",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": baudrate,
                "timeout": timeout,
                "frame_id": frame_id,
                "ntrip_server": ntrip_server,
                "ntrip_port": ntrip_port,
                "ntrip_username": ntrip_username,
                "ntrip_password": ntrip_password,
                "ntrip_mountpoint": ntrip_mountpoint,
                "gga_period": gga_period,
            }],
        ),
    ])
