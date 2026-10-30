"""HITL (Hardware-In-The-Loop) Mission:
ArduCopter SITL + MAVProxy (simulated GPS & flight dynamics)
+ Real SIYI A8 mini camera over Ethernet (live RTSP stream to detectors).

Usage:
    ros2 launch bc_bringup hitl_mission.launch.py
    ros2 launch bc_bringup hitl_mission.launch.py camera_src:=rtsp://192.168.144.25:8554/main.264
    ros2 launch bc_bringup hitl_mission.launch.py qr_search:=lawnmower
    ros2 launch bc_bringup hitl_mission.launch.py show_window:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = get_package_share_directory("bc_sim")
    bringup = get_package_share_directory("bc_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("camera_src", default_value="",
                              description="SIYI RTSP URL override; default rtsp://192.168.144.25:8554/main.264"),
        DeclareLaunchArgument("geofence_file",
                              default_value=os.path.join(bringup, "config", "delivery_zone.sim.txt"),
                              description="Geofence file for search zone"),
        DeclareLaunchArgument("mavproxy", default_value="true",
                              description="Run MAVProxy for QGC / telemetry"),
        DeclareLaunchArgument("qr_search", default_value="target_search",
                              description="target_search | lawnmower"),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="true",
                              description="OpenCV GUI window showing camera + HUD detections"),
        DeclareLaunchArgument("tree", default_value="true",
                              description="Run autonomous mission tree"),

        # 1. Start SITL (simulated GPS & MAVProxy, without synthetic camera)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, "launch", "sitl.launch.py")),
            launch_arguments={
                "mode": "lite",
                "camera": "none",
                "mavproxy": LaunchConfiguration("mavproxy"),
            }.items(),
        ),

        # 2. Start Full Mission Stack with Real SIYI Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "mission.launch.py")),
            launch_arguments={
                "bench": "false",
                "start_mavros": "false",
                "camera": "siyi",
                "camera_src": LaunchConfiguration("camera_src"),
                "geofence_file": LaunchConfiguration("geofence_file"),
                "qr_search": LaunchConfiguration("qr_search"),
                "hud": LaunchConfiguration("hud"),
                "show_window": LaunchConfiguration("show_window"),
                "tree": LaunchConfiguration("tree"),
            }.items(),
        ),
    ])
