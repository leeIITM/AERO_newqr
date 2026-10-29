"""REAL DRONE mission: Cube Orange+ (ArduCopter 4.5) on the Pixhawk's USB port, SIYI A8 mini over RTSP.

    ros2 launch bc_bringup drone.launch.py geofence_file:=/home/pi/delivery_zone.txt
    ros2 launch bc_bringup drone.launch.py geofence_file:=... fcu_url:=/dev/serial/by-id/usb-CubePilot_CubeOrange+_0-if00:115200
    ros2 launch bc_bringup drone.launch.py qr_search:=lawnmower          # script's lawnmower: no geofence needed
    ros2 launch bc_bringup drone.launch.py geofence_file:=... gcs_url:=udp://@192.168.1.10:14550   # + QGC

Starts MAVROS on `fcu_url` first, in its own gnome-terminal window (stopped and closed with the launch on Ctrl+C) (mavros_terminal:=none: in this terminal; no display,
e.g. over SSH: always here) (USB CDC: the baud rate is ignored by the port, 115200 is conventional), the real
flight_controller, the SIYI RTSP camera, every task node and the mission tree. It is mission.launch.py with
bench:=false camera:=siyi start_mavros:=true fixed; every other mission.launch.py argument works here too
(alt:=, corridor_alt:=, hud:=, show_window:=, tree:=, params_file:=, ...).

Before the first flight: the user running ROS must be in the `dialout` group (sudo usermod -aG dialout $USER,
then log out/in) and MAVROS needs the GeographicLib datasets (install.sh installs them).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def check_port(context):
    url = LaunchConfiguration("fcu_url").perform(context)
    device = url.split(":")[0] if url.startswith("/dev/") else ""
    if device and not os.path.exists(device):
        by_id = "/dev/serial/by-id"
        found = ", ".join(os.path.join(by_id, d) for d in sorted(os.listdir(by_id))) if os.path.isdir(by_id) else "none"
        return [LogInfo(msg=f"[drone.launch] WARNING: {device} does not exist. Is the Cube plugged in by USB? "
                            f"Serial devices: {found}. MAVROS keeps retrying.")]
    if device and not os.access(device, os.R_OK | os.W_OK):
        return [LogInfo(msg=f"[drone.launch] WARNING: no read/write access to {device}: "
                            "add yourself to the dialout group (sudo usermod -aG dialout $USER) and log in again.")]
    return [LogInfo(msg=f"[drone.launch] FCU link {url}")]


def generate_launch_description():
    mission = os.path.join(get_package_share_directory("bc_bringup"), "launch", "mission.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:115200",
                              description="Pixhawk USB port (prefer /dev/serial/by-id/...:115200)"),
        DeclareLaunchArgument("gcs_url", default_value="", description="optional GCS forward, e.g. udp://@<ip>:14550"),
        DeclareLaunchArgument("camera_src", default_value="", description="override the SIYI RTSP URL"),
        DeclareLaunchArgument("mavros_terminal", default_value="auto",
                              description="MAVROS window: auto (gnome-terminal if there is a display) | none | gnome-terminal | xterm"),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="false"),
        OpaqueFunction(function=check_port),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mission),
            launch_arguments={
                "bench": "false",
                "spin_motors": "false",
                "start_mavros": "true",
                "camera": "siyi",
                "fcu_url": LaunchConfiguration("fcu_url"),
                "mavros_terminal": LaunchConfiguration("mavros_terminal"),
                "gcs_url": LaunchConfiguration("gcs_url"),
                "camera_src": LaunchConfiguration("camera_src"),
                "hud": LaunchConfiguration("hud"),
                "show_window": LaunchConfiguration("show_window"),
            }.items()),
    ])
