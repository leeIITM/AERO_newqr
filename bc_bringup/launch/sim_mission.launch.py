"""SIMULATION mission: the same nodes and mission tree as the real drone, against ArduCopter SITL + Gazebo.

Raspberry Pi (no Gazebo): SITL with built-in physics + the synthetic Mission 2 field
    ros2 launch bc_sim sitl.launch.py                                   # mode:=lite
    ros2 launch bc_bringup sim_mission.launch.py field:=synthetic hud:=false
  field:=synthetic uses bc_bringup/config/delivery_zone.sim.txt and bc_sim/config/sim_lite.yaml.

PC with Gazebo, two terminals (the simulator keeps running between mission attempts):
    ros2 launch bc_sim sitl.launch.py mode:=gazebo    # Gazebo world + ArduCopter SITL + MAVProxy + MAVROS + camera
    ros2 launch bc_bringup sim_mission.launch.py      # flight_controller, detectors, task nodes, mission tree
Or everything at once:
    ros2 launch bc_bringup sim_mission.launch.py with_sitl:=true

Differences from drone.launch.py:
  - MAVROS is NOT started here (sitl.launch.py runs it on MAVProxy's UDP output), unless with_sitl:=true,
    which includes sitl.launch.py.
  - No SIYI camera: /camera/image_raw comes from the Gazebo downward camera (bc_sim's gz_camera_relay).
  - bc_sim/config/sim_mission.yaml is loaded after mission.yaml (fixed camera, latency, reference path).
  - geofence_file defaults to the Gazebo room (bc_sim/config/delivery_zone.gazebo.txt).
Every other mission.launch.py argument works here too (qr_search:=lawnmower, alt:=, hud:=, tree:=, ...).
To restart a mission: stop this launch, reset the world (restart sitl.launch.py), launch this again.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def bc_sim_share():
    try:
        return get_package_share_directory("bc_sim")
    except PackageNotFoundError as exc:
        raise RuntimeError("package bc_sim is not built: colcon build (install.sh without --no-sim)") from exc


def launch_setup(context):
    sim = bc_sim_share()
    bringup = get_package_share_directory("bc_bringup")
    mission = os.path.join(bringup, "launch", "mission.launch.py")
    field = LaunchConfiguration("field").perform(context)
    if field not in ("gazebo", "synthetic"):
        raise RuntimeError("field must be gazebo or synthetic")
    synthetic = field == "synthetic"
    geofence = LaunchConfiguration("geofence_file").perform(context) or (
        os.path.join(bringup, "config", "delivery_zone.sim.txt") if synthetic
        else os.path.join(sim, "config", "delivery_zone.gazebo.txt"))
    extra = LaunchConfiguration("extra_params_file").perform(context) or \
        os.path.join(sim, "config", "sim_lite.yaml" if synthetic else "sim_mission.yaml")
    out = []
    if LaunchConfiguration("with_sitl").perform(context).lower() in ("1", "true", "yes", "on"):
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, "launch", "sitl.launch.py")),
            launch_arguments={"mode": "lite" if synthetic else "gazebo"}.items()))
    out.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mission),
        launch_arguments={
            "bench": "false",
            "spin_motors": "false",
            "start_mavros": "false",
            "camera": "none",
            "extra_params_file": extra,
            "geofence_file": geofence,
            "hud": LaunchConfiguration("hud"),
            "show_window": LaunchConfiguration("show_window"),
        }.items()))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("field", default_value="gazebo",
                              description="gazebo (sitl.launch.py mode:=gazebo) | synthetic (mode:=lite, Raspberry Pi)"),
        DeclareLaunchArgument("with_sitl", default_value="false",
                              description="also start bc_sim's sitl.launch.py (Gazebo, SITL, MAVProxy, MAVROS)"),
        DeclareLaunchArgument("geofence_file", default_value="",
                              description="default: the Gazebo room, or delivery_zone.sim.txt with field:=synthetic"),
        DeclareLaunchArgument("extra_params_file", default_value="",
                              description="default: bc_sim/config/sim_mission.yaml (sim_lite.yaml with field:=synthetic)"),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="true", description="OpenCV HUD window"),
        OpaqueFunction(function=launch_setup),
    ])
