"""Banner-corridor mission: every node + the mission tree (the base launch file).

Use the wrappers for the two main cases:
    ros2 launch bc_bringup drone.launch.py geofence_file:=/home/pi/delivery_zone.txt     real drone (USB)
    ros2 launch bc_bringup sim_mission.launch.py                                         against bc_sim's SITL
This file directly (same arguments):
    ros2 launch bc_bringup mission.launch.py geofence_file:=/home/pi/delivery_zone.txt
    ros2 launch bc_bringup mission.launch.py qr_search:=lawnmower        # script's lawnmower, no geofence
Bench, simulated flight + real SIYI camera (as --bench-sim):
    ros2 launch bc_bringup mission.launch.py bench:=true qr_search:=lawnmower
Bench, real motors spinning, PROPELLERS OFF (as --spin-motors):
    ros2 launch bc_bringup mission.launch.py bench:=true spin_motors:=true thrust:=0.25 qr_search:=lawnmower
Bench, simulated flight + synthetic camera (no hardware at all; the synthetic field's geofence):
    ros2 launch bc_bringup mission.launch.py bench:=true camera:=sim \
        geofence_file:=$(ros2 pkg prefix bc_bringup)/share/bc_bringup/config/delivery_zone.sim.txt
Without the mission tree (to run single actions by hand):
    ros2 launch bc_bringup mission.launch.py tree:=false
MAVROS starts FIRST, in its own gnome-terminal window (mavros_terminal:=auto, when there is a display); every
other node logs in this terminal and is started once MAVROS reports the autopilot connected
(wait_for_mavros, up to mavros_wait_timeout = 60 s, then they start anyway). mavros_terminal:=none keeps MAVROS here too (always the case over SSH).
Other structural arguments: start_mavros (auto | true | false; auto = not bench, or spin_motors),
extra_params_file (a YAML loaded after params_file, e.g. bc_sim/config/sim_mission.yaml).

Stage 9 (qr_search): target_search (default; needs geofence_file) or lawnmower (the script's serpentine).
Both are always started. red_zone_detector starts IDLE: the mission tree switches it on only in the
delivery zone (target search -> navigator -> centring -> drop) and off again; with lawnmower it stays off.
With target_search, payload_drop (default true) adds: geotagged frames every capture_spacing m during the
search (survey_capture), target_navigator to the QR's coordinates, centring + hold, then the drop
(payload_drop: SERVO drop_channel open_pwm -> drop_interval -> close_pwm). On the bench without MAVROS the
drop is simulated (logged only).

Every parameter in config/mission.yaml is also a launch argument, e.g.
    surge1:=2.0 surge2:=2.0 alt:=5.0 corridor_alt:=3.0 max_sway:=20.0 blue_max_sway:=20.0 thrust:=0.2
A parameter whose name appears under several nodes is prefixed with the node: qr_detector_match_thresh:=0.9.
An empty argument (the default) keeps the params file's value. List them all with:
    ros2 launch bc_bringup mission.launch.py --show-args

The standalone vision tuning tool (no ROS graph): ros2 run banner_detector vision_test --color green
"""
import os
import tempfile
from collections import Counter

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from bc_common.terminal import (MAVROS_ACK_TIMEOUT, mavros_script, pick_terminal, refuse_second_mavros,
                                window_command)

DEFAULT_PARAMS = os.path.join(get_package_share_directory("bc_bringup"), "config", "mission.yaml")
STRUCTURAL = {"params_file", "extra_params_file", "bench", "spin_motors", "start_mavros", "mavros_terminal", "camera",
              "camera_src", "fcu_url", "gcs_url", "hud", "show_window", "tree", "wait_for_mavros", "mavros_wait_timeout"}
# <ws>/install/bc_bringup/share/bc_bringup -> <ws>/install/setup.bash (sourced in the MAVROS window)
WS_SETUP = os.path.abspath(os.path.join(get_package_share_directory("bc_bringup"), "..", "..", "..", "setup.bash"))


def parameter_arguments():
    """[(launch argument, node section, parameter, default value, comment)] for every mission.yaml parameter."""
    with open(DEFAULT_PARAMS) as f:
        cfg = yaml.safe_load(f)
    entries = [(section.lstrip("/"), name, value)
               for section, body in cfg.items() for name, value in body["ros__parameters"].items()
               if name not in STRUCTURAL]  # e.g. camera_src: already a launch argument
    count = Counter(name for _, name, _ in entries)
    return [(name if count[name] == 1 else f"{section}_{name}", section, name, value)
            for section, name, value in entries]


def typed(text, default):
    """Launch argument text -> the type of the params file's default."""
    value = yaml.safe_load(text)
    if isinstance(default, bool):
        if not isinstance(value, bool):
            raise RuntimeError(f"expected true/false, got {text!r}")
        return value
    if isinstance(default, float):
        return float(value)
    if isinstance(default, int):
        return int(value)
    if isinstance(default, str):
        return str(text)
    return value


def truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true", "yes", "on")


def effective(files, section, name, default=None):
    """Value of section.name after loading the parameter files in order (later files win)."""
    value = default
    for path in files:
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
        for key in (section, "/" + section):
            body = (cfg.get(key) or {}).get("ros__parameters") or {}
            if name in body:
                value = body[name]
    return value


def nodes(context):
    params = LaunchConfiguration("params_file").perform(context)
    bench = truthy(context, "bench")
    spin_motors = truthy(context, "spin_motors")
    camera = LaunchConfiguration("camera").perform(context)
    if camera not in ("siyi", "sim", "none"):
        raise RuntimeError("camera must be siyi, sim or none")
    if camera == "sim" and not bench:
        raise RuntimeError("camera:=sim renders from the simulated pose: use it with bench:=true")

    start_mavros = LaunchConfiguration("start_mavros").perform(context).lower()
    if start_mavros not in ("auto", "true", "false"):
        raise RuntimeError("start_mavros must be auto, true or false")
    if start_mavros == "auto":
        start_mavros = "true" if (not bench or spin_motors) else "false"

    overrides = {}  # node section -> {parameter: value} from non-empty launch arguments
    for arg, section, name, default in parameter_arguments():
        text = LaunchConfiguration(arg).perform(context)
        if text != "":
            overrides.setdefault(section, {})[name] = typed(text, default)
    files = [params]
    extra = LaunchConfiguration("extra_params_file").perform(context)
    if extra:
        files.append(os.path.expanduser(extra))
    # Forced values go into the same override file (node sections after the params file)
    if bench and start_mavros == "false":
        overrides.setdefault("payload_drop", {})["simulate"] = True   # no MAVROS: log the drop only
    if bench and camera == "siyi":
        overrides.setdefault("banner_detector", {})["check_physical_size"] = False
    if overrides:
        # Same node sections as the params file, loaded after it. (Inline dicts would be written under
        # /**, and a node's own section in the params file takes precedence over /**.)
        fd, path = tempfile.mkstemp(prefix="bc_overrides_", suffix=".yaml")
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump({("/**" if s == "**" else s): {"ros__parameters": v} for s, v in overrides.items()}, f)
        files.append(path)

    if truthy(context, "tree"):
        qr_search = effective(files, "mission_tree", "qr_search", "target_search")
        geofence = effective(files, "mission_tree", "geofence_file", "")
        if qr_search not in ("target_search", "lawnmower"):
            raise RuntimeError(f"qr_search must be target_search or lawnmower, not {qr_search!r}")
        if qr_search == "target_search":
            if not geofence:
                raise RuntimeError("qr_search is target_search: give the delivery zone with geofence_file:=<file> "
                                   "(one 'lat,lon' corner per line), or use qr_search:=lawnmower")
            if not os.path.isfile(os.path.expanduser(geofence)):
                raise RuntimeError(f"geofence_file {geofence} does not exist")
    out = []
    first = []  # MAVROS: started before everything else

    def node(pkg, exe, section, extra=None):
        return Node(package=pkg, executable=exe, output="screen", parameters=files + ([extra] if extra else []))

    if start_mavros == "true":
        refuse_second_mavros()   # never two MAVROS on one autopilot
        mavros = get_package_share_directory("mavros")
        terminal = pick_terminal(LaunchConfiguration("mavros_terminal").perform(context))
        if terminal != "none":  # MAVROS in its own window, every other node in this terminal
            script = mavros_script(LaunchConfiguration("fcu_url").perform(context),
                                   LaunchConfiguration("gcs_url").perform(context), mavros_share=mavros)
            ros_setup = f"/opt/ros/{os.environ.get('ROS_DISTRO', 'jazzy')}/setup.bash"
            first.append(ExecuteProcess(cmd=window_command("MAVROS", script, terminal, ros_setup, WS_SETUP),
                                      name="mavros_window", output="log"))
        else:
            first.append(Node(
                package="mavros", executable="mavros_node", output="screen",  # no name=: it renames every node
                parameters=[os.path.join(mavros, "launch", "apm_pluginlists.yaml"),
                            os.path.join(mavros, "launch", "apm_config.yaml"),
                            {"fcu_url": LaunchConfiguration("fcu_url"), "gcs_url": LaunchConfiguration("gcs_url"),
                             "target_system_id": 1, "target_component_id": 1, "fcu_protocol": "v2.0",
                             "command_ack_timeout": MAVROS_ACK_TIMEOUT}]))
    if bench:
        out.append(node("bench_flight_controller", "bench_controller_node", "flight_controller",
                        {"spin_motors": spin_motors}))
    else:
        out.append(node("flight_controller", "flight_controller_node", "flight_controller"))

    if camera == "siyi":
        src = LaunchConfiguration("camera_src").perform(context)
        out.append(node("siyi_camera", "rtsp_camera_node", "siyi_camera", {"camera_src": src} if src else None))
    elif camera == "sim":
        out.append(node("bench_flight_controller", "sim_camera_node", "sim_camera"))

    # On the bench with the real camera the simulated altitude is not the banner's real distance:
    # skip the physical length check (the original script did the same in --bench-sim).
    detector_extra = None  # bench + real camera: check_physical_size false, set in the override file above
    out += [
        node("banner_detector", "banner_detector_node", "banner_detector", detector_extra),
        node("qr_detector", "qr_detector_node", "qr_detector"),
        node("banner_search", "banner_search_node", "banner_search"),
        node("banner_align", "banner_align_node", "banner_align"),
        node("corridor_navigator", "corridor_navigator_node", "corridor_navigator"),
        node("red_zone_detector", "red_zone_detector_node", "red_zone_detector"),
        node("target_search", "target_search_node", "target_search"),
        node("target_search", "lawnmower_search_node", "lawnmower_search"),
        node("target_navigator", "target_navigator_node", "target_navigator"),
        node("payload_drop", "payload_drop_node", "payload_drop"),
        node("payload_drop", "survey_capture_node", "survey_capture"),
        node("qr_centering", "qr_centering_node", "qr_centering"),
    ]
    if truthy(context, "hud"):
        out.append(node("mission_hud", "hud_node", "mission_hud", {"show_window": truthy(context, "show_window")}))
    if truthy(context, "tree"):
        out.append(node("mission_management", "mission_tree", "mission_tree"))
    wait = LaunchConfiguration("wait_for_mavros").perform(context).lower()
    if wait not in ("auto", "true", "false"):
        raise RuntimeError("wait_for_mavros must be auto, true or false")
    if wait == "false" or (wait == "auto" and start_mavros != "true"):
        return first + out
    # MAVROS first; every other node starts once MAVROS reports the autopilot connected (/mavros/state),
    # or after mavros_wait_timeout s anyway (the flight controller keeps waiting for it).
    timeout = float(LaunchConfiguration("mavros_wait_timeout").perform(context))
    waiter = ExecuteProcess(name="wait_for_mavros", output="screen", cmd=["bash", "-c", "\n".join([
        f"echo '[wait_for_mavros] waiting up to {timeout:.0f} s for MAVROS to connect to the autopilot ...'",
        f"if timeout {timeout:.0f} bash -c 'until ros2 topic echo --once /mavros/state 2>/dev/null "
        "| grep -q \"connected: true\"; do sleep 1; done'; then",
        "  echo '[wait_for_mavros] MAVROS connected: starting the mission nodes'",
        "else",
        f"  echo '[wait_for_mavros] WARNING: no autopilot connection after {timeout:.0f} s: starting the "
        "mission nodes anyway (they wait for it)'",
        "fi"])])
    return first + [waiter, RegisterEventHandler(OnProcessExit(target_action=waiter, on_exit=out))]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("params_file", default_value=DEFAULT_PARAMS),
        DeclareLaunchArgument("extra_params_file", default_value="",
                              description="YAML loaded after params_file (e.g. sim overrides)"),
        DeclareLaunchArgument("bench", default_value="false", description="bench_flight_controller instead of MAVROS"),
        DeclareLaunchArgument("spin_motors", default_value="false",
                              description="bench only: spin the real motors in GUIDED_NOGPS (PROPS OFF)"),
        DeclareLaunchArgument("wait_for_mavros", default_value="auto",
                              description="start the other nodes only once MAVROS is connected: auto (= when this "
                                          "launch starts MAVROS) | true | false"),
        DeclareLaunchArgument("mavros_wait_timeout", default_value="60.0",
                              description="s to wait for the MAVROS connection before starting the nodes anyway"),
        DeclareLaunchArgument("mavros_terminal", default_value="auto",
                              description="MAVROS in its own window: auto (gnome-terminal if there is a display) | "
                                          "none (this terminal) | gnome-terminal | xterm"),
        DeclareLaunchArgument("start_mavros", default_value="auto",
                              description="auto (= not bench, or spin_motors) | true | false"),
        DeclareLaunchArgument("camera", default_value="siyi", description="siyi | sim | none"),
        DeclareLaunchArgument("camera_src", default_value="", description="override RTSP URL / device index"),
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:115200",
                              description="MAVROS FCU link (USB: /dev/ttyACM0:115200)"),
        DeclareLaunchArgument("gcs_url", default_value=""),
        DeclareLaunchArgument("hud", default_value="true"),
        DeclareLaunchArgument("show_window", default_value="false", description="OpenCV HUD window"),
        DeclareLaunchArgument("tree", default_value="true", description="start the mission tree"),
    ]
    for arg, section, name, default in parameter_arguments():
        args.append(DeclareLaunchArgument(arg, default_value="",
                                          description=f"{section}.{name} (params file: {default})"))
    return LaunchDescription(args + [OpaqueFunction(function=nodes)])
