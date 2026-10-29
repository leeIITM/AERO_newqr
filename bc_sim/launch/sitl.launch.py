"""SITL for the banner-corridor mission, two modes.

mode:=lite (default; Raspberry Pi 5, no Gazebo, no GPU needed)
    ArduCopter SITL     Copter-4.5 binary with its built-in quad physics (--model quad), home 13.0827,80.2707,
                        heading 0 (north), copter.parm + bc_sim/config/sitl.parm
    MAVROS              straight to SITL's SERIAL0, tcp://127.0.0.1:5760 (no MAVProxy needed)
    sim_camera          the workspace's synthetic downward camera (bench_flight_controller), rendering the
                        Mission 2 field from /flight_controller/pose -> /camera/image_raw
    [MAVProxy]          only with mavproxy:=true: then MAVROS goes through it (udp 14555) and QGroundControl
                        can connect on udp 14550 (gcs_out:=udp:<laptop-ip>:14550 for another machine)
  Verifies the real path: MAVROS -> flight_controller -> ArduPilot GUIDED take-off / setpoints / land, the
  detectors, target search with red avoidance, navigator, centring, payload drop (DO_SET_SERVO), the tree.
  Run the mission against it with the synthetic field:
      ros2 launch bc_sim sitl.launch.py
      ros2 launch bc_bringup sim_mission.launch.py field:=synthetic

mode:=external (you start SITL yourself with sim_vehicle.py; do NOT also use lite/gazebo, they start
another SITL on the same ports)
    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -v ArduCopter -l 13.0827,80.2707,10,0 \
        --add-param-file=$(ros2 pkg prefix bc_sim)/share/bc_sim/config/sitl.parm
    ros2 launch bc_sim sitl.launch.py mode:=external         # MAVROS on udp://:14551@ + synthetic camera
    ros2 launch bc_bringup sim_mission.launch.py field:=synthetic hud:=false

mode:=gazebo (PC with a GPU)
    Gazebo Harmonic world aerothon_mission2_ardupilot.sdf, ArduCopter SITL --model JSON (ardupilot_gazebo),
    MAVProxy, MAVROS (udp 14555), ros_gz_bridge + gz_camera_relay for the Gazebo camera.
      ros2 launch bc_sim sitl.launch.py mode:=gazebo
      ros2 launch bc_bringup sim_mission.launch.py

Order: the MAVROS window opens FIRST (it waits for SITL / MAVProxy by itself), the rest 2 s later.
Terminals: everything runs and logs in THIS terminal, except MAVROS, which opens in its own window
(mavros_terminal:=auto: a gnome-terminal window when there is a display; with no display, e.g. over SSH,
it runs here too). mavros_terminal:=none keeps it here. Ctrl+C here stops MAVROS and closes its window;
if MAVROS exits by itself the window stays open so errors can be read.

Other arguments: wipe (true: fresh SITL parameters), home, ardupilot_dir ($ARDUPILOT_HOME or ~/ardupilot),
ardupilot_gazebo ($ARDUPILOT_GAZEBO_HOME), gui (gazebo), console (MAVProxy console + map), gcs_out,
mavros, ros_setup, ws_setup. ArduCopter needs ~20-40 s after start for a GPS fix and the EKF origin;
flight_controller reports ready only then and the mission tree waits for it.
"""
import os
import shlex

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration

from bc_common.terminal import mavros_script, pick_terminal, refuse_second_mavros, window_command

SHARE = get_package_share_directory("bc_sim")
WORLD = os.path.join(SHARE, "worlds", "aerothon_mission2_ardupilot.sdf")
# <ws>/install/bc_sim/share/bc_sim -> <ws>/install/setup.bash
WS_SETUP = os.path.abspath(os.path.join(SHARE, "..", "..", "..", "setup.bash"))
HOME = "13.0827,80.2707,10,0"   # Gazebo world origin = SITL home: lat,lon,alt(AMSL),heading
RUN_DIR = os.path.expanduser("~/.ros/bc_sitl")


def truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("1", "true", "yes", "on")


def q(cmd):
    return " ".join(shlex.quote(c) for c in cmd)


def launch_setup(context):
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    mode = arg("mode")
    if mode not in ("lite", "gazebo", "external"):
        raise RuntimeError("mode must be lite, gazebo or external")
    gazebo = mode == "gazebo"
    external = mode == "external"   # SITL started by you (sim_vehicle.py): only MAVROS + synthetic camera
    ap_dir = os.path.expanduser(arg("ardupilot_dir"))
    apgz = os.path.expanduser(arg("ardupilot_gazebo"))
    ros_setup = os.path.expanduser(arg("ros_setup"))
    ws_setup = os.path.expanduser(arg("ws_setup"))
    use_mavproxy = (gazebo or truthy(context, "mavproxy")) and not external
    if external:
        out = [LogInfo(msg="[sitl.launch] mode external: YOUR sim_vehicle.py SITL + MAVProxy; starting only MAVROS "
                           f"({arg('fcu_url') or 'udp://:14551@'}) + the synthetic camera")]
    else:
        out = [LogInfo(msg=f"[sitl.launch] mode {mode}: ArduCopter SITL"
                           f"{' + Gazebo' if gazebo else ' (built-in quad physics) + synthetic camera'}"
                           f"{' + MAVProxy' if use_mavproxy else ''}{' + MAVROS' if truthy(context, 'mavros') else ''}")]

    # ---------------------------------------------------------------- where programs run
    mavros_terminal = pick_terminal(arg("mavros_terminal"))   # only MAVROS gets its own window
    if truthy(context, "mavros"):
        refuse_second_mavros()   # never two MAVROS on one autopilot

    env_lines = ""
    if gazebo:
        resource = ":".join([os.path.join(SHARE, "models"), os.path.join(SHARE, "worlds"),
                             os.path.join(apgz, "models"), os.path.join(apgz, "worlds")])
        env_lines = "\n".join([
            "export GZ_VERSION=harmonic",
            f"export GZ_SIM_RESOURCE_PATH={shlex.quote(resource)}${{GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}}",
            f"export GZ_SIM_SYSTEM_PLUGIN_PATH={shlex.quote(os.path.join(apgz, 'build'))}"
            "${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}",
        ])

    # MAVROS (its window) starts first; everything else follows `after_mavros` s later
    after_mavros = 2.0 if truthy(context, "mavros") else 0.0

    def run(title, script, delay=0.0, terminal="none"):
        """`script` (bash) in this launch's terminal, or in its own window (terminal != 'none')."""
        if title != "MAVROS":
            delay += after_mavros
        cmd = window_command(title, script, terminal, ros_setup, ws_setup, env_lines)
        proc = ExecuteProcess(cmd=cmd, name=title.lower().replace(" ", "_"),
                              output="screen" if terminal == "none" else "log", sigterm_timeout="5")
        return TimerAction(period=delay, actions=[proc]) if delay > 0 else proc

    # ---------------------------------------------------------------- checks
    binary = os.path.join(ap_dir, "build", "sitl", "bin", "arducopter")
    checks = [(binary, "build it: cd ~/ardupilot && ./waf configure --board sitl && ./waf copter "
                       "(install.sh --sitl-only on the Pi)"),
              (ros_setup, "ros_setup:=<ROS setup.bash>"),
              (ws_setup, "build the workspace, or ws_setup:=<ws>/install/setup.bash")]
    if gazebo:
        checks.append((os.path.join(apgz, "build", "libArduPilotPlugin.so"), "build ardupilot_gazebo (install.sh)"))
    if external:
        checks = checks[1:]   # no ArduCopter binary needed here
    for path, hint in checks:
        if not os.path.exists(path):
            out.append(LogInfo(msg=f"[sitl.launch] WARNING: {path} missing: {hint}"))

    # ---------------------------------------------------------------- Gazebo (gazebo mode)
    if gazebo:
        gz = ["gz", "sim", "-r", "-v", arg("gz_verbosity")]
        if not truthy(context, "gui"):
            gz += ["-s", "--headless-rendering"]
        out.append(run("Gazebo", q(gz + [arg("world")])))

    # ---------------------------------------------------------------- ArduCopter SITL
    if external:
        return out + external_actions(arg, run, mavros_terminal, truthy(context, "mavros"))
    default_params = os.path.join(ap_dir, "Tools", "autotest", "default_params")
    defaults = [os.path.join(default_params, "copter.parm")]
    if gazebo:
        defaults.append(os.path.join(default_params, "gazebo-iris.parm"))
    defaults.append(os.path.join(SHARE, "config", "sitl.parm"))
    sitl = [binary, "-S", "--model", "JSON" if gazebo else "quad", "--speedup", "1", "-I0",
            "--home", arg("home"), "--defaults", ",".join(defaults)]
    if gazebo:
        sitl.append("--sim-address=127.0.0.1")
    if truthy(context, "wipe"):
        sitl.append("-w")
    out.append(run("ArduCopter SITL", f"mkdir -p {RUN_DIR} && cd {RUN_DIR}\n{q(sitl)}",
                   delay=2.0 if gazebo else 0.0))

    # ---------------------------------------------------------------- MAVProxy (optional in lite)
    wait_sitl = "until ss -ltn 2>/dev/null | grep -q ':5760 '; do sleep 0.5; done"
    if use_mavproxy:
        outs = ["udp:127.0.0.1:14550", "udp:127.0.0.1:14555"] + ([arg("gcs_out")] if arg("gcs_out") else [])
        mavproxy = ["mavproxy.py", "--master", "tcp:127.0.0.1:5760", "--sitl", "127.0.0.1:5501",
                    "--state-basedir", RUN_DIR]
        for o in outs:
            mavproxy += ["--out", o]
        if truthy(context, "console"):
            mavproxy += ["--console", "--map"]
        else:
            mavproxy += ["--non-interactive"]   # no keyboard in a launch: use QGC / the console for commands
        out.append(run("MAVProxy", "\n".join([
            '[ -f "$HOME/venv-ardupilot/bin/activate" ] && source "$HOME/venv-ardupilot/bin/activate"',
            "echo 'MAVProxy: waiting for ArduCopter SITL on tcp:5760 ...'", wait_sitl,
            q(mavproxy)]), delay=3.0 if gazebo else 1.0))

    # ---------------------------------------------------------------- MAVROS (own window)
    if truthy(context, "mavros"):
        fcu_url = arg("fcu_url") or ("udp://:14555@" if use_mavproxy else "tcp://127.0.0.1:5760")
        # without MAVProxy MAVROS is SERIAL0's only client: connect once SITL listens
        out.append(run("MAVROS", mavros_script(fcu_url, wait_tcp_port=0 if use_mavproxy else 5760),
                       delay=0.0, terminal=mavros_terminal))   # first: it waits for SITL by itself

    # ---------------------------------------------------------------- camera
    if gazebo:
        out.append(run("Camera bridge", "ros2 run ros_gz_bridge parameter_bridge "
                       "'/bc/down_camera/image@sensor_msgs/msg/Image[gz.msgs.Image' "
                       "--ros-args -r __node:=gz_camera_bridge", delay=4.0))
        out.append(run("Camera relay", "ros2 run bc_sim gz_camera_relay", delay=4.0))
    else:
        mission_yaml = "$(ros2 pkg prefix bc_bringup)/share/bc_bringup/config/mission.yaml"
        out.append(run("Synthetic camera",
                       f"ros2 run bench_flight_controller sim_camera_node --ros-args --params-file {mission_yaml}",
                       delay=2.0))
    return out


def external_actions(arg, run, mavros_terminal, start_mavros):
    """mode:=external: SITL + MAVProxy already running from sim_vehicle.py (its MAVProxy outputs to udp
    127.0.0.1:14550 and 14551; QGC takes 14550, MAVROS 14551)."""
    out = []
    if start_mavros:
        out.append(run("MAVROS", mavros_script(arg("fcu_url") or "udp://:14551@"), terminal=mavros_terminal))
    mission_yaml = "$(ros2 pkg prefix bc_bringup)/share/bc_bringup/config/mission.yaml"
    out.append(run("Synthetic camera",
                   f"ros2 run bench_flight_controller sim_camera_node --ros-args --params-file {mission_yaml}",
                   delay=2.0))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="lite",
                              description="lite: SITL quad + synthetic camera (Raspberry Pi) | gazebo: Gazebo world | "
                                          "external: your own sim_vehicle.py, only MAVROS (udp 14551) + camera"),
        DeclareLaunchArgument("mavros_terminal", default_value="auto",
                              description="MAVROS window: auto (gnome-terminal if there is a display) | "
                                          "none (this terminal) | gnome-terminal | xterm"),
        DeclareLaunchArgument("mavproxy", default_value="false",
                              description="lite: also run MAVProxy (QGC on udp 14550); always on in gazebo mode"),
        DeclareLaunchArgument("world", default_value=WORLD),
        DeclareLaunchArgument("gui", default_value="true", description="gazebo: GUI (false = server, headless)"),
        DeclareLaunchArgument("gz_verbosity", default_value="2"),
        DeclareLaunchArgument("ardupilot_dir", default_value=os.environ.get("ARDUPILOT_HOME", "~/ardupilot")),
        DeclareLaunchArgument("ardupilot_gazebo",
                              default_value=os.environ.get("ARDUPILOT_GAZEBO_HOME", "~/ardupilot_gazebo")),
        DeclareLaunchArgument("ros_setup", default_value="/opt/ros/jazzy/setup.bash"),
        DeclareLaunchArgument("ws_setup", default_value=WS_SETUP),
        DeclareLaunchArgument("home", default_value=HOME,
                              description="SITL home lat,lon,alt,heading (lite: the synthetic field's origin, "
                                          "heading 0; gazebo: the world origin)"),
        DeclareLaunchArgument("wipe", default_value="true", description="wipe SITL EEPROM (fresh parameters)"),
        DeclareLaunchArgument("console", default_value="false", description="MAVProxy console + map (wxPython)"),
        DeclareLaunchArgument("gcs_out", default_value="", description="extra MAVProxy --out, e.g. udp:192.168.1.5:14550"),
        DeclareLaunchArgument("mavros", default_value="true"),
        DeclareLaunchArgument("fcu_url", default_value="",
                              description="MAVROS link; default tcp://127.0.0.1:5760 (lite) / udp://:14555@ (MAVProxy)"),
        OpaqueFunction(function=launch_setup),
    ])
