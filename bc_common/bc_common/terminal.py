"""Launch helper: run one program in its own terminal window (used for MAVROS by the launch files).

The window sources ROS 2 and the workspace itself, so it works whatever environment the terminal program
passes on, and stays open after the program exits ("press Enter to close") so errors can be read.
With no display (SSH) or terminal "none" the program runs inside the launch instead, logging there.
"""
import os
import shlex
import shutil
import subprocess

# auto: the desktop's own terminal (gnome-terminal), xterm only if it is not installed. A window is tied to
# the launch: when the launch stops (Ctrl+C), the program in it is stopped and the window closes.
TERMINALS = ("gnome-terminal", "xterm")
MAVROS_ACK_TIMEOUT = 12.0  # s, MAVROS cmd plugin: longer than flight_controller's 10 s arm / takeoff wait


def pick_terminal(choice):
    """'auto' | 'none' | 'gnome-terminal' | 'xterm' -> the terminal program to use, or 'none'."""
    if choice not in ("auto", "none") + TERMINALS:
        raise RuntimeError(f"terminal must be auto, none, {' or '.join(TERMINALS)}, not {choice!r}")
    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    if choice == "auto":
        if not has_display:
            return "none"
        return next((t for t in TERMINALS if shutil.which(t)), "none")
    if choice != "none" and not shutil.which(choice):
        raise RuntimeError(f"{choice} not found: use auto or none")
    return choice


def window_command(title, script, terminal, ros_setup="", ws_setup="", env_lines=""):
    """Command list running bash `script` in a window titled `title` (terminal from pick_terminal), or
    in this process's terminal when terminal == 'none' (the last line is exec'd so it gets Ctrl+C)."""
    ros_setup = ros_setup or os.environ.get("ROS_SETUP") or f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/setup.bash"
    lines = [f"source {shlex.quote(ros_setup)}"]
    if ws_setup:
        lines.append(f"[ -f {shlex.quote(ws_setup)} ] && source {shlex.quote(ws_setup)}")
    if env_lines:
        lines.append(env_lines)
    if terminal == "none":
        body = script.split("\n")
        body[-1] = "exec " + body[-1]
        return ["bash", "-c", "\n".join(lines + body)]
    # The program runs in its own process group; a watcher stops that group (SIGINT, then SIGTERM) and
    # closes the window as soon as the launch that opened it is gone (Ctrl+C), so no MAVROS is left running.
    launch_pid = os.getpid()
    program = "\n".join(lines + [script])
    body = "\n".join([
        f"echo -ne '\\033]0;{title}\\007'",
        # job control: the program gets its own process group and does NOT ignore SIGINT (a plain `&` job
        # of a non-interactive shell would), so it can be stopped cleanly with Ctrl+C semantics
        "set -m",
        f"bash -c {shlex.quote(program)} &",
        "CHILD=$!",
        f"( while kill -0 {launch_pid} 2>/dev/null; do sleep 1; done",
        "  kill -INT -- -$CHILD 2>/dev/null; sleep 5; kill -TERM -- -$CHILD 2>/dev/null ) &",
        "WATCH=$!",
        "trap 'kill -INT -- -$CHILD 2>/dev/null' INT TERM HUP",
        "wait $CHILD; rc=$?",
        f"if ! kill -0 {launch_pid} 2>/dev/null; then exit 0; fi   # launch stopped: close the window",
        "kill $WATCH 2>/dev/null",
        f'echo; echo "[{title}] exited (code $rc). Press Enter to close."; read -r _',
    ])
    if terminal == "gnome-terminal":
        return ["gnome-terminal", "--wait", f"--title={title}", "--", "bash", "-c", body]
    return ["xterm", "-T", title, "-geometry", "110x30", "-e", "bash", "-c", body]


def mavros_script(fcu_url, gcs_url="", mavros_share="", wait_tcp_port=0):
    """`ros2 run mavros mavros_node ...` with the ArduPilot plugin set (as the launch files' MAVROS Node).
    wait_tcp_port: wait until something listens on that local TCP port first (SITL's SERIAL0)."""
    share = mavros_share or "$(ros2 pkg prefix mavros)/share/mavros"
    cmd = " ".join([
        "ros2 run mavros mavros_node --ros-args",
        f"--params-file {share}/launch/apm_pluginlists.yaml",
        f"--params-file {share}/launch/apm_config.yaml",
        f"-p fcu_url:={shlex.quote(fcu_url)}",
        # rcl rejects an empty override ("-p gcs_url:="): pass gcs_url only when set (MAVROS default: none)
        f"-p gcs_url:={shlex.quote(gcs_url)}" if gcs_url else "",
        # no "-r __node:=mavros": that renames EVERY node in the mavros_node process (router + UAS) to
        # /mavros, i.e. duplicate /mavros nodes. The UAS node is called "mavros" already.
        "-p target_system_id:=1 -p target_component_id:=1 -p fcu_protocol:=v2.0",
        # MAVROS waits 5 s for a COMMAND_ACK by default; flight_controller waits 10 s for arm / takeoff
        f"-p command_ack_timeout:={MAVROS_ACK_TIMEOUT}"]).replace("  ", " ")
    if wait_tcp_port:
        return "\n".join([f"echo 'MAVROS: waiting for the autopilot on tcp:{wait_tcp_port} ...'",
                          f"until ss -ltn 2>/dev/null | grep -q ':{wait_tcp_port} '; do sleep 0.5; done", cmd])
    return cmd


def running_mavros():
    """[(pid, command line)] of mavros_node processes already running on this machine."""
    try:
        out = subprocess.run(["pgrep", "-a", "-x", "mavros_node"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [(int(line.split(None, 1)[0]), line.split(None, 1)[1] if " " in line else "")
            for line in out.splitlines() if line.strip()]


def refuse_second_mavros():
    """Raise if a mavros_node process is already running: a second one on the same autopilot link gives
    duplicate nodes and doubled MAVLink traffic (e.g. 'unsolicited param value')."""
    found = running_mavros()
    if found:
        pids = " ".join(str(pid) for pid, _ in found)
        details = "\n".join(f"    {pid}: {cmd[:160]}" for pid, cmd in found)
        raise RuntimeError(f"MAVROS is already running (PID {pids}):\n{details}\n"
                           f"Stop it first (pkill -x mavros_node), or reuse it with start_mavros:=false / "
                           f"mavros:=false.")
