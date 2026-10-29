#!/usr/bin/env bash
# ArduCopter SITL (JSON backend -> ardupilot_gazebo's ArduPilotPlugin) + MAVProxy, started by sitl.launch.py.
#
#   run_sitl.sh <ardupilot_dir> <home lat,lon,alt,hdg> <wipe true|false> <extra .parm> <console true|false> <out>...
#
# Same as `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON` but without sim_vehicle's terminal windows:
#   arducopter  -S --model JSON --defaults copter.parm,gazebo-iris.parm,<extra>  (SERIAL0 = tcp:5760)
#   mavproxy.py --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --out <out> ...
# MAVProxy comes from ~/venv-ardupilot (created by ArduPilot's install-prereqs) when it exists.
set -u
AP_DIR="${1/#\~/$HOME}"; HOME_LOC="$2"; WIPE="$3"; EXTRA_PARM="$4"; CONSOLE="$5"; shift 5
OUTS=("$@")

BIN="$AP_DIR/build/sitl/bin/arducopter"
DEFAULTS="$AP_DIR/Tools/autotest/default_params/copter.parm,$AP_DIR/Tools/autotest/default_params/gazebo-iris.parm"
[ -n "$EXTRA_PARM" ] && DEFAULTS="$DEFAULTS,$EXTRA_PARM"

if [ ! -x "$BIN" ]; then
  echo "[run_sitl] $BIN not found. Build it: cd $AP_DIR && ./waf configure --board sitl && ./waf copter" >&2
  exit 1
fi
if [ -f "$HOME/venv-ardupilot/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "$HOME/venv-ardupilot/bin/activate"
fi
if ! command -v mavproxy.py >/dev/null 2>&1; then
  echo "[run_sitl] mavproxy.py not found (pip install MAVProxy in ~/venv-ardupilot, see install.sh)" >&2
  exit 1
fi

RUN_DIR="$HOME/.ros/bc_sitl"     # eeprom.bin, logs/, terrain/
mkdir -p "$RUN_DIR"
cd "$RUN_DIR" || exit 1
WIPE_ARG=()
[ "$WIPE" = "true" ] && WIPE_ARG=(-w)

echo "[run_sitl] arducopter JSON, home $HOME_LOC, defaults $DEFAULTS, wipe=$WIPE (log: $RUN_DIR/arducopter.log)"
"$BIN" -S --model JSON --speedup 1 --sim-address=127.0.0.1 -I0 --home "$HOME_LOC" \
       --defaults "$DEFAULTS" "${WIPE_ARG[@]}" > "$RUN_DIR/arducopter.log" 2>&1 &
AP_PID=$!
MP_PID=""
cleanup() {
  trap - EXIT INT TERM
  [ -n "$MP_PID" ] && kill -INT "$MP_PID" 2>/dev/null
  kill "$AP_PID" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 100); do      # SERIAL0 listens on tcp 5760 once the binary is up
  if ! kill -0 "$AP_PID" 2>/dev/null; then
    echo "[run_sitl] arducopter exited:" >&2; tail -n 20 "$RUN_DIR/arducopter.log" >&2; exit 1
  fi
  ss -ltn 2>/dev/null | grep -q ':5760 ' && break   # listening (no probe connection)
  sleep 0.2
done

MP_ARGS=(--master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501 --state-basedir "$RUN_DIR")
for o in "${OUTS[@]}"; do MP_ARGS+=(--out "$o"); done
if [ "$CONSOLE" = "true" ]; then
  MP_ARGS+=(--console --map)
else
  MP_ARGS+=(--non-interactive)
fi
echo "[run_sitl] mavproxy.py ${MP_ARGS[*]}"
mavproxy.py "${MP_ARGS[@]}" &      # background + wait: a SIGINT from ros2 launch runs cleanup at once
MP_PID=$!
wait -n "$AP_PID" "$MP_PID"
echo "[run_sitl] arducopter or MAVProxy exited (arducopter log: $RUN_DIR/arducopter.log)"
