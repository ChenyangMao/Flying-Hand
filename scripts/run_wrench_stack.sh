#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRMWARE_DIR="$ROOT_DIR/Firmware/Flying-Hand-PX4"

FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"

if [[ ! -d "$FIRMWARE_DIR" ]]; then
  echo "Firmware directory not found: $FIRMWARE_DIR" >&2
  exit 1
fi

if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "ROS 2 Humble setup not found: /opt/ros/humble/setup.bash" >&2
  exit 1
fi

if [[ ! -f "$ROOT_DIR/install/setup.bash" ]]; then
  echo "Workspace setup not found: $ROOT_DIR/install/setup.bash" >&2
  echo "Run 'colcon build' first." >&2
  exit 1
fi

source_env() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$ROOT_DIR/install/setup.bash"
  set -u
}

source_env

PIDS=()

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo
    echo "Stopping stack..."
    kill "${PIDS[@]}" 2>/dev/null || true
    wait "${PIDS[@]}" 2>/dev/null || true
  fi

  exit "$exit_code"
}

trap cleanup EXIT INT TERM

start_proc() {
  local name="$1"
  shift

  echo "[$name] starting"
  "$@" &
  local pid=$!
  PIDS+=("$pid")
  echo "[$name] pid=$pid"
}

start_proc "mavros" bash -lc "set +u; source /opt/ros/humble/setup.bash; source '$ROOT_DIR/install/setup.bash'; set -u; ros2 launch mavros px4.launch fcu_url:='$FCU_URL'"
start_proc "wrench" bash -lc "set +u; source /opt/ros/humble/setup.bash; source '$ROOT_DIR/install/setup.bash'; set -u; ros2 launch core_wrench_controller wrench_controller_gazebo.launch.py"
start_proc "ft_bridge" bash -lc "set +u; source /opt/ros/humble/setup.bash; source '$ROOT_DIR/install/setup.bash'; set -u; python3 '$ROOT_DIR/scripts/gz_ft_bridge.py'"

echo
echo "Stack is running."
echo "Start PX4 separately before this script, for example:"
echo "  cd '$FIRMWARE_DIR' && make px4_sitl gz_hexa_scorpion"
echo "  FCU URL:    $FCU_URL"
echo "Press Ctrl+C to stop all processes."
echo

wait -n "${PIDS[@]}"
