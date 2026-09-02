#!/usr/bin/env bash
# Launch the full Flying Hand control stack for REAL HARDWARE.
#
# Prerequisites:
#   1. PX4 is running on the flight controller (not SITL).
#   2. MAVROS can reach the FCU at the specified FCU_URL.
#   3. The real camera driver is already running and publishing images.
#   4. The real F/T sensor driver is already running and publishing
#      geometry_msgs/WrenchStamped on /ft_data.
#   5. The workspace has been built:  colcon build
#
# Usage:
#   FCU_URL=<your_url> bash scripts/run_hw_stack.sh
#
# Default FCU_URL assumes a USB/serial connection via MAVROS.
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"

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
start_proc "wrench" bash -lc "set +u; source /opt/ros/humble/setup.bash; source '$ROOT_DIR/install/setup.bash'; set -u; ros2 launch core_wrench_controller wrench_controller_hw.launch.py"
start_proc "visual_servo" bash -lc "set +u; source /opt/ros/humble/setup.bash; source '$ROOT_DIR/install/setup.bash'; set -u; ros2 launch core_visual_servo visual_servo_hw.launch.py"

echo
echo "Hardware stack is running."
echo "  FCU URL:    $FCU_URL"
echo ""
echo "IMPORTANT: Make sure the real camera and FT sensor drivers are running."
echo "  Camera images -> topic configured in vs_exp.yaml (image_topic)"
echo "  FT sensor     -> /ft_data (WrenchStamped)"
echo ""
echo "To run the full visual-servoing contact test:"
echo "  python3 scripts/test_visual_servoing_ros2.py"
echo ""
echo "Press Ctrl+C to stop all processes."
echo

wait -n "${PIDS[@]}"
