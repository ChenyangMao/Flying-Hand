#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u

python3 "$ROOT_DIR/scripts/test_plot_inputs_ros2.py"
python3 "$ROOT_DIR/scripts/plot_thrust_live_ros2.py"

python3 "$ROOT_DIR/ft_sensor/read_digital_ft_linux.py" --port /dev/ttyUSB1 --cal-json FT33454_cal.json --tare-samples 50 \
  --publish-ros-ft-data --ros-force-axis x --ros-force-sign -1.0 --samples 0 --baud 1250000

