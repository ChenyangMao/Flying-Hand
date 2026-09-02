#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FIRMWARE_DIR="$ROOT_DIR/Firmware/Flying-Hand-PX4"
FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"

SOURCE_ENV="source /opt/ros/humble/setup.bash && source $ROOT_DIR/install/setup.bash"

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

SESSION="flying-hand"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' already exists. Attaching..."
  exec tmux attach-session -t "$SESSION"
fi

echo "Launching Flying-Hand stack in 7 tmux windows..."

# Window 0: PX4 SITL (no ROS source needed)
tmux new-session -d -s "$SESSION" -n "PX4-SITL" \
  "cd '$FIRMWARE_DIR' && make px4_sitl gz_hexa_scorpion; exec bash"

sleep 5

# Window 1: MAVROS
tmux new-window -t "$SESSION" -n "MAVROS" \
  "bash -c '$SOURCE_ENV && cd $ROOT_DIR && ros2 launch mavros px4.launch fcu_url:=\"$FCU_URL\"; exec bash'"

sleep 2

# Window 2: Wrench Controller
tmux new-window -t "$SESSION" -n "Wrench-Ctrl" \
  "bash -c '$SOURCE_ENV && cd $ROOT_DIR && ros2 launch core_wrench_controller wrench_controller_gazebo.launch.py; exec bash'"

sleep 2

# Window 3: Visual Servo
tmux new-window -t "$SESSION" -n "Visual-Servo" \
  "bash -c '$SOURCE_ENV && cd $ROOT_DIR && ros2 launch core_visual_servo visual_servo.launch.py; exec bash'"

sleep 2

# Window 4: Gazebo FT Bridge
tmux new-window -t "$SESSION" -n "GZ-FT-Bridge" \
  "bash -c '$SOURCE_ENV && cd $ROOT_DIR && python3 scripts/gz_ft_bridge.py; exec bash'"

sleep 2

# Window 5: Thrust Plot
tmux new-window -t "$SESSION" -n "Thrust-Plot" \
  "bash -c '$SOURCE_ENV && cd $ROOT_DIR && python3 scripts/plot_thrust_live_ros2.py; exec bash'"

sleep 2

# Window 6: Visual Servoing Test
tmux new-window -t "$SESSION" -n "VS-Test" \
  "bash -c '$SOURCE_ENV && cd $ROOT_DIR && python3 scripts/test_visual_servoing_ros2.py; exec bash'"

tmux select-window -t "$SESSION:0"
echo "All 7 windows launched in tmux session '$SESSION'."
echo "Attaching now... (Use Ctrl-b then w to list windows, Ctrl-b then n/p to switch)"
exec tmux attach-session -t "$SESSION"
