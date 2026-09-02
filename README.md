## Firmware(PX4) build

```
git clone git@github.com:ChenyangMao/Flying-Hand.git
cd Flying-Hand
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

## Gazebo classic simulation

```
cd Firmware/Flying-Hand-PX4
make px4_sitl gazebo-classic_hexa_scorpion
```

## Gazebo harmonic simulation

```

make px4_sitl gz_hexa_scorpion
```

## Build Visual Servo

```
sudo apt install ros-humble-ros-gzharmonic-bridge ros-humble-ros-gzharmonic-image
```

## Build wrench controller

```
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-mavros-msgs
sudo apt install ros-humble-mavros
sudo apt install ros-humble-ros-gz-bridge
source /opt/ros/humble/setup.bash
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
cd Flying-Hand
colcon build
```

## Test wrench controller

For every terminal:

```
source /opt/ros/humble/setup.bash
source $HOME/Flying-Hand/install/setup.bash
```

### Start PX4

```
make px4_sitl gz_hexa_scorpion
```

Check PX4 offboard mode:

```
listener offboard_control_mode
```

### Start MAVROS

```
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
```

### Launch ROS2 control stack

```
ros2 launch core_wrench_controller wrench_controller_gazebo.launch.py
```

### Launch Visual Servo stack

```
ros2 launch core_visual_servo visual_servo.launch.py
```

### Start ft bridge

```
python3 scripts/gz_ft_bridge.py
```

### Run Wrench/Pos test script

```
python3 scripts/test_wall_contact_ros2.py
```

### Or Run Wrench/VS test script

```
python3 scripts/test_visual_servoing_ros2.py
```

### Plot the control info

```
python3 scripts/plot_thrust_live_ros2.py
```

### Config environment for visual sevoing
```
sudo apt update
sudo apt install ros-humble-ros-gzharmonic
```


### TODO: Hardware Readiness improvements and tests
sim-to-hardware deployment — hardened detection, safety watchdogs, hw launch stack

- color_circle_detection: add circularity check, temporal consistency gate, min contour area
- IBVS: EMA smoothing on detections/depth, velocity rate limiter, state reset on target loss
- wrench_controller_node: sensor-dropout watchdog (odom/FT), auto-disable wrench on stale data
- test_visual_servoing_ros2: hard force abort (15N), odom/FT dropout abort during flight
- New hw config: wrench_px4_hw_params.yaml (geofence on, conservative PID, x_hold PD)
- New hw launches: wrench_controller_hw.launch.py, visual_servo_hw.launch.py
- New hw stack script: run_hw_stack.sh (MAVROS + wrench + VS for real drone)
- New validation scripts: test_hw_readiness, test_ft_sensor, test_camera_detection, test_flight_stages
- New FT bridge: ft_sensor_bridge.py for real hardware sensor
- Updated vs_exp.yaml with full IBVS + detection params for hardware
- Default show_window=false in all configs
