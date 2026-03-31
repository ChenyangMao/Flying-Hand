## Clone without Firmware
```
git clone --recurse-submodules=no git@github.com:ChenyangMao/Flying-Hand.git
```
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
cd Firmware/Flying-Hand-PX4
make px4_sitl gz_hexa_scorpion
```

## Build wrench controller

```
sudo apt update
sudo apt install python3-colcon-common-extensions
sudo apt install ros-humble-mavros-msgs
sudo apt install ros-humble-mavros
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

### Start ft bridge

```
python3 scripts/gz_ft_bridge.py
```




### Run test script

```
python3 scripts/test_wall_contact_ros2.py
```

### Plot the control info
```
python3 scripts/plot_thrust_live_ros2.py
```

## Deploy controller on Jetson Nano
### Start a simulated FT sensor

```
python3 scripts/sim_ft_sensor.py
```
### On Jetson
```
source /opt/ros/humble/setup.bash
source /home/teamc/Flying-Hand/install/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
python3 scripts/sim_ft_sensor.py
```
### On PC
```
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
export ROS_LOCALHOST_ONLY=0
ros2 topic echo /ft_data
```