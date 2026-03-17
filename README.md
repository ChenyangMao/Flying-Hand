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

## Test Wrench Controller
For every terminal:
```
source /opt/ros/humble/setup.bash
source /home/ubuntu/Flying-Hand/install/setup.bash
```
### Start PX4
```
make px4_sitl gz_hexa_scorpion
```
### Start MAVROS
```
ros2 launch mavros px4.launch
```
### Launch ROS2 control stack
One terminal for each one:
```
ros2 launch core_drone_interface drone_interface_node.launch.py drone_interface:=PX4Interface
ros2 launch core_pose_controller pose_controller_gazebo.launch.py
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

