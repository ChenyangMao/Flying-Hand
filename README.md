## Introduction

Flying-Hand is an aerial robotic manipulation system for contact-based Non-Destructive Testing (NDT) on hard-to-reach industrial assets, built on a fully-actuated drone platform. It combines a multirotor vehicle with a manipulator and contact end-effector, enabling high-precision contact measurements while reducing inspection cost, downtime, and risk.

Project website: https://mrsdprojects.ri.cmu.edu/2026teamc/

This repository contains the software stack (PX4/SITL, Gazebo simulation, MAVROS, and ROS 2 control) used to support operator-guided deployment and autonomous contact interaction: an operator pilots to an area/point of interest, after which the system executes the contact procedure and streams data back to the ground station.

## Configuration Instructions

### ROS 2 environment

In each terminal (example for ROS 2 Humble):

```
source /opt/ros/humble/setup.bash
source "$HOME/Flying-Hand/install/setup.bash"
```

### MAVROS connection (SITL)

Example FCU URL used in this repo:

```
udp://:14540@127.0.0.1:14580
```

### Force/Torque (FT) input topic

- The wrench/FT input is expected as `geometry_msgs/WrenchStamped` on the `ft_data` topic.
- For hardware UDP input, the bridge exposes parameters such as `udp_port`, `output_topic`, `frame_id`, `force_axis`, and `force_sign`.

## Installation Instructions

### Clone

Clone without firmware submodules (when you do not need PX4 firmware sources):

```
git clone --recurse-submodules=no git@github.com:ChenyangMao/Flying-Hand.git
```

Clone with firmware submodules (PX4 build/simulation):

```
git clone git@github.com:ChenyangMao/Flying-Hand.git
cd Flying-Hand
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```

### Build PX4 SITL (Firmware)

Gazebo Classic:

```
cd Firmware/Flying-Hand-PX4
make px4_sitl gazebo-classic_hexa_scorpion
```

Gazebo Harmonic (gz):

```
cd Firmware/Flying-Hand-PX4
make px4_sitl gz_hexa_scorpion
```

### Build the ROS 2 wrench controller workspace

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

## Operating Instructions

### Run the wrench controller in simulation

1) Start PX4 SITL (Gazebo Harmonic):

```
make px4_sitl gz_hexa_scorpion
```

Check PX4 offboard mode:

```
listener offboard_control_mode
```

2) Start MAVROS:

```
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14580
```

3) Launch ROS 2 control stack:

```
ros2 launch core_wrench_controller wrench_controller_gazebo.launch.py
```

4) Start FT bridge:

```
python3 scripts/gz_ft_bridge.py
```

5) Run test script:

```
python3 scripts/test_wall_contact_ros2.py
```

6) Plot control info:

```
python3 scripts/plot_thrust_live_ros2.py
```

### Deploy / run on Jetson Nano

#### Force sensor input (hardware)

UDP bridge (publishes `geometry_msgs/WrenchStamped` on `ft_data`, same interface as the sim script):

```
source /opt/ros/humble/setup.bash
source /home/teamc/Flying-Hand/install/setup.bash
ros2 run ft_fz_udp_bridge udp_fz_bridge --ros-args \
  -p udp_port:=5005 -p output_topic:=ft_data -p frame_id:=ft_sensor -p force_axis:=x \
  -p force_sign:=-1.0
```

Add `-p force_sign:=-1.0` when the UDP readings are opposite in sign from what the controller expects (default `force_sign` is `1.0`).

#### Simulated FT sensor (desktop / no hardware)

```
source /opt/ros/humble/setup.bash
source /home/teamc/Flying-Hand/install/setup.bash
python3 scripts/sim_ft_sensor.py
```

#### Feed the plot topics

```
source /opt/ros/humble/setup.bash
source /home/teamc/Flying-Hand/install/setup.bash
python3 scripts/test_plot_inputs_ros2.py
```

#### Plot

```
source /opt/ros/humble/setup.bash
source /home/teamc/Flying-Hand/install/setup.bash
python3 scripts/plot_thrust_live_ros2.py
```
