## Firmware(PX4) build
```
git clone git@github.com:ChenyangMao/Flying-Hand.git
cd Flying-Hand
git submodule update --init --recursive
cd Firmware/Flying-Hand-PX4
bash ./Tools/setup/ubuntu.sh
```
## Gazebo classic simulation
```
make px4_sitl gazebo-classic_hexa_scorpion
```
## Gazebo harmonic simulation
```
make px4_sitl gz_hexa_scorpion
```
