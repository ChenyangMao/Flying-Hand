## Firmware(PX4) build
```
git clone git@github.com:ChenyangMao/Flying-Hand.git
cd Flying-Hand
git submodule update --init --recursive
cd Firmware/Flying-Hand-PX4
bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo-classic_hexa_scorpion
```
