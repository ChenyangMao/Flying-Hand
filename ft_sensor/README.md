**Windows**

```text
python read_digital_ft_com4.py --port COM4 --cal-json FT33454_cal.json --tare-samples 50 --publish-udp 172.26.42.200:5005 --samples 0
```

```text
python read_digital_ft_com4.py --apply-gains-from-json FT33454_cal.json --cal-json FT33454_cal.json --tare-samples 50 --demo-format --samples 200
```

**Linux** (default `--port` is `/dev/ttyUSB0`; change to match your device)

```text
python read_digital_ft_com4.py --port /dev/ttyUSB0 --cal-json FT33454_cal.json --tare-samples 50 --publish-udp 172.26.42.200:5005 --samples 0
```

```text
python read_digital_ft_com4.py --port /dev/ttyUSB0 --apply-gains-from-json FT33454_cal.json --cal-json FT33454_cal.json --tare-samples 50 --demo-format --samples 200
```

**Jetson / ROS 2 (same machine as wrench):** publish calibrated Fz on `ft_data` (no UDP bridge). Source ROS first, then e.g.:

```text
python3 read_digital_ft_linux.py --port /dev/ttyUSB0 --cal-json FT33454_cal.json --tare-samples 50 \
  --publish-ros-ft-data --ros-force-axis x --ros-force-sign -1.0 --samples 0
```

Use `--ros-force-axis z` if your controller expects Fz on `force.z`. Omit `--ros-force-sign` or use `1.0` for no flip.
