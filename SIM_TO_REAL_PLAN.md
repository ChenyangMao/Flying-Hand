# Sim-to-Real Plan

**Status:** Custom PX4 firmware has already been flight-tested on the real vehicle. Remaining work is to bring the contact / visual-servo stack from Gazebo onto hardware.

**Scope of remaining work:**

1. Camera integration
2. Force/torque (F/T) sensor integration
3. Coordinate-frame / TF integration
4. Control-parameter updates

**Rule for team conventions below:** Names, units, and axes are fixed. Numbers (mount offsets, camera intrinsics, `hover_thrust`) are filled in after measurement.

---

## Team schedule (3 people)

| Phase | Duration | Work | Ownership |
|-------|----------|------|-----------|
| **P0 — Align** | ~0.5 day | Agree on frames, topics, mount baseline, and interfaces in this document | All three together |
| **P1 — Parallel bring-up** | 2–4 days | Camera, F/T, and ground-side TF / safety params | **A** camera · **B** F/T · **C** TF draft + geofence + `run_hw_stack` prep |
| **P2 — Frame convergence** | ~1 day | Write measured mounts into launch; verify force/image axes | C leads TF; A+B bring sensors; run `test_hw_readiness.py` |
| **P3 — Tune + staged flight** | 2–4 days | Measure `hover_thrust`, then `pose_only` → `vs_track` → `light_contact` | C safety/pilot · A watches vision · B watches force |

Dependency sketch:

```text
P0 conventions
  ├─ A: camera ────────┐
  ├─ B: F/T sensor ────┼─→ P2 TF / axis checks → P3 hover thrust → staged flight
  └─ C: TF draft + safety params + stack launch prep ─┘
```

Do **not** wait for flight to start items 1–3. Camera and F/T should publish on the bench first. Safety limits / geofence can be set on the ground; force / IBVS gains need tethered flight.

---

## Hardware Interface Convention v0.1

### 1. Coordinate frames

| Frame | Provided by | Meaning |
|-------|-------------|---------|
| `map` | MAVROS / PX4 local | World frame; local origin near takeoff |
| `base_link` | MAVROS odom `child_frame_id` (usually this) | Body frame |
| `ft_sensor` | Static TF + `/ft_data.header.frame_id` | Force-torque sensor frame |
| `camera` | Static TF | Camera optical frame (for VS / TF) |

**Body frame `base_link` (PX4/ENU convention — confirm once on the real vehicle):**

- `+x`: forward (nose)
- `+y`: left
- `+z`: up

**F/T frame `ft_sensor` (controller convention in this repo — must follow):**

- **Contact normal = `ft_sensor` `+x`** (for wall contact, measured contact force is primarily **`force.x`**)
- If the vendor driver uses a different axis: fix it with **TF rotation** or in the **bridge / driver remap**, do not change controller logic
- Units: **N / N·m** (not kgf)
- Before flight: near zero with no contact (bias allowed; tare/zero once before takeoff)

**Camera frame `camera`:**

- Image `u` right, `v` down
- Optical `+z`: out of the lens (toward the target)
- Sim launch used yaw/pitch/roll ≈ `(-π/2, 0, -π/2)` to map optical → body; **on hardware use the measured mount** and fill Section 3

**Required TF tree** (checked by `scripts/test_hw_readiness.py`):

```text
map → base_link          (MAVROS)
base_link → ft_sensor    (static, in wrench_controller_hw.launch.py)
base_link → camera       (static, same launch)
```

**TF format:** `static_transform_publisher` args

```text
x y z yaw pitch roll parent child
```

Units: **meters / radians**.

---

### 2. Topic convention

Do not change the standard names below unless the whole team updates the matching code/config together.

#### Autopilot / state

| Standard topic | Type | Notes |
|----------------|------|-------|
| `/mavros/state` | `mavros_msgs/State` | Connection, mode |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | Pose; wrench remaps this to `odometry` |

#### Camera (Person A)

| Standard topic | Type | Notes |
|----------------|------|-------|
| `/uav1/camera/color/image_raw` | `sensor_msgs/Image` | External standard name (`vs_exp.yaml` + readiness) |
| `/detected_circle` | `geometry_msgs/Vector3` | Detector output |
| `/visual_servo/debug_image` | `sensor_msgs/Image` | Debug view |
| `/visual_servo/active` | `std_msgs/Bool` | VS active flag |

If the driver publishes a different topic: **remap or relay** to the standard name, or agree as a team and update both `src/core_visual_servo/config/vs_exp.yaml` and `scripts/test_hw_readiness.py`.

#### Force-torque (Person B)

| Standard topic | Type | Notes |
|----------------|------|-------|
| `/ft_sensor/wrench` | `geometry_msgs/WrenchStamped` | Optional raw driver topic (if using `ft_sensor_bridge.py`) |
| `/ft_data` | `geometry_msgs/WrenchStamped` | **Controller subscribes to this**; `frame_id` must be `ft_sensor` |
| filtered FT topic | `WrenchStamped` | Use `scripts/test_ft_sensor.py` on the bench |

Preferred hardware paths already in-repo:

- UDP → ROS 2: `ros2 run ft_fz_udp_bridge udp_fz_bridge` with `output_topic:=ft_data`, `frame_id:=ft_sensor`, and `force_axis` / `force_sign` as needed (see README)
- Direct ROS publish from `ft_sensor/read_digital_ft_linux.py` with `--publish-ros-ft-data`
- Generic remap bridge:

```bash
python3 scripts/ft_sensor_bridge.py \
  --input-topic /ft_sensor/wrench \
  --output-topic /ft_data \
  --frame-id ft_sensor
```

#### Comms / stack launch

| Item | Convention |
|------|------------|
| `FCU_URL` | Default `/dev/ttyACM0:921600`; override via env var if different; record in team notes |
| Launch | `FCU_URL=... bash scripts/run_hw_stack.sh` |

`run_hw_stack.sh` starts MAVROS + `wrench_controller_hw` + `visual_servo_hw`. Camera and F/T drivers must already be publishing.

---

### 3. Mount baseline

**Measurement rules:**

- Origin: `base_link` origin (consistent with PX4 / flight-controller mount; if unsure, define “FC center” and agree)
- Translation: from `base_link` to sensor / camera optical center, **meters**, 3 decimals
- Attitude: yaw → pitch → roll relative to `base_link`, **radians**
- Measure twice; re-measure if difference > 5 mm / 5°

**Fill-in table** (update after P0 / measurement):

| Transform | x | y | z | yaw | pitch | roll | Write to |
|-----------|---|---|---|-----|-------|------|----------|
| `base_link` → `camera` | ___ | ___ | ___ | ___ | ___ | ___ | `wrench_controller_hw.launch.py` |
| `base_link` → `ft_sensor` | ___ | ___ | ___ | ___ | ___ | ___ | same |

**Axis acceptance tests (required):**

1. Gently push the end-effector along the **wall normal** → `/ft_data.wrench.force.x` increases with the **agreed sign**
2. **Agreement: when pressing into the wall, `force.x` is _______ (positive / negative)**
3. Camera facing the wall: target motion in the image matches vehicle pitch/yaw; optical axis roughly forward (adjust per mission)

Placeholder values in launch (sim leftovers — **not final for hardware**):

- camera `(0.12, 0, 0.12, -π/2, 0, -π/2)`
- ft `(0.15, 0, -0.10, 0, 0, 0)`

---

### 4. Interface and parameter convention

| Item | Convention |
|------|------------|
| Force units | N; torque N·m |
| Length | m |
| Primary force axis | **sensor X only** (`ft_setpoint.x` / `fx.*`) |
| Camera intrinsics | Write real calibration into `vs_exp.yaml` (`camera.fx/fy/cx/cy`); do not keep sim values |
| Target | Physical radius `target_radius` (m); default color `red`; match the real target |
| `hover_thrust` | Measure in real hover; write into `wrench_px4_hw_params.yaml` |
| geofence | Enabled on hardware by default; set `x/y/z_min/max` for your arena (`map` frame) |
| Renaming standard topics/frames | Requires full-team agreement; update readiness / launch / yaml together |

**What can be set on the ground vs in flight:**

- Ground: intrinsics, target size/color, TF geometry, geofence, thrust/tilt limits, F/T tare and units
- Short hover: `hover_thrust`
- Tethered / staged flight: force PID, contact `x_hold`, IBVS gains and rate limits

**Integration gate (before tethered flight):**

```bash
python3 scripts/test_hw_readiness.py
```

All checks OK, then:

```bash
python3 scripts/test_flight_stages.py --stage pose_only
python3 scripts/test_flight_stages.py --stage vs_track
python3 scripts/test_flight_stages.py --stage light_contact
```

---

### 5. Three-person ownership

| Person | Owns |
|--------|------|
| **A — Camera** | Image on standard topic; intrinsics; `/detected_circle` publishing |
| **B — F/T** | Driver / bridge → `/ft_data`; units in N; `force.x` sign matches wall press |
| **C — Integration / TF** | Mount table → launch; `FCU_URL` + `run_hw_stack`; geofence; run readiness |

---

### 6. P0 day checklist

- [ ] `base_link` axes agreed in person on the vehicle
- [ ] Wall-press `force.x` sign written in Section 3
- [ ] Standard topic table agreed (especially whether to keep `/uav1/...` for camera)
- [ ] `FCU_URL` recorded
- [ ] Who measures mounts / who edits launch assigned (recommend C)
- [ ] Rename rule: no solo renames of standard names

---

## Sim vs hardware quick reference

| Simulation | Hardware |
|------------|----------|
| `make px4_sitl gz_hexa_scorpion` | Board firmware already flown; power the real FCU |
| Gazebo camera / `ros-gz` bridge | Real camera driver → standard image topic |
| `scripts/gz_ft_bridge.py` | `ft_fz_udp_bridge` / `ft_sensor` scripts / `ft_sensor_bridge.py` → `/ft_data` |
| `wrench_controller_gazebo.launch.py` | `wrench_controller_hw.launch.py` |
| `visual_servo.launch.py` | `visual_servo_hw.launch.py` |
| Sim parameter overlays | `wrench_px4_hw_params.yaml` + `vs_exp.yaml` |
