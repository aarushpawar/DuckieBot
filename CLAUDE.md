# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workflow

After completing any code change, always commit and push to `main` automatically — do not wait for the user to ask.

**Keep this file up to date.** Whenever you change any source file — node logic, topics, parameters, dependencies, versions, or repo structure — update the relevant section(s) of `CLAUDE.md` in the same commit. Future Claude Code sessions rely on this file as the authoritative description of the codebase; stale docs cause incorrect edits.

## What this is

A **Duckietown ROS package** for the Duckiebot DB21J — a differential-drive robot running Jetson Nano 4GB with a fully containerized ROS Noetic stack. The repo builds on top of `dt-core` (the Duckietown autonomy base image) and runs inside Docker on the robot.

## Repository layout

```
DuckieBot/
├── CLAUDE.md                        # This file
├── README.md                        # User-facing setup docs
├── Dockerfile                       # arm64v8 Docker image definition
├── configurations.yaml              # Stub (unused currently)
├── dependencies-apt.txt             # APT packages (currently empty)
├── dependencies-py3.txt             # Python packages: numpy, opencv-python, PyYAML, flask, flask-cors
├── duckie_launcher.py               # HTTP proxy (port 8766) for browser-based build/run UI
├── duckie_control.html              # Browser UI for duckie_launcher.py
├── code/
│   ├── simple_lane_follow.py        # Standalone (non-DTROS) lane follower; hostname hardcoded to "nasavpns"
│   └── reference.md
├── packages/
│   ├── shape_driver/                # DTROS shape-driving node (circle/straight/stop)
│   │   ├── src/shape_driver_node.py
│   │   └── launch/shape_driver_node.launch
│   └── lane_follower_backup/        # DTROS vision-based lane follower (active default)
│       ├── src/lane_follower_node.py
│       └── launch/lane_follower_node.launch
├── launchers/
│   └── default.sh                   # Container entrypoint → launches lane_follower_backup
└── docs/                            # Sphinx documentation stubs
```

## Build and run

**Build the Docker image on the robot (replace `HOSTNAME` with robot name, e.g. `nasavpns`):**
```bash
dts devel build -f -H HOSTNAME
dts devel run -H HOSTNAME
```

**Build locally (no robot):**
```bash
dts devel build -f
```

**Run locally connected to a remote ROS master:**
```bash
dts devel run -R HOSTNAME -L default
```

**The default launcher** (`launchers/default.sh`) runs:
```bash
roslaunch lane_follower_backup lane_follower_node.launch veh:="$VEHICLE_NAME"
```
> Note: `shape_driver` is **not** the default. To use it, either modify `default.sh` or pass a different launcher.

**Robot access:**
```bash
ssh duckie@HOSTNAME.local   # password: quackquack
dts fleet discover          # find robots on network
```

**Browser-based launcher** (`duckie_launcher.py`):
```bash
python3 duckie_launcher.py  # starts HTTP proxy on port 8766
# open duckie_control.html in a browser, enter robot hostname, click Build/Run
```
Supports Windows + WSL2 (auto-prepends `wsl` to `dts` commands).

## ROS packages

### `packages/shape_driver/` — geometric shape driver

- **Version**: `NODE_VERSION = "1.3.2"` in `shape_driver_node.py`
- **Base class**: `DTROS` with `NodeType.CONTROL`
- **Default on startup**: drives two laps of a circle (`CIRCLE_LAPS=2`, `CIRCLE_LAP_DURATION=8 s`)
- **Supported commands** (case-insensitive, sent as `std_msgs/String`):
  - `circle` — v=0.2 m/s, omega=2.0 rad/s, runs `CIRCLE_LAPS` laps
  - `straight` — v=0.2 m/s, omega=0 for 3 s
  - `stop` — zero velocity
- **Publishes**: `Twist2DStamped` → `/{veh}/car_cmd_switch_node/cmd`
- **Subscribes**: `String` → `/{veh}/shape_driver_node/command`
- **Thread safety**: `current_shape` is a property backed by `_shape_lock`; safe for concurrent access from the ROS subscriber and the `run()` main-thread loop
- **Shutdown**: `on_shutdown()` sends zero velocity to stop motors

### `packages/lane_follower_backup/` — vision-based lane follower (active default)

- **Version**: `NODE_VERSION = "1.1.0"` in `lane_follower_node.py`
- **Base class**: `DTROS` with `NodeType.CONTROL`
- **Vision pipeline**:
  - Input: compressed image resized to 160×120
  - ROI: lower 55–90% of frame height (in HSV)
  - Yellow lane (attract): HSV hue 20–35°, S>100, V>100; min 50 px
  - White lane (repel): HSV H∈[0,179], S<40, V>180; min 80 px
  - Morphological denoising: 3×3 kernel, `MORPH_OPEN`
- **Control states**: `FOLLOW_YELLOW`, `WHITE_RECOVERY`, `LOST`
- **Tunable DTParams**:
  - `v_base` (default 0.15 m/s) — base forward speed
  - `k_p` (default 3.0) — proportional gain; error = 0.35 − y_center (yellow)
  - `k_repel` (default 0.6) — white-line repulsion strength
- **Adaptive speed**: `v = max(0.4×v_base, v_base×(1 − |omega|×0.15))`
- **Publishes**:
  - `Twist2DStamped` → `/{veh}/car_cmd_switch_node/cmd`
  - `CompressedImage` → `/{veh}/lane_follower_node/debug/image/compressed` (debug overlay)
- **Subscribes**:
  - `CompressedImage` ← `/{veh}/camera_node/image/compressed`
  - `Bool` ← `/{veh}/lane_follower_node/enable`
- **Flask HTTP bridge** on port 8765 (daemon thread):
  - `GET /status` → JSON: state, motor commands, centroids, FPS, params, kinematics (loaded from `/data/config/calibrations/kinematics/{veh}.yaml` at init; defaults used if file absent, `calibrated: false` flag set)
  - `POST /start`, `POST /stop` → enable/disable lane following
  - `GET /debug_image.jpg` → latest annotated JPEG frame
  - `GET /debug_stream` → MJPEG stream
- **Threading**: three locks protect shared state across the ROS callback and Flask threads:
  - `_active_lock` — guards `_lane_active` (enable/disable flag); held by `cb_enable`, `/start`, `/stop`, and `/status`
  - `_stats_lock` — guards `_last_v/omega/state`, centroids, and FPS counters; all written together in one acquisition per frame
  - `_debug_lock` — guards the latest JPEG bytes for the HTTP debug endpoints

### `code/simple_lane_follow.py` — standalone lane follower (not containerized)

- Plain `rospy` node, no DTROS lifecycle
- Hostname from `VEHICLE_NAME` env var; falls back to `"nasavpns"` if unset
- Publishes `WheelsCmdStamped` to `/{HOSTNAME}/wheels_driver_node/wheels_cmd` (manual kinematics)
- Yellow target: 0.5 normalized X (center), K_P=1.2; omega clipped to ±4.0 rad/s (`OMEGA_MAX`)
- Forward speed `v = max(0.05, V_BASE × (1 − |omega| × 0.2))` — floored to prevent backwards motion
- WHITE_RECOVERY uses centroid position only (no fitLine slope — unreliable in shallow 36 px ROI)
- Minimum pixel thresholds: MIN_YELLOW_PX=50, MIN_WHITE_PX=80
- Reads kinematics calibration from `/data/config/calibrations/kinematics/{HOSTNAME}.yaml`
- Prints ASCII progress bar telemetry to console (no ROS logging)

## Key topics

| Topic | Type | Direction | Node |
|-------|------|-----------|------|
| `/{veh}/car_cmd_switch_node/cmd` | `Twist2DStamped` | publish | shape_driver, lane_follower |
| `/{veh}/shape_driver_node/command` | `String` | subscribe | shape_driver |
| `/{veh}/camera_node/image/compressed` | `CompressedImage` | subscribe | lane_follower |
| `/{veh}/lane_follower_node/enable` | `Bool` | subscribe | lane_follower |
| `/{veh}/lane_follower_node/debug/image/compressed` | `CompressedImage` | publish | lane_follower |
| `/{veh}/wheels_driver_node/wheels_cmd` | `WheelsCmdStamped` | publish | simple_lane_follow |

`Twist2DStamped`: `v` (m/s linear), `omega` (rad/s, positive = CCW).
`WheelsCmdStamped`: `vel_left`, `vel_right` as PWM duty cycles (−1.0 to 1.0).

## Kinematics

`simple_lane_follow.py` manually converts v/omega to wheel duty cycles:
```python
v_l = (v - 0.5 * omega * baseline) / (radius * k) * (gain - trim)
v_r = (v + 0.5 * omega * baseline) / (radius * k) * (gain + trim)
```
Calibration defaults: `gain=1.0, trim=0.0, baseline=0.1, radius=0.0318, k=27.0, limit=1.0`.
Robot-specific overrides live at `/data/config/calibrations/kinematics/{HOSTNAME}.yaml`.

The DTROS nodes (`shape_driver`, `lane_follower_backup`) publish `Twist2DStamped` directly and let the downstream `kinematics_node` handle wheel conversion.

## Dependencies

- **apt**: declared in `dependencies-apt.txt` (currently none)
- **Python** (`dependencies-py3.txt`): `numpy`, `opencv-python`, `PyYAML`, `flask`, `flask-cors`
- **ROS**: `rospy`, `std_msgs`, `sensor_msgs`, `duckietown_msgs`, `cv_bridge`
- **Base image**: `duckietown/dt-core:ente-arm64v8` (arch: `arm64v8`, distro: `ente`, ROS Noetic)

## Calibration files (on robot)

| File | Purpose |
|------|---------|
| `/data/config/calibrations/kinematics/{HOSTNAME}.yaml` | gain, trim, baseline, radius, k, limit |
| `/data/config/calibrations/camera_intrinsic/{HOSTNAME}.yaml` | camera matrix + distortion coefficients |
| `/data/config/calibrations/camera_extrinsic/{HOSTNAME}.yaml` | homography for ground projection |

Re-run after any physical change to the camera mount:
```bash
dts duckiebot calibrate_intrinsics HOSTNAME
dts duckiebot calibrate_extrinsics HOSTNAME
```

## Versioning

Each ROS node has a `NODE_VERSION` constant at the top of its source file, printed on startup. **Bump it on every change** using semver:
- PATCH — bug fix, tuning tweak
- MINOR — new shape/behaviour or non-breaking feature
- MAJOR — breaking interface change

Current versions:
- `packages/shape_driver/src/shape_driver_node.py` → `1.3.2`
- `packages/lane_follower_backup/src/lane_follower_node.py` → `1.1.0`

## Sending commands manually

```bash
# Enter ROS environment on robot
dts start_gui_tools HOSTNAME

# Shape driver commands
rostopic pub /HOSTNAME/shape_driver_node/command std_msgs/String "data: 'circle'"
rostopic pub /HOSTNAME/shape_driver_node/command std_msgs/String "data: 'straight'"
rostopic pub /HOSTNAME/shape_driver_node/command std_msgs/String "data: 'stop'"

# Lane follower enable/disable
rostopic pub /HOSTNAME/lane_follower_node/enable std_msgs/Bool "data: true"
rostopic pub /HOSTNAME/lane_follower_node/enable std_msgs/Bool "data: false"

# Lane follower HTTP bridge (from any machine on the network)
curl http://HOSTNAME.local:8765/status
curl http://HOSTNAME.local:8765/debug_image.jpg
curl -X POST http://HOSTNAME.local:8765/start
curl -X POST http://HOSTNAME.local:8765/stop
```

## Common debugging

```bash
# I2C bus health (SSH into robot)
sudo i2cdetect -r -y 1

# Check camera is publishing
rostopic hz /HOSTNAME/camera_node/image/compressed

# Tune kinematics live
rosparam set /HOSTNAME/kinematics_node/trim 0.05
rosservice call /HOSTNAME/kinematics_node/save_calibration

# Container logs
docker -H HOSTNAME.local logs duckiebot-interface
```

**Motor not responding**: HUT firmware needs reflash — `dts duckiebot hut_upgrade HOSTNAME`.
**Container restart loop**: `camera_node` or `tof_node` crashing; check ribbon cable or bypass front bumper I2C multiplexer.
**I2C devices missing**: reseat the 40-pin GPIO header fully (both rows).
**Lane follower not detecting**: check `/debug_image.jpg` via HTTP bridge to see HSV masks and centroid overlays.
