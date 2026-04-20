# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workflow

After completing any code change, always commit and push to `main` automatically — do not wait for the user to ask.

## What this is

A **Duckietown ROS package** for the Duckiebot DB21J — a differential-drive robot running Jetson Nano 4GB with a fully containerized ROS Noetic stack. The repo builds on top of `dt-core` (the Duckietown autonomy base image) and runs inside Docker on the robot.

## Build and run

**Build the Docker image locally:**
```bash
dts devel build -f
```

**Build and run on robot (replace `HOSTNAME` with robot name, e.g. `nasavpns`):**
```bash
dts devel build -f -H HOSTNAME
dts devel run -H HOSTNAME
```

**Run locally connected to a remote ROS master:**
```bash
dts devel run -R HOSTNAME -L default
```

The default launcher (`launchers/default.sh`) runs:
```bash
roslaunch shape_driver shape_driver_node.launch veh:="$VEHICLE_NAME"
```

**Robot access:**
```bash
ssh duckie@HOSTNAME.local   # password: quackquack
dts fleet discover          # find robots on network
```

## ROS packages

### `packages/shape_driver/` (active)
Drives the Duckiebot in geometric shapes (circle, rectangle, triangle). Uses `DTROS` base class with `NodeType.CONTROL`. Publishes `Twist2DStamped` to `/{veh}/car_cmd_switch_node/cmd`. Accepts shape commands via a `std_msgs/String` subscriber on `/{veh}/shape_driver_node/command`. Defaults to running rectangle on startup.

### `packages/lane_follower_backup/` (backup)
A full DTROS lane-following node using computer vision. Subscribes to `/{veh}/camera_node/image/compressed`, publishes `WheelsCmdStamped` to `/{veh}/wheels_driver_node/wheels_cmd`. Implements its own kinematics (reads from `/data/config/calibrations/kinematics/{veh}.yaml`). Uses HSV color filtering on a lower ROI (60–90% of frame height) to detect yellow (attract) and white (repel) lane markings.

### `code/simple_lane_follow.py`
Standalone (non-DTROS) version of the lane follower. Has the robot hostname hardcoded as `nasavpns`. Functionally identical to the backup node but runs as a plain ROS node without the DTROS lifecycle machinery.

## Key topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/{veh}/car_cmd_switch_node/cmd` | `Twist2DStamped` | publish (shape_driver) |
| `/{veh}/shape_driver_node/command` | `String` | subscribe (shape_driver) |
| `/{veh}/wheels_driver_node/wheels_cmd` | `WheelsCmdStamped` | publish (lane_follower) |
| `/{veh}/camera_node/image/compressed` | `CompressedImage` | subscribe (lane_follower) |

`Twist2DStamped`: `v` (m/s linear), `omega` (rad/s, positive = CCW).  
`WheelsCmdStamped`: `vel_left`, `vel_right` as PWM duty cycles (−1.0 to 1.0).

## Kinematics

Both lane follower implementations manually compute wheel commands:
```python
v_l = (v - 0.5 * omega * baseline) / (radius * k) * (gain - trim)
v_r = (v + 0.5 * omega * baseline) / (radius * k) * (gain + trim)
```
Calibration defaults: `gain=1.0, trim=0.0, baseline=0.1, radius=0.0318, k=27.0, limit=1.0`. The robot-specific file at `/data/config/calibrations/kinematics/{HOSTNAME}.yaml` overrides these.

## Dependencies

- **apt**: declared in `dependencies-apt.txt`
- **Python**: `numpy`, `opencv-python`, `PyYAML` (`dependencies-py3.txt`)
- **ROS**: `rospy`, `std_msgs`, `duckietown_msgs`
- **Base image**: `duckietown/dt-core:ente-arm64v8` (arch: `arm64v8`, distro: `ente`)

## Calibration files (on robot)

| File | Purpose |
|------|---------|
| `/data/config/calibrations/kinematics/{HOSTNAME}.yaml` | gain, trim, baseline, radius, k, limit |
| `/data/config/calibrations/camera_intrinsic/{HOSTNAME}.yaml` | camera matrix + distortion |
| `/data/config/calibrations/camera_extrinsic/{HOSTNAME}.yaml` | homography for ground projection |

Re-run calibration after any physical changes to the camera mount:
```bash
dts duckiebot calibrate_intrinsics HOSTNAME
dts duckiebot calibrate_extrinsics HOSTNAME
```

## Versioning

`NODE_VERSION` at the top of `packages/shape_driver/src/shape_driver_node.py` is printed on startup. **Bump it on every change** using semver (`MAJOR.MINOR.PATCH`):
- PATCH — bug fix, tuning tweak
- MINOR — new shape or behaviour
- MAJOR — breaking interface change

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

**Motor not responding**: HUT firmware likely needs reflash — `dts duckiebot hut_upgrade HOSTNAME`.  
**Container restart loop**: `camera_node` or `tof_node` crashing; check ribbon cable or bypass front bumper I2C multiplexer.  
**I2C devices missing**: reseat the 40-pin GPIO header fully (both rows).
