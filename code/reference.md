The **Duckiebot DB21J** is a differential-drive autonomous robot built on an NVIDIA Jetson Nano 4GB, running the Duckietown “Daffy” software distribution — a fully containerized ROS Noetic stack orchestrated via Docker. This document serves as an exhaustive offline reference for every hardware component, software layer, ROS node, calibration procedure, and known failure mode in the system. Whether troubleshooting a dead I2C bus, tuning a PID controller, or understanding why the `duckiebot-interface` container keeps restarting, the answer is here.

The DB21J represents the second revision of the Jetson-based Duckiebot line, upgrading the original DB21M with a 4GB compute module, 64GB storage, chassis v2.0, HUT v3.15, and the MPU-6050 IMU (downgraded from the MPU-9250 due to the 2021–2022 chip shortage).  The entire autonomy stack — from camera driver to lane-following controller — runs inside three Docker containers communicating over ROS topics,  with all calibration data persisted to `/data/config/calibrations/` on the robot’s SD card. 

-----

## 1. Hardware overview and platform variants

### Model lineage and configuration matrix

|Model     |Compute                   |Sensors                   |Actuation                             |Storage|Chassis|HUT  |IMU     |
|----------|--------------------------|--------------------------|--------------------------------------|-------|-------|-----|--------|
|DB21M     |Jetson Nano **2GB**       |Camera, encoders, ToF, IMU|2× DC motors, 4× RGB LEDs, OLED screen|32GB   |v1.0   |v3.1 |MPU-9250|
|DB21/DB21J|Jetson Nano **2GB or 4GB**|Camera, encoders, ToF, IMU|2× DC motors, 4× RGB LEDs, OLED screen|64GB   |v2.0   |v3.15|MPU-6050|

The **DB21J** designation specifically means Jetson Nano 4GB; if a 2GB module is used, it should be flashed as **DB21M**.  The chassis v2.0 reduces assembly complexity and increases rigidity over v1.0. HUT v3.15 is backwards-compatible with v3.1 but integrates the top-button pull-up resistor that v3.1 required externally.   The platform is **FCC and CE certified** (RoHS 2.0 compliant).  

### Complete bill of materials

The mechanical chassis consists of a bottom plate, top plate, roof plate, left/right side plates, left/right side covers, a camera mount (3D-printed), motor mounts (×4), battery supports (×2), and a HUT support bracket. Fasteners include **nylon M2×8 screws (×4)**, **nylon M2.5×10 screws (×16)**, **metal M3×8 screws (×26)**, **metal M3×30 screws (×4)**, and **metal M3×12 screws (×4, for fan mounting)**. Standoffs include nylon M3×5+6mm male-female (×1), metal M2.5×18+6mm male-female (×6), and metal hexagon M3×25mm female-female (×2). Nuts: metal M3 (×24), nylon M3 (×2), nylon M2.5 (×18), nylon M2 (×4).  **Nylon screws must never be interchanged with metal screws** — they prevent electrical shorts near PCBs. 

Cables: USB Jetson power cable (×1), USB Ext5V cable (×1), USB charge cable (×1), motor cables with 6-pin connectors (×2), 4-pin I2C cables for front bumper (×2, 260mm), IMU (×1), display (×1), ToF (×1), button cable (×1, soldered), and fan cable (×1, 2-pin).

### Hardware variant notes

**Omni-wheel standoffs** may have inconsistent threading due to manufacturing tolerances; shorter screws are included as alternatives.   **Nylon bolts** are critical near PCBs and must not be substituted with metal. Some front bumper PCBs ship with a **faulty I2C multiplexer** — the recommended workaround is to bypass it entirely by connecting the ToF sensor directly to the HUT I2C port.  The **screen wiring** uses a 4-wire color-coded connector: blue (SDA↔SDA), yellow (SCL↔SCL), black (GND↔GND), red (3.3V↔VCC).  Wheels may slip off motor axles due to tolerances; set screws are provided but should not be overtightened. 

-----

## 2. Jetson Nano internals — GPIO, I2C, power, and cooling

### Jetson Nano 4GB Developer Kit specifications

The compute module features a **128-core NVIDIA Maxwell GPU**, a **quad-core ARM Cortex-A57 CPU at 1.43 GHz**,  and **4GB 64-bit LPDDR4 RAM** with 25.6 GB/s bandwidth. It provides 2× MIPI CSI-2 camera connectors (the 2GB variant has only 1×), HDMI output, Gigabit Ethernet, USB 3.0 (×1), USB 2.0 (×2), and a USB 2.0 Micro-B port. The board measures 100×80×29mm.  Power can come via Micro-USB (default) or DC barrel jack (requires J48 jumper);  on the Duckiebot, power flows from the Duckiebattery through the HUT via the 40-pin header’s 5V pins.

### 40-pin expansion header (J41) — the HUT interface

The expansion header is the sole electrical interface between the Jetson Nano and the HUT board. All 40 pins must be fully seated — both rows — or I2C communication fails silently.  

|Pin(s)                      |Function     |Notes                                                                                           |
|----------------------------|-------------|------------------------------------------------------------------------------------------------|
|3 (SDA1), 5 (SCL1)          |**I2C Bus 1**|Primary bus for ALL Duckiebot peripherals. Level-shifted 1.8V→3.3V. Pulled up with 2.2kΩ to 3.3V|
|27 (SDA0), 28 (SCL0)        |I2C Bus 0    |Secondary bus, not used by default Duckiebot peripherals                                        |
|8 (TX), 10 (RX)             |UART         |Available but not used in stock configuration                                                   |
|32, 33                      |Hardware PWM |2 channels available                                                                            |
|1, 17                       |3.3V power   |                                                                                                |
|2, 4                        |**5V power** |Main power path from HUT to Jetson                                                              |
|6, 9, 14, 20, 25, 30, 34, 39|Ground       |                                                                                                |

Non-I2C GPIO signals pass through **TXB0108RGYR level shifters** (1.8V internal ↔ 3.3V at header). 

### I2C device address map (Bus 1)

|Device                     |Address  |Function                        |
|---------------------------|---------|--------------------------------|
|HUT ATMEGA328P             |**0x40** |LED control, motor command relay|
|Motor driver (PCA9685-like)|**0x60** |PWM motor control               |
|VL53L0X ToF                |**0x29** |Front distance ranging          |
|IMU (MPU-6050/9250)        |**0x68** |Accelerometer + gyroscope       |
|OLED Display (SSD1306)     |**0x3C** |128×32 status screen            |
|Front bumper multiplexer   |**~0x70**|I2C routing (known issue source)|

Debug the bus directly via SSH: `sudo i2cdetect -r -y 1` 

### Fan mounting on the unthreaded heatsink

The cooling fan attaches to the Jetson Nano heatsink using **4× metal M3×12 screws** that must **self-tap into unthreaded holes** in the heatsink aluminum. This requires firm pressure on first insertion.  The fan cable connects to the HUT fan header: red wire to the **5V pin** for full speed, or the **3.3V pin** for reduced speed and noise.  Fan orientation: cable should point toward the back-right of the assembled Duckiebot.

### Power distribution architecture

```
Duckiebattery
  ├── USB OUT-1 ("muscles") → HUT → Motors, LEDs, sensors, display
  ├── USB OUT-2 ("brains")  → HUT → Jetson Nano (via 40-pin 5V pins)
  └── USB Serial            → Jetson Nano (battery telemetry, 1Hz JSON)
```

The Duckiebattery is a **10,000 mAh @ 3.7V** smart power bank  with pass-through charging,   soft shutdown support, and real-time telemetry (state of charge, cell temperature in °K, cell voltage in mV, current in mA, time to empty, cycle count). Charging: **5V 2A Micro-USB** on the HUT.   Full charge takes **~5 hours**.   Minimum firmware **v1.2.2** is required for battery code updates;  **v2.0.0+** for the proper shutdown protocol.  

-----

## 3. Software and OS stack — Daffy, Docker, and dt-shell

### Docker image hierarchy

The entire Duckietown software stack is containerized.  The inheritance chain from base to application:

```
ubuntu:18.04
  └── dt-base-environment     (ROS Noetic, numpy, scipy, OpenCV, smbus, i2c libs)
       └── dt-commons          (Duckietown libraries, hostname resolution, env vars)
            └── dt-ros-commons (DTROS framework, duckietown_msgs, ros_http_api_node)
                 ├── dt-duckiebot-interface  (hardware drivers)
                 ├── dt-car-interface        (kinematics, joystick mapping)
                 └── dt-core                 (autonomy: lane following, intersection handling)
```

### Three core containers running on every Duckiebot

|Container                       |Image                                            |Purpose                                            |Starts            |
|--------------------------------|-------------------------------------------------|---------------------------------------------------|------------------|
|`duckiebot-interface`           |`duckietown/dt-duckiebot-interface:daffy-arm64v8`|Camera, motors, LEDs, encoders, ToF, IMU drivers   |Auto on boot      |
|`car-interface`                 |`duckietown/dt-car-interface:daffy-arm64v8`      |Kinematics, joystick mapping, car command switching|Auto on boot      |
|Demo container (e.g., `dt-core`)|`duckietown/dt-core:daffy-arm64v8`               |Lane following, perception, control, FSM           |Manually for demos|

Additional infrastructure containers: **dt-duckiebot-dashboard** (web dashboard on port 80), **dt-rosbridge-websocket** (ROS-to-web bridge), and **Portainer** (Docker management UI on port 9000). 

### DTROS framework

All Duckietown ROS nodes inherit from the **DTROS** base class (defined in `dt-ros-commons`), which provides:  **DTPublisher/DTSubscriber** with `active` flag and `anybody_listening()`/`anybody_publishing()` methods; **DTParam** for dynamically tunable parameters with min/max bounds, type checking, and update callbacks; a **`~switch` service** to deactivate/reactivate all pubs/subs;  built-in **diagnostics** (NodeHealth enum: UNKNOWN, STARTING, HEALTHY, WARNING, ERROR, FATAL); **profiling** context managers; and a common graceful shutdown procedure. 

**NodeType enum values**: GENERIC, DRIVER, PERCEPTION, CONTROL, PLANNING, LOCALIZATION, MAPPING, SWARM, BEHAVIOR, VISUALIZATION, INFRASTRUCTURE, COMMUNICATION, DIAGNOSTICS, DEBUG. 

### Duckietown Shell (dts) — command reference

Install: `pip3 install --no-cache-dir --user -U duckietown-shell`,  then `dts --set-version daffy && dts update`.

**Core operational commands:**

|Command                                                                                               |Description                     |
|------------------------------------------------------------------------------------------------------|--------------------------------|
|`dts init_sd_card --hostname NAME --type duckiebot --configuration DB21J --wifi SSID:PSK --country US`|Flash SD card                   |
|`dts fleet discover`                                                                                  |Discover robots on local network|
|`dts duckiebot update HOSTNAME`                                                                       |OTA software update             |
|`dts duckiebot shutdown HOSTNAME`                                                                     |Soft shutdown                   |
|`dts duckiebot reboot HOSTNAME`                                                                       |Reboot                          |
|`dts duckiebot keyboard_control HOSTNAME`                                                             |Keyboard teleoperation          |
|`dts duckiebot calibrate_intrinsics HOSTNAME`                                                         |Camera intrinsic calibration    |
|`dts duckiebot calibrate_extrinsics HOSTNAME`                                                         |Camera extrinsic calibration    |
|`dts duckiebot demo --demo_name NAME --duckiebot_name HOST --package_name PKG`                        |Run a demo                      |
|`dts duckiebot hut_upgrade HOSTNAME`                                                                  |Flash HUT microcontroller       |
|`dts duckiebot battery upgrade HOSTNAME`                                                              |Update battery firmware         |
|`dts start_gui_tools HOSTNAME`                                                                        |Launch ROS GUI tools container  |
|`dts diagnostics HOSTNAME`                                                                            |Run diagnostics                 |

**Development commands** (`dts devel`): `build -f` (local build), `build -f -H ROBOT` (remote build), `run` (local run), `run -H ROBOT` (remote run),  `run -R ROBOT -L LAUNCHER` (local run connected to remote ROS master).  

-----

## 4. ROS architecture — every node, topic, service, and parameter

All topics follow the namespace pattern: `/<HOSTNAME>/<node_name>/<topic>`. The tilde (`~`) notation below means relative to the node namespace.

### dt-duckiebot-interface nodes (hardware drivers)

#### CameraNode (`camera_node`)

|Direction    |Topic                                 |Message Type                 |
|-------------|--------------------------------------|-----------------------------|
|**Publishes**|`/<HOST>/camera_node/image/compressed`|`sensor_msgs/CompressedImage`|
|**Publishes**|`/<HOST>/camera_node/camera_info`     |`sensor_msgs/CameraInfo`     |

**Service**: `~set_camera_info` — saves calibration to `/data/config/calibrations/camera_intrinsic/<HOST>.yaml` 

|Parameter       |Type |Default |Description           |
|----------------|-----|--------|----------------------|
|`~framerate`    |float|30.0    |Camera FPS            |
|`~res_w`        |int  |640     |Image width (px)      |
|`~res_h`        |int  |480     |Image height (px)     |
|`~exposure_mode`|str  |`sports`|PiCamera exposure mode|

Camera node is a **required process** — if it dies, the entire `duckiebot-interface` container restarts. 

#### WheelsDriverNode (`wheels_driver_node`)

|Direction     |Topic                                           |Message Type                      |
|--------------|------------------------------------------------|----------------------------------|
|**Subscribes**|`/<HOST>/wheels_driver_node/wheels_cmd`         |`duckietown_msgs/WheelsCmdStamped`|
|**Publishes** |`/<HOST>/wheels_driver_node/wheels_cmd_executed`|`duckietown_msgs/WheelsCmdStamped`|

The `WheelsCmdStamped` message contains `Header header`, `float32 vel_left`, `float32 vel_right`   — these represent **PWM duty cycles** (range approximately -1.0 to 1.0), not physical velocities.  Internally, the node uses the **Adafruit_MotorHAT** library to control DC motors via I2C at address 0x60.

#### WheelEncoderNode (`left_wheel_encoder_node` / `right_wheel_encoder_node`)

Two instances run — one per wheel. 

|Direction     |Topic                 |Message Type                         |
|--------------|----------------------|-------------------------------------|
|**Subscribes**|`~wheels_cmd_executed`|`duckietown_msgs/WheelsCmdStamped`   |
|**Publishes** |`~tick`               |`duckietown_msgs/WheelEncoderStamped`|

`WheelEncoderStamped`: `Header header`, `int32 data` (cumulative tick count), `int32 resolution` (ticks per revolution), `uint8 type` (ENCODER_TYPE_INCREMENTAL).

**TF published**: `<HOST>/<name>_wheel_axis` → `<HOST>/<name>_wheel` (rotation as quaternion).

|Parameter           |Type |Description                                     |
|--------------------|-----|------------------------------------------------|
|`~gpio`             |int  |GPIO pin number for encoder input               |
|`~resolution`       |int  |Encoder ticks per revolution (nominally **135**)|
|`~configuration`    |str  |`left` or `right`                               |
|`~publish_frequency`|float|Publishing rate, 1–100 Hz                       |

Calibration file: `/data/config/calibrations/encoder/<configuration>/<HOST>.yaml`

#### LEDEmitterNode (`led_emitter_node`)

Controls **4 addressable RGB LEDs** (2 front bumper, 2 back bumper)  via I2C commands to the HUT ATMEGA328P at address 0x40 (registers starting at command 6+). Provides ROS services for changing LED patterns, colors, and blinking modes.  LED states: **white** = powered on and booted; **random colors** = powering on; **blue** = robot off, charger connected. 

#### ToFNode (`tof_node`)

|Direction    |Topic                   |Message Type       |
|-------------|------------------------|-------------------|
|**Publishes**|`/<HOST>/tof_node/range`|`sensor_msgs/Range`|

Drives the **VL53L0X** sensor over I2C at address 0x29.  Range: ~50mm to 1.2m (default), up to 2m in long-range mode.

### dt-car-interface nodes (mid-level control)

#### KinematicsNode (`kinematics_node`)

Performs **inverse kinematics** (Twist2DStamped → WheelsCmdStamped) and forward kinematics (velocity estimation).  

|Direction     |Topic                               |Message Type                      |
|--------------|------------------------------------|----------------------------------|
|**Subscribes**|`/<HOST>/kinematics_node/car_cmd`   |`duckietown_msgs/Twist2DStamped`  |
|**Publishes** |`/<HOST>/kinematics_node/wheels_cmd`|`duckietown_msgs/WheelsCmdStamped`|
|**Publishes** |`/<HOST>/kinematics_node/velocity`  |`duckietown_msgs/Twist2DStamped`  |

**Service**: `~save_calibration` (`std_srvs/Empty`) — saves to `/data/config/calibrations/kinematics/<HOST>.yaml` 

`Twist2DStamped`: `Header header`, `float32 v` (linear velocity, m/s), `float32 omega` (angular velocity, rad/s; positive = counter-clockwise). 

|Parameter   |Default|Min |Max |Description                |
|------------|-------|----|----|---------------------------|
|`~gain`     |1.0    |0.1 |1.0 |Velocity scaling factor    |
|`~trim`     |0.0    |-1.0|1.0 |Left/right motor offset    |
|`~baseline` |0.1    |0.05|0.2 |Wheel-to-wheel distance (m)|
|`~radius`   |0.0318 |0.01|0.1 |Wheel radius (m)           |
|`~k`        |27.0   |—   |—   |Motor constant             |
|`~limit`    |1.0    |0.1 |1.0 |Max motor command          |
|`~v_max`    |1.0    |0.01|2.0 |Max linear velocity input  |
|`~omega_max`|8.0    |1.0 |10.0|Max angular velocity input |

**Inverse kinematics equations**:

```
omega_r = (v + 0.5 * omega * baseline) / radius
omega_l = (v - 0.5 * omega * baseline) / radius
```

#### CarCmdSwitchNode (`car_cmd_switch_node`)

Multiplexes between command sources (joystick, lane controller, intersection controller).   Subscribes to multiple `~cmd` inputs from various sources; publishes the selected command to kinematics_node.

#### VelocityToPoseNode (`velocity_to_pose_node`)

Integrates wheel commands to produce dead-reckoning pose estimates relative to the starting position. 

#### JoyMapperNode (`joy_mapper_node`)

|Direction     |Topic                            |Message Type                    |
|--------------|---------------------------------|--------------------------------|
|**Subscribes**|`/<HOST>/joy_mapper_node/joy`    |`sensor_msgs/Joy`               |
|**Publishes** |`/<HOST>/joy_mapper_node/car_cmd`|`duckietown_msgs/Twist2DStamped`|
|**Publishes** |Various button-triggered topics  |`duckietown_msgs/BoolStamped`   |

### dt-core nodes (autonomy stack — lane following pipeline)

#### AntiInstagramNode (`anti_instagram_node`)

Computes color transformation to normalize lighting conditions.

|Direction     |Topic                          |Message Type                             |
|--------------|-------------------------------|-----------------------------------------|
|**Subscribes**|`~camera_node/image/compressed`|`sensor_msgs/CompressedImage`            |
|**Publishes** |`~thresholds`                  |`duckietown_msgs/AntiInstagramThresholds`|

Parameters: `~ai_interval` (float, default 10s), `~fancyGeom`, `~n_centers`, `~blur`, `~resize`. 

#### LineDetectorNode (`line_detector_node`)

Detects white, yellow, and red line segments using HSV color filtering and Canny/Hough transforms.  

|Direction     |Topic                            |Message Type                             |
|--------------|---------------------------------|-----------------------------------------|
|**Subscribes**|`~camera_node/image/compressed`  |`sensor_msgs/CompressedImage`            |
|**Subscribes**|`~anti_instagram_node/thresholds`|`duckietown_msgs/AntiInstagramThresholds`|
|**Publishes** |`~segment_list`                  |`duckietown_msgs/SegmentList`            |
|**Publishes** |`~debug/segments/compressed`     |`sensor_msgs/CompressedImage`            |
|**Publishes** |`~debug/edges/compressed`        |`sensor_msgs/CompressedImage`            |
|**Publishes** |`~debug/maps/compressed`         |`sensor_msgs/CompressedImage`            |

Parameters: `~img_size` (default [120, 160]), `~top_cutoff` (default 40 rows), `~colors` (HSV ranges for white/yellow/red), `~line_detector_parameters` (Canny thresholds, Hough params). 

#### GroundProjectionNode (`ground_projection_node`)

Projects line segments from image space to ground plane using homography (H) from extrinsic calibration:  **P_ground = H × P_camera**. 

|Direction     |Topic                                      |Message Type                 |
|--------------|-------------------------------------------|-----------------------------|
|**Subscribes**|`~lineseglist_in`                          |`duckietown_msgs/SegmentList`|
|**Subscribes**|`~camera_info`                             |`sensor_msgs/CameraInfo`     |
|**Publishes** |`~lineseglist_out`                         |`duckietown_msgs/SegmentList`|
|**Publishes** |`~debug/ground_projection_image/compressed`|`sensor_msgs/CompressedImage`|

#### LaneFilterNode (`lane_filter_node`)

Estimates lane pose (lateral deviation `d` and heading deviation `phi`) using a histogram grid filter on ground-projected segments. 

|Direction     |Topic          |Message Type                    |
|--------------|---------------|--------------------------------|
|**Subscribes**|`~segment_list`|`duckietown_msgs/SegmentList`   |
|**Subscribes**|`~car_cmd`     |`duckietown_msgs/Twist2DStamped`|
|**Publishes** |`~lane_pose`   |`duckietown_msgs/LanePose`      |
|**Publishes** |`~belief_img`  |`sensor_msgs/CompressedImage`   |
|**Publishes** |`~in_lane`     |`duckietown_msgs/BoolStamped`   |

`LanePose` message: `d` (lateral offset), `phi` (heading error), plus associated uncertainties.

#### LaneControllerNode (`lane_controller_node`)

PID controller converting lane pose estimates into car commands. 

|Direction     |Topic                                 |Message Type                      |
|--------------|--------------------------------------|----------------------------------|
|**Subscribes**|`~lane_pose`                          |`duckietown_msgs/LanePose`        |
|**Subscribes**|`~intersection_navigation_pose`       |`duckietown_msgs/LanePose`        |
|**Subscribes**|`~wheels_cmd`                         |`duckietown_msgs/WheelsCmdStamped`|
|**Subscribes**|`~stop_line_reading`                  |`duckietown_msgs/StopLineReading` |
|**Publishes** |`/<HOST>/lane_controller_node/car_cmd`|`duckietown_msgs/Twist2DStamped`  |

|Parameter  |Default|Range   |Description                         |
|-----------|-------|--------|------------------------------------|
|`~v_bar`   |—      |0.0–5.0 |Nominal velocity (m/s)              |
|`~k_d`     |—      |-100–100|Proportional gain, lateral deviation|
|`~k_theta` |—      |-100–100|Proportional gain, heading deviation|
|`~k_Id`    |—      |-100–100|Integral gain, lateral deviation    |
|`~k_Itheta`|—      |-100–100|Integral gain, heading deviation    |

Common tuning overrides: `rosparam set /<HOST>/lane_controller_node/k_d -45` and `rosparam set /<HOST>/lane_controller_node/k_theta -11`. 

#### StopLineFilterNode (`stop_line_filter_node`)

|Direction     |Topic               |Message Type                     |
|--------------|--------------------|---------------------------------|
|**Subscribes**|`~lane_pose`        |`duckietown_msgs/LanePose`       |
|**Publishes** |`~stop_line_reading`|`duckietown_msgs/StopLineReading`|
|**Publishes** |`~at_stop_line`     |`duckietown_msgs/BoolStamped`    |

Parameters: `~stop_distance`, `~min_segs`, `~off_time`, `~max_y`.

#### FSMNode (`fsm_node`)

Finite state machine controlling robot behavior modes. Publishes `~mode` (`duckietown_msgs/FSMState`). States include: `JOYSTICK_CONTROL`, `LANE_FOLLOWING`, `INTERSECTION_CONTROL`, `NORMAL_JOYSTICK_CONTROL`.

### Complete lane following data flow

```
Camera HW → camera_node/image/compressed
                    ↓
            anti_instagram_node → thresholds
                    ↓
            line_detector_node → segment_list
                    ↓
            ground_projection_node → lineseglist_out
                    ↓
            lane_filter_node → lane_pose
                    ↓
            lane_controller_node → car_cmd
                    ↓
            car_cmd_switch_node → (selected cmd)
                    ↓
            kinematics_node → wheels_cmd
                    ↓
            wheels_driver_node → Motor HW
                    ↓
            wheel_encoder_nodes → tick (odometry feedback)
```

### Key message types (duckietown_msgs)

|Message                  |Fields                                                           |Primary usage          |
|-------------------------|-----------------------------------------------------------------|-----------------------|
|`WheelsCmdStamped`       |`header`, `vel_left` (float32), `vel_right` (float32)            |Motor commands         |
|`Twist2DStamped`         |`header`, `v` (float32), `omega` (float32)                       |Car-level commands     |
|`LanePose`               |`d`, `phi`, `d_ref`, `phi_ref`, `status`, `in_lane`              |Lane estimation output |
|`SegmentList`            |`header`, `segments[]` (Segment)                                 |Line detection pipeline|
|`WheelEncoderStamped`    |`header`, `data` (int32), `resolution` (int32), `type` (uint8)   |Encoder readings       |
|`BoolStamped`            |`header`, `data` (bool)                                          |FSM events, switches   |
|`FSMState`               |`header`, `state` (string)                                       |Behavior mode          |
|`StopLineReading`        |`header`, `stop_line_detected`, `at_stop_line`, `stop_line_point`|Stop line detection    |
|`AntiInstagramThresholds`|`low[3]`, `high[3]`                                              |Color normalization    |

-----

## 5. Sensor suite — camera, ToF, IMU, and encoders

### Camera: Sony IMX219-160 wide-angle

The camera module uses a **Sony IMX219 8MP sensor** with a **160° diagonal FOV** fisheye lens. CMOS size: 1/4 inch. Aperture: F/2.35. Focal length: 3.15mm. Distortion: <14.3%.  Operating temperature: -20°C to 70°C.  Power consumption: 660mW typical, 1.48W max. The focus is **manually adjustable** by rotating the lens barrel — factory glue may need to be broken on first adjustment.   Connection is via a **15-pin MIPI CSI-2 flat flex cable** to the Jetson Nano camera port. The camera cable may need a twist to align pins with the connector; this is normal. 

**Default capture settings**: 640×480 at 30 FPS, `sports` exposure mode.   The Dashboard throttles the stream to **8 FPS**  by default (adjustable in the Properties tab). At full resolution, the camera publishes `sensor_msgs/CompressedImage` on `/<HOST>/camera_node/image/compressed`.

**Known camera issues**: If the camera is not detected, `camera_node` crashes and the entire `duckiebot-interface` container restarts in a loop (GitHub Issue #32). Verify the ribbon cable connection and ensure the correct end is inserted with pins aligned. Camera latency in lane following can be reduced by using compressed image transport and smaller image sizes (160×120 after resize in the line detector).

### Time-of-Flight sensor: STMicroelectronics VL53L0X

A **VCSEL 940nm laser-based** ranging sensor. Range: ~50mm to 1.2m (default mode), up to 2m in long-range mode.  Accuracy: 3–12% depending on ambient light, surface reflectivity, and distance. I2C address: **0x29**. Mounted front-facing, typically on the front bumper board. The ROS driver (`tof_node`) publishes `sensor_msgs/Range` on `/<HOST>/tof_node/range`.

**Critical wiring note**: The ToF can connect either through the front bumper’s I2C multiplexer (channel CH4 on boards with 2 camera ports; CH6 otherwise) or **directly to the HUT I2C port** via a long cable. **Direct HUT connection is strongly recommended** to bypass the known multiplexer issue that causes I2C bus failures. A known bug (GitHub Issue #51) causes `tof_node.py` to crash looking for `/dev/i2c-13`, which may not exist on all hardware configurations.

### IMU: InvenSense MPU-6050 (DB21J) / MPU-9250 (DB21M)

The DB21J ships with the **MPU-6050** (6-DOF: 3-axis accelerometer + 3-axis gyroscope, no magnetometer), replacing the MPU-9250 (9-DOF, with magnetometer) used on the original DB21M. I2C address: **0x68** (or 0x69 depending on the AD0 pin state). Connected to the HUT via a 4-pin I2C cable.

### Wheel encoders: Hall effect (integrated in DG01D-E motors)

Each motor integrates a **Hall effect encoder** with 2 quadrature channels (A and B) enabling direction detection. The encoder produces **3 pulses per motor shaft revolution** (6 magnetic poles → 3 pole pairs). With the **48:1 gear reduction**, this yields approximately **135 ticks per wheel revolution** (as documented by Duckietown, counting single-edge of one channel). Angular resolution: ~2.67° per tick. The encoder connects via the 6-pin motor cable: Motor+, Motor-, Encoder VCC, Encoder A, Encoder B, Encoder GND.

-----

## 6. Actuators — motors, LEDs, and display

### DC motors: Dagu DG01D-E (×2)

Brushed DC motors with metal gearbox (48:1 ratio). Voltage range: 3V–9V (nominal: 4.5V). No-load speed: ~90±10 RPM at 4.5V. Stall torque: 0.15 N·m. Stall current: 0.75A at 6V. Dimensions: 80×22.4×25.8mm. **Motor control path**: ROS `WheelsCmdStamped` → `wheels_driver_node` → Adafruit_MotorHAT library → I2C to HUT at address 0x60 → PWM to motors. The differential drive configuration independently controls left and right motors.

**Motor cable orientation matters**: if the Duckiebot drives backwards when commanded forward, swap the motor cables on the HUT connectors.

### Addressable RGB LEDs (×4)

Four individually addressable RGB LEDs — 2 on the front bumper PCB, 2 on the back bumper PCB. Driven by the **HUT ATMEGA328P** firmware via I2C commands from the Jetson Nano (address 0x40, registers starting at command 6+). Control via `led_emitter_node` ROS services. **Warning**: do not unplug/replug bumper wires while power is on — this can damage the bumper PCBs. A known issue (GitHub Issue #5) causes **occasional LED flickering** with no official fix; this may be related to I2C bus contention.

### OLED display: SSD1306 128×32

Monochrome I2C display at address **0x3C**. Wiring from HUT: blue=SDA, yellow=SCL, black=GND, red=3.3V→VCC. Displays battery state, hostname, IP address, and status information. The display is programmable through the `display_driver_node`.

### Top shutdown button

Non-latching momentary button with M12 nut, mounted on the top plate. **Hold for 5 seconds then release** for soft shutdown (button blinks for 3s → LEDs off → compute/fan shutdown in ~10s). HUT v3.15 has the required pull-up resistor integrated; HUT v3.1 requires an external resistor.

-----

## 7. Networking — WiFi, SSH, mDNS, and dashboard

### WiFi configuration

WiFi credentials are set during SD card flashing:

```bash
dts init_sd_card --wifi "network1:password1,network2:password2" --country US
```

Post-flash WiFi changes on Jetson Nano: edit `/etc/wpa_supplicant.conf` (on the `APP` partition if editing from a laptop, or via SSH). Hostname resolution uses **Avahi/mDNS** — the robot is accessible at `<HOSTNAME>.local`.

### SSH access

```bash
ssh duckie@HOSTNAME.local
# Password: quackquack
```

The `init_sd_card` procedure auto-generates an SSH config entry at `~/.ssh/config` using the key `/home/user/.ssh/DT18_key_00`. StrictHostKeyChecking is disabled by default.

### Dashboard and Portainer

The dashboard is available at `http://HOSTNAME.local/` (built on the **\compose** framework). Key pages: **Robot > Info** (CPU usage, temperature, firmware version), **Mission Control** (camera stream, motor speed plots, joystick), **Components** (hardware health status for HUT, bumpers, ToF, IMU, screen), **File Manager** (browse calibration files), and **Portainer** (Docker container management). Portainer is also directly accessible at `http://HOSTNAME.local:9000`.

### Network troubleshooting essentials

If `HOSTNAME.local` doesn’t resolve: verify laptop and robot on the same WiFi; restart Avahi on the robot (`sudo systemctl restart avahi-daemon`); check `/etc/avahi/avahi-daemon.conf` for hostname conflicts. On Linux, ensure `mdns` (not `mdns_minimal`) is in `/etc/nsswitch.conf`. For networks that block Docker Hub, push images over SSH: `docker save duckietown/IMAGE | ssh -C HOSTNAME docker load`. For enterprise WiFi that won’t work with `wpa_supplicant`, use an Ethernet bridge: connect the robot via cable, set laptop’s wired connection to “Shared to other computers.”

-----

## 8. Calibration procedures — camera, wheels, and kinematics

### Camera intrinsic calibration (step by step)

**Materials**: 8×6 inner corners calibration checkerboard, printed on A3 at **exactly 100% scale** (no scaling). Square side: **0.031m (3.1cm)** — measure this; incorrect size produces bad calibration.

```bash
dts duckiebot calibrate_intrinsics HOSTNAME
```

1. A GUI window opens on the laptop (if black, resize the window)
1. **Set camera focus**: rotate the mechanical lens ring until the x/y axis labels are readable — **do not change focus after this point**
1. Move the checkerboard in front of the camera; colored lines overlay when the full board is detected
1. Fill all four bars (X, Y, Size, Skew) by moving the checkerboard: left/right, up/down, closer/farther, tilted
1. When all bars are green, click **CALIBRATE** (screen dims; wait for computation)
1. Click **COMMIT** to save (not “SAVE”)

**Output**: `/data/config/calibrations/camera_intrinsic/<HOSTNAME>.yaml`

### Camera extrinsic calibration

Align the Duckiebot’s wheel axis with the y-axis of the checkerboard. Ensure no background clutter and evenly bright overhead lighting.

```bash
dts duckiebot calibrate_extrinsics HOSTNAME
```

The calibration computes a **homography matrix H** that maps pixel coordinates to ground-plane coordinates. A validation window shows the top-down projected view.

**Output**: `/data/config/calibrations/camera_extrinsic/<HOSTNAME>.yaml`

**Post-calibration rules**: never adjust camera focus after intrinsic calibration; never use a lens cover; re-calibrate if the camera holder is touched or the robot is shipped.

### Wheel and kinematics calibration

Start keyboard control and open a GUI tools terminal:

```bash
dts duckiebot keyboard_control HOSTNAME
dts start_gui_tools HOSTNAME
```

**Trim calibration** (corrects left/right drift):

```bash
rosparam set /<HOSTNAME>/kinematics_node/trim <VALUE>
```

Place a tape line on the floor (~2m). Drive forward. Target: **<10cm drift over 2 meters**. Drifted left → decrease trim (e.g., -0.1). Drifted right → increase trim (e.g., +0.1). Iterate.

**Gain calibration** (overall speed):

```bash
rosparam set /<HOSTNAME>/kinematics_node/gain <VALUE>
```

**Save calibration**:

```bash
rosservice call /<HOSTNAME>/kinematics_node/save_calibration
```

**Output**: `/data/config/calibrations/kinematics/<HOSTNAME>.yaml`

Example YAML content:

```yaml
baseline: 0.1
calibration_time: '2024-01-15-10-30-00'
gain: 1.0
k: 27.0
limit: 1.0
radius: 0.0318
trim: -0.02
```

-----

## 9. Demo workflows — lane following and beyond

### Lane following (primary supported demo)

**Prerequisites**: camera calibrated (intrinsic + extrinsic), wheels calibrated, Duckietown city loop with white/yellow lane markings, white diffused lighting, no intersections in loop.

**Launch**:

```bash
dts duckiebot demo --demo_name lane_following --duckiebot_name HOSTNAME --package_name duckietown_demos
```

Wait ~1 minute for containers to start (check Portainer).

**Activate autonomy**:

```bash
dts duckiebot keyboard_control HOSTNAME
```

Press `a` to start lane following, `s` to stop. On joystick: R1 = start, L1 = stop.

**Debug visualization** (from `dts start_gui_tools HOSTNAME`):

```bash
rqt_image_view
# Select: /<HOST>/line_detector_node/debug/segments/compressed
# Or: /<HOST>/ground_projection_node/debug/ground_projection_image/compressed
```

**Processing pipeline**: Image capture → Anti-Instagram color normalization → Line detection (HSV filtering + Canny edges + Hough lines) → Ground projection (homography) → Lane filter (histogram grid: estimates `d` and `phi`) → PID lane controller → Kinematics → Motor actuation.

### Lane following with obstacles (legacy demo)

Requires manually starting the base containers first, then launching `multi_lane_following`:

```bash
dts duckiebot demo --demo_name multi_lane_following --duckiebot_name HOSTNAME --package_name duckietown_demos
```

### General demo launch pattern

```bash
dts duckiebot demo --demo_name DEMO_NAME --duckiebot_name HOSTNAME --package_name PACKAGE_NAME
```

Available packages and demos are defined in the `duckietown_demos` package within `dt-core`. The FSM node manages state transitions between `JOYSTICK_CONTROL`, `LANE_FOLLOWING`, and `INTERSECTION_CONTROL`.

-----

## 10. Common issues and fixes — the definitive troubleshooting guide

### Power and battery failures

**Robot won’t power on**: verify all cable connections; ensure the battery is charged; press the **side button on the Duckiebattery** (NOT the top button). **Boot loops / repeated reboots**: battery too low — unplug everything except the charging cable, charge for 5+ hours. **First boot corrupted**: if shutdown/reboot occurred during first boot, the SD card must be re-flashed entirely. **Battery shows “NoBT” on display**: single-press the battery button, then retry `dts duckiebot battery upgrade`. **Battery enters protection mode**: only use **5V 2A** chargers; wrong voltage triggers protection.

### HUT and motor issues

**Motors don’t respond despite software commands**: the most common cause is **HUT firmware needing reflash**. Check Dashboard > Robot > Components — a red alert on HUT confirms this. Fix: `dts duckiebot hut_upgrade HOSTNAME` or manually via SSH (clone `fw-device-hut`, copy correct `avrdude.conf` for Jetson Nano, run `make fuses` → expect “Fuses OK (E:FF, H:DF, L:E2)”, then `make clean && make` → expect “2220 bytes of flash written”, reboot). **Jerky motor operation**: caused by the front bumper I2C multiplexer interfering with the bus — bypass it by connecting the ToF directly to the HUT. **Robot drives backwards**: motor cables are swapped on HUT connectors.

### I2C bus and sensor failures

**Multiple I2C devices missing** (ToF, screen, IMU all gone): verify both rows of the 40-pin GPIO header are fully seated. Debug systematically: power off, unplug one I2C device at a time, reboot — if removing one device restores others, that device (or its cable) is faulty. Run `sudo i2cdetect -r -y 1` over SSH to check which addresses respond. **ToF not detected**: connect directly to HUT, bypassing the front bumper multiplexer. **Screen blank**: often a cascade from the same multiplexer issue — the ToF bypass fix typically resolves this too.

### Camera problems

**`duckiebot-interface` restart loop**: usually caused by `camera_node` crashing when the camera isn’t detected (GitHub Issue #32). Check ribbon cable. **Camera out of focus**: rotate the lens barrel (may need to break factory glue). **High latency**: increase Dashboard stream frequency; use compressed transport; reduce image size.

### Software and Docker container issues

**Container keeps restarting**: check logs with `docker logs duckiebot-interface`. Common causes: camera node crash, ToF node crash (looking for non-existent `/dev/i2c-13`), or numpy/OpenBLAS crash (fix: set env `OPENBLAS_NUM_THREADS=1`). **All containers stopped after `dts duckiebot evaluate`**: this command kills running containers (GitHub Issue #305) — manually restart or re-initialize. **Wrong model flashed**: DB21J = 4GB, DB21M = 2GB — re-flash with correct `--configuration`.

### Networking issues

**Cannot resolve `HOSTNAME.local`**: ensure same WiFi network; restart Avahi (`sudo systemctl restart avahi-daemon`); on Linux, use `mdns` (not `mdns_minimal`) in `/etc/nsswitch.conf`. **Bot not found in `dts fleet discover`**: wait up to 5 minutes for first boot; check router admin for connected devices. **Enterprise WiFi fails**: use an Ethernet bridge or a dedicated WPA2-PSK router.

### Calibration failures

**Intrinsic calibration crashes with exit code -11**: ensure camera is publishing (`rostopic hz /<HOST>/camera_node/image/compressed`). **Extrinsic calibration: `findChessBoardCorners` failed**: reposition checkerboard to be fully within camera FOV, remove background clutter. **Calibration doesn’t persist after reboot**: verify containers use `-v /data:/data` volume mount (GitHub Issue #92). **Wrong checkerboard square size**: must be exactly 3.1cm — measure, don’t trust printer scaling.

### Open GitHub issues (dt-duckiebot-interface)

|#  |Issue                                            |Impact                             |
|---|-------------------------------------------------|-----------------------------------|
|#71|PWM signal generation clarification for DB21M    |Motor control confusion            |
|#53|**HATv3 mixes PWM and GPIO motor control**       |Motor/LED addressing conflicts     |
|#51|**tof_node.py tries to open non-existent i2c-13**|Container restart loop             |
|#32|**Camera crash causes container restart loop**   |System instability                 |
|#5 |**LEDs flicker occasionally**                    |Unknown root cause                 |
|#1 |Hardcoded radius limit in wheels driver          |Turning radius artificially limited|

-----

## 11. Docker container reference — what runs where

### Container-to-node mapping

**`duckiebot-interface`** (auto-starts on boot):

- `camera_node` — image capture and publishing
- `wheels_driver_node` — motor PWM control
- `left_wheel_encoder_node` — left encoder ticks
- `right_wheel_encoder_node` — right encoder ticks
- `led_emitter_node` — LED pattern control
- `tof_node` — ToF distance measurement
- `display_driver_node` — OLED screen updates
- `imu_node` — IMU data publishing

**`car-interface`** (auto-starts on boot):

- `kinematics_node` — inverse/forward kinematics
- `car_cmd_switch_node` — command source selection
- `joy_mapper_node` — joystick-to-command mapping
- `velocity_to_pose_node` — dead-reckoning pose estimation

**`dt-core`** (manual launch for demos):

- `anti_instagram_node` — color normalization
- `line_detector_node` — line segment detection
- `ground_projection_node` — image-to-ground projection
- `lane_filter_node` — lane pose estimation
- `lane_controller_node` — PID control
- `stop_line_filter_node` — stop line detection
- `fsm_node` — finite state machine

**Infrastructure containers** (always running):

- `dt-duckiebot-dashboard` — web dashboard (port 80)
- `dt-rosbridge-websocket` — ROS web bridge
- Portainer — Docker management UI (port 9000)
- `ros_http_api_node` — ROS environment exposed as HTTP API

### Configuration modes

- **Teleoperation only**: `duckiebot-interface` + `car-interface`
- **Lane following demo**: all three core containers
- **Simulation**: `car-interface` + `dt-core` + simulator interface (replaces `duckiebot-interface`)

-----

## 12. Key file paths and configuration files

### On-robot directory structure

```
/data/config/
├── calibrations/
│   ├── camera_intrinsic/
│   │   ├── <HOSTNAME>.yaml        ← Camera matrix, distortion coefficients
│   │   └── default.yaml           ← Fallback defaults
│   ├── camera_extrinsic/
│   │   ├── <HOSTNAME>.yaml        ← Homography matrix for ground projection
│   │   └── default.yaml
│   ├── kinematics/
│   │   ├── <HOSTNAME>.yaml        ← gain, trim, baseline, radius, k, limit
│   │   └── default.yaml
│   └── encoder/
│       └── <configuration>/
│           └── <HOSTNAME>.yaml    ← Encoder resolution
├── permissions/                    ← Access control (chmod 777 if write errors)
```

### Network and system files

|File                      |Location                      |Purpose                            |
|--------------------------|------------------------------|-----------------------------------|
|WiFi config (Jetson)      |`/etc/wpa_supplicant.conf`    |WPA supplicant configuration       |
|Avahi config              |`/etc/avahi/avahi-daemon.conf`|mDNS hostname resolution           |
|SSH key (laptop)          |`~/.ssh/DT18_key_00`          |Auto-generated during SD card flash|
|SSH config (laptop)       |`~/.ssh/config`               |Auto-generated Host entry for robot|
|avrdude config (HUT flash)|`/etc/avrdude.conf`           |Microcontroller programming config |

### Docker image tags

|Image                              |Tag pattern    |Architecture         |
|-----------------------------------|---------------|---------------------|
|`duckietown/dt-duckiebot-interface`|`daffy-arm64v8`|Jetson Nano (aarch64)|
|`duckietown/dt-car-interface`      |`daffy-arm64v8`|Jetson Nano (aarch64)|
|`duckietown/dt-core`               |`daffy-arm64v8`|Jetson Nano (aarch64)|
|`duckietown/dt-duckiebot-dashboard`|`daffy`        |Web dashboard        |
|`duckietown/dt-rosbridge-websocket`|`daffy`        |ROS bridge           |

### Source repositories

|Repository                         |Branch       |Contents                            |
|-----------------------------------|-------------|------------------------------------|
|`duckietown/dt-duckiebot-interface`|`daffy`      |Hardware driver nodes               |
|`duckietown/dt-car-interface`      |`daffy`      |Kinematics, joystick mapping        |
|`duckietown/dt-core`               |`daffy`      |Autonomy stack (perception, control)|
|`duckietown/fw-device-hut`         |`jetson-nano`|HUT ATMEGA328P firmware             |
|`duckietown/duckietown-shell`      |`daffy`      |`dts` command-line tool             |

### Quick-reference commands for daily use

```bash
# Discovery and status
dts fleet discover
ping HOSTNAME.local
ssh duckie@HOSTNAME.local                    # password: quackquack

# Container inspection
docker -H HOSTNAME.local ps                  # list running containers
docker -H HOSTNAME.local logs duckiebot-interface  # driver container logs

# ROS inspection (from dts start_gui_tools HOSTNAME)
rostopic list
rostopic hz /<HOST>/camera_node/image/compressed
rostopic echo /<HOST>/tof_node/range
rosparam list
rosparam get /<HOST>/kinematics_node/trim

# I2C bus inspection (via SSH)
sudo i2cdetect -r -y 1                      # scan all devices on Bus 1

# Power management
dts duckiebot shutdown HOSTNAME              # soft shutdown
dts duckiebot reboot HOSTNAME                # reboot
# Top button: hold 5s → release → shutdown
```

-----

## Conclusion: navigating the DB21J as a complete system

The Duckiebot DB21J is a tightly integrated system where hardware, firmware, and containerized software must align precisely. Three insights emerge from this comprehensive analysis. First, **the I2C bus is the single most critical failure domain** — the HUT, ToF, IMU, display, and bumper LEDs all share I2C Bus 1, and a faulty front bumper multiplexer can cascade failures across seemingly unrelated components. The first diagnostic step for any sensor issue should be `sudo i2cdetect -r -y 1`. Second, **the HUT microcontroller firmware is the most overlooked failure point** — new DB21J units almost always require a HUT reflash before motors will respond, yet nothing in the boot process flags this explicitly. Third, **calibration file persistence depends on Docker volume mounts** — the `/data` directory must be mounted into every container that reads or writes calibration data, and older versions of certain `dts` commands failed to do this (GitHub Issue #92). Understanding these architectural bottlenecks transforms debugging from guesswork into systematic diagnosis.