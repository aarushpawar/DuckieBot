# DuckieBot Shape Driver

Drives a Duckiebot DB21J in geometric shapes (circle, rectangle, triangle) via ROS commands. Built on `dt-core`, runs fully containerized via Docker on the robot.

---

## Prerequisites

- Windows with WSL2 (Ubuntu recommended)
- `dts` (Duckietown Shell) installed inside WSL
- Docker Desktop with WSL2 backend enabled
- Robot hostname (e.g. `nasavpns`) on the same WiFi as your machine

---

## Setup

### 1. Install dts (if not already done)

Open a WSL terminal and run:

```bash
pip3 install --no-cache-dir --user -U duckietown-shell
dts --set-version daffy
dts update
```

### 2. Clone the repo

```bash
git clone https://github.com/aarushpawar/DuckieBot.git
cd DuckieBot
```

---

## Build and run

### Build on the robot

```bash
dts devel build -f -H HOSTNAME
```

Replace `HOSTNAME` with your robot's name (e.g. `nasavpns`).

### Run on the robot

```bash
dts devel run -H HOSTNAME
```

---

## Sending commands

Once the container is running, send shape commands via ROS topic from a separate WSL terminal:

```bash
dts start_gui_tools HOSTNAME
```

Then inside that shell:

```bash
rostopic pub /HOSTNAME/shape_driver_node/command std_msgs/String "data: 'circle'"
rostopic pub /HOSTNAME/shape_driver_node/command std_msgs/String "data: 'rectangle'"
rostopic pub /HOSTNAME/shape_driver_node/command std_msgs/String "data: 'triangle'"
rostopic pub /HOSTNAME/shape_driver_node/command std_msgs/String "data: 'stop'"
```

---

## WSL-specific notes

- Run all `dts` and `git` commands inside WSL, not PowerShell or CMD
- Docker Desktop must be running on Windows with the WSL2 integration enabled for your distro (Docker Desktop > Settings > Resources > WSL Integration)
- If `dts fleet discover` doesn't find the robot, make sure your WSL network adapter and Windows are on the same WiFi network. Bridged networking works best; if using NAT (default), mDNS may not resolve — use the robot's IP address directly instead

---

## Robot access

```bash
ssh duckie@HOSTNAME.local   # password: quackquack
dts fleet discover          # find robots on the network
```
