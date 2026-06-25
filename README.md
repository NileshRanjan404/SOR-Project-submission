# 🚀 ROS2 Mars Rover Simulation

> A **ROS 2 Jazzy Jalisco** simulation of a 6-wheeled rocker-bogie rover in a Martian terrain — featuring custom Ackermann-inspired steering kinematics, LX-16A servo motor control, EKF-based localization, Nav2 autonomous navigation, and a fully textured Mars world in **Gazebo Harmonic**.

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [How the Rover Works](#how-the-rover-works)
- [Control Pipeline — cmd_vel → Motors](#control-pipeline--cmd_vel--motors)
- [Steering Kinematics](#steering-kinematics)
- [Wheel Odometry Node](#wheel-odometry-node)
- [Localization — EKF Sensor Fusion](#localization--ekf-sensor-fusion)
- [Navigation — Nav2 Stack](#navigation--nav2-stack)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation & Build](#installation--build)
- [Launch Command](#launch-command)
- [ROS 2 Topics Reference](#ros-2-topics-reference)
- [Teleop & Manual Control](#teleop--manual-control)
- [Troubleshooting](#troubleshooting)

---

## Overview

This repository is a ROS 2 Jazzy port of a Sawppy-inspired Mars rover, built for the **Gazebo Harmonic** simulator. It provides a full autonomous navigation stack operating inside a custom Mars terrain world complete with craters, rock formations, and reddish sand textures.

**Stack:**

| Component | Version |
|---|---|
| ROS 2 | **Jazzy Jalisco** |
| Ubuntu | **24.04 LTS (Noble Numbat)** |
| Gazebo | **Harmonic (gz sim)** |
| Python | 3.12 |

**Key capabilities:**
- 6-wheel rocker-bogie simulation in a textured Mars world
- Custom Ackermann-inspired kinematics via `vel_parser_node` and `motors_command_parser_node`
- LX-16A bus servo controller (Python + C++ implementations)
- Wheel odometry from joint states + EKF fusion with IMU heading
- Nav2 autonomous navigation (SmacHybrid / SmacLattice planners, RPP / TEB controllers)
- Hokuyo LiDAR + ASUS Xtion RGB-D camera sensors
- Custom `MotorsCommand` ROS 2 message bridging kinematics to `ros2_control`

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                              ROS 2 Node Graph                            │
│                                                                          │
│  ┌──────────────┐  /cmd_vel   ┌──────────────────┐  /motors_command     │
│  │  Nav2 Stack  │ ──────────► │  vel_parser_node  │ ──────────────────► │
│  │  or Teleop   │             │  (Ackermann IK)   │                     │
│  └──────────────┘             └──────────────────┘                      │
│                                                       ┌────────────────┐ │
│                                                       │ motors_command │ │
│                                                       │ _parser_node   │ │
│                                                       │ (Gazebo only)  │ │
│                                                       └───────┬────────┘ │
│                                          velocity_controller/ │           │
│                                          position_controller/ │           │
│                                                               ▼           │
│  ┌──────────────┐  /joint_states   ┌──────────────────────────────────┐ │
│  │  Gazebo      │ ───────────────► │  odometry_node                   │ │
│  │  Harmonic    │  /imu            │  → /wheel_odom                   │ │
│  │  (gz sim)    │ ───────────────► │  ekf_filter_node                 │ │
│  │              │  /scan           │  → /odometry/filtered            │ │
│  └──────────────┘                  └─────────────────┬────────────────┘ │
│         │                                            │                   │
│         │                                            ▼                   │
│         │                          ┌──────────────────────────────────┐ │
│         └────────────────────────► │  Nav2 (SLAM + path planning)     │ │
│               /scan, /camera/*     │  bt_navigator, planner, control  │ │
│                                    └──────────────────────────────────┘ │
│                                                      │                   │
│                                                      ▼                   │
│                                             ┌─────────────┐             │
│                                             │   RViz2     │             │
│                                             └─────────────┘             │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## How the Rover Works

The rover is a **6-wheeled rocker-bogie platform** — the same suspension concept used by NASA's Curiosity and Perseverance rovers. The suspension passively distributes weight across all 6 wheels even on highly uneven terrain, keeping them all in ground contact without active control.

### Wheel & Joint Layout

```
     [front_left]────────[front_right]      ← corner joints (steering) +
          |                    |               wheel joints (drive)
     [mid_left]           [mid_right]       ← wheel joints only (no steering)
          |                    |
     [back_left]─────────[back_right]       ← corner joints (steering) +
                                               wheel joints (drive)
```

From the URDF (`rover.urdf.xacro`), the rover has **10 joints** in `ros2_control`:

| Joint | Type | Purpose |
|---|---|---|
| `front_left_corner_joint` | Position | Steer front-left wheel |
| `front_right_corner_joint` | Position | Steer front-right wheel |
| `back_left_corner_joint` | Position | Steer rear-left wheel |
| `back_right_corner_joint` | Position | Steer rear-right wheel |
| `front_left_wheel_joint` | Velocity | Drive front-left wheel |
| `front_right_wheel_joint` | Velocity | Drive front-right wheel |
| `mid_left_wheel_joint` | Velocity | Drive middle-left wheel |
| `mid_right_wheel_joint` | Velocity | Drive middle-right wheel |
| `back_left_wheel_joint` | Velocity | Drive rear-left wheel |
| `back_right_wheel_joint` | Velocity | Drive rear-right wheel |

These are commanded through `ros2_control` via two controllers:
- **`position_controller`** → corner joints (steering angles)
- **`velocity_controller`** → wheel joints (drive speeds)

---

## Control Pipeline — cmd_vel → Motors

The full pipeline from a `geometry_msgs/Twist` command to physical joint motion:

```
/cmd_vel (Twist: linear.x, angular.z)
          │
          ▼
  vel_parser_node                        ← rover_motor_controller_cpp pkg
  (Ackermann kinematics)
          │
          │  rover_msgs/msg/MotorsCommand
          │    drive_motor[6]  ← per-wheel int32 speeds (-600 to +600)
          │    corner_motor[4] ← per-corner int32 encoder ticks (250–750)
          ▼
  motors_command_parser_node             ← rover_gazebo pkg (sim only)
  (tick/speed → ros2_control commands)
          │
          ├──► /position_controller/commands  (Float64MultiArray, 4 values)
          │        └──► corner joints → steering angles (rad)
          │
          └──► /velocity_controller/commands  (Float64MultiArray, 6 values)
                   └──► wheel joints → drive velocities (rad/s)
```

### Custom Message: `MotorsCommand`

Defined in `rover_msgs/msg/MotorsCommand.msg`:

```
int32[] drive_motor    # 6 values: [FL, FR, ML, MR, BL, BR] speed (-600 to +600)
int32[] corner_motor   # 4 values: [FL, FR, BL, BR] encoder tick (250–750)
```

This message is the internal contract between the kinematics layer and the hardware/simulation layer. On real hardware, `motors_command_parser_node` is replaced by the LX-16A serial driver.

---

## Steering Kinematics

The `vel_parser_node` (in both Python and C++) implements the full Ackermann-inspired kinematics. Here is exactly what happens when it receives a `cmd_vel`:

### Hardware Distances (configured as ROS parameters)

```
hardware_distances: [23.0, 25.5, 28.5, 26.0]  # cm
  d1 = 23.0  ← half track width (left-right wheel spacing / 2)
  d2 = 25.5  ← rear wheel to centre distance
  d3 = 28.5  ← front wheel to centre distance
  d4 = 26.0  ← middle wheel to centre distance
```

### Step 1 — Normalize cmd_vel to [-100, +100]

```python
linear_limit  = 1.0   # m/s max
angular_limit = 1.0   # rad/s max

norm_speed    = normalize(linear.x,   -1.0, +1.0, -100, +100)
norm_steering = normalize(angular.z,  -1.0, +1.0, -100, +100) * -1
```

### Step 2 — Calculate Turning Radius

Steering of ±100 maps to a turning radius between `MIN_RADIUS = 55 cm` and `MAX_RADIUS = 255 cm`:

```python
radius = MAX_RADIUS - ((MAX_RADIUS - MIN_RADIUS) * abs(norm_steering)) / 100.0
# radius = 255 cm at zero steering, 55 cm at full steering
```

### Step 3 — Per-Wheel Drive Speeds

For each of the 6 wheels, the speed is proportional to its arc radius relative to the outermost wheel's arc:

```python
# Distances from turning center to each wheel
front_far  = sqrt(d3² + (radius + d1)²)
front_near = sqrt(d3² + (radius - d1)²)
mid_far    = radius + d4
mid_near   = radius - d4
back_far   = sqrt(d2² + (radius + d1)²)
back_near  = sqrt(d2² + (radius - d1)²)

# Outermost radius (the reference, gets 100% of speed)
rx = front_far   (if radius < 111 cm)
rx = mid_far     (if radius ≥ 111 cm)

# Each wheel gets speed proportional to its arc length
v_each = abs(v) * (arc_radius_of_that_wheel / rx)
```

Speeds are then multiplied by `speed_factor = 10` to produce the final `drive_motor[]` values.

### Step 4 — Corner Steering Angles

Each corner angle is computed from the turning radius geometry:

```python
# Front corners (arctan of front-distance / radial distance)
ang_front_far  = atan(d3 / (radius + d1))   # outer front wheel
ang_front_near = atan(d3 / (radius - d1))   # inner front wheel

# Rear corners (counter-steer)
ang_back_far  = atan(d2 / (radius + d1))
ang_back_near = atan(d2 / (radius - d1))
```

These angles (in degrees) are converted to LX-16A encoder ticks via:

```python
tick = (enc_max + enc_min) / 2 + ((enc_max - enc_min) / 90) * degrees
# enc_min = 250, enc_max = 750  →  500 is center (0°), range is ±45°
```

### Step 5 — Gazebo Translation (motors_command_parser_node)

In simulation, `motors_command_parser_node` translates `MotorsCommand` back to `ros2_control` format:

```cpp
// Drive speeds: normalize tick range [-1000, +1000] → [-1, +1], scale by ×20
wheel_velocity = normalize(-1000, 1000, clamp(-1000, 1000, drive_motor[i])) * 20;

// Corner positions: normalize tick [250, 750] → [-1, +1], scale to [-π/2, +π/2]
corner_position = normalize(250, 750, position) * -π/2;
```

---

## Wheel Odometry Node

The `odometry_node` (C++, in `rover_gazebo`) computes wheel odometry by reading `/joint_states` and publishing to `/wheel_odom`.

**Parameters from `odometry.yaml`:**

```yaml
odometry_node:
  ros__parameters:
    wheel_radius: 0.06          # 6 cm wheels
    wheel_separation: 0.5458    # 54.58 cm between left and right wheels
    publish_tf: false           # TF published by EKF, not here
    odom_frame: "odom"
    base_frame: "base_link"
```

It tracks left and right wheel position changes from `joint_states`, integrates them with a sliding velocity window to produce smooth `nav_msgs/Odometry` on `/wheel_odom`.

---

## Localization — EKF Sensor Fusion

The `rover_localization` package uses `robot_localization`'s **EKF node** to fuse wheel odometry velocity with IMU heading into a smooth, drift-corrected pose on `/odometry/filtered`.

### What Gets Fused

```
/odom_rgbd  (visual/wheel odometry)  ──► fuse: vx only
/imu        (IMU sensor)             ──► fuse: yaw + angular velocities
                                          ↓
                                    EKF node
                                          ↓
                              /odometry/filtered    ← used by Nav2
                              TF: odom → base_link
```

### Key EKF Settings (from `ekf.yaml`)

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 20.0
    two_d_mode: true           # flat-plane assumption for Nav2
    publish_tf: true           # EKF owns the odom → base_link TF
    use_sim_time: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # Odometry: only fuse forward velocity (vx)
    odom0: /odom_rgbd
    odom0_config: [false, false, false,
                   false, false, false,
                   true,  false, false,   # ← vx only
                   false, false, false,
                   false, false, false]

    # IMU: fuse yaw + angular rates, NOT acceleration (avoids terrain pitch errors)
    imu0: /imu
    imu0_config: [false, false, false,
                  false, false, true,     # ← yaw
                  false, false, false,
                  true,  true,  true,     # ← angular velocities
                  false, false, false]
    imu0_remove_gravitational_acceleration: false  # pitch on terrain must not corrupt ax/ay
```

> **Why not fuse IMU acceleration?** On Mars terrain, the rover pitches as it climbs rocks. Fusing IMU accelerations (ax, ay) would introduce false forward velocity estimates. Keeping only yaw + angular velocity from the IMU gives stable heading without terrain noise.

### TF Tree

```
map
 └── odom              ← EKF (world_frame: odom)
      └── base_link    ← EKF (publish_tf: true)
           ├── front_left_corner_link
           ├── front_right_corner_link
           ├── ... (all 10 wheel/corner links)
           ├── imu_link
           ├── hokuyo_link       ← Hokuyo LiDAR
           └── camera_link       ← ASUS Xtion RGB-D
```

---

## Navigation — Nav2 Stack

The `rover_navigation` package configures the full Nav2 autonomous navigation stack.

### Planners Available

| Planner | Launch arg | Best for |
|---|---|---|
| `SmacHybrid` | `nav2_planner:=SmacHybrid` | Ackermann-constrained paths (default) |
| `SmacLattice` | `nav2_planner:=SmacLattice` | State lattice planning |

### Controllers Available

| Controller | Launch arg | Best for |
|---|---|---|
| `RPP` | `nav2_controller:=RPP` | Regulated Pure Pursuit (default) |
| `TEB` | `nav2_controller:=TEB` | Timed Elastic Band (dynamic obstacles) |

### Nav2 Components

| Component | Config file | Role |
|---|---|---|
| `bt_navigator` | `common.yaml` | Behavior tree mission orchestration |
| `nav2_planner` | `SmacHybrid.yaml` / `SmacLattice.yaml` | Global path planning |
| `nav2_controller` | `RPP.yaml` / `TEB.yaml` | Local trajectory following |
| `nav2_costmap_2d` | `costmaps.yaml` | Obstacle inflation & costmap generation |
| `smoother_server` | `common.yaml` | Path smoothing (SimpleSmoother) |
| `behavior_server` | `common.yaml` | Recovery behaviors (spin, backup, wait) |

Nav2 consumes `/odometry/filtered` (from EKF) and `/scan` (from Hokuyo) to plan and execute paths.

---

## Package Structure

```
ros2_rover/src/
│
├── rover_bringup/                   # Real hardware launch files
│   ├── launch/
│   │   ├── rover.launch.py          ← Real rover full bringup
│   │   ├── ublox.launch.py          ← GPS (uBlox) launch
│   │   └── urg_node.launch.py       ← Hokuyo LiDAR serial launch
│   └── config/
│       ├── ublox.yaml               ← GPS config
│       └── urg_node_serial.yaml     ← Hokuyo serial config
│
├── rover_description/               # URDF/Xacro robot model & STL meshes
│   ├── robots/
│   │   └── rover.urdf.xacro         ← Top-level robot URDF
│   ├── urdf/
│   │   ├── bases/                   ← body, bogie, rocker, differential xacros
│   │   ├── wheels/                  ← wheel and corner xacros
│   │   └── sensors/                 ← hokuyo, IMU, ASUS Xtion xacros
│   ├── meshes/
│   │   ├── bases/                   ← STL files (rocker, bogie, body, steering asm)
│   │   ├── wheels/                  ← wheel STLs
│   │   └── sensors/                 ← Hokuyo DAE files, ASUS Xtion DAE
│   ├── config/
│   │   └── control.yaml             ← ros2_control hardware interface config
│   └── launch/
│       └── robot_state_publisher.launch.py
│
├── rover_gazebo/                    # Simulation worlds, nodes, launch files
│   ├── launch/
│   │   ├── mars.launch.py           ← Mars simulation ✅ (YOUR LAUNCH CMD)
│   │   ├── moon.launch.py           ← Moon simulation
│   │   ├── forest.launch.py         ← Forest simulation
│   │   ├── low_moon.launch.py       ← Low gravity moon
│   │   ├── curiosity.launch.py      ← Curiosity path world
│   │   ├── gazebo.launch.py         ← Core Gazebo bringup (called by all worlds)
│   │   └── include/
│   │       ├── spawn.launch.py      ← Spawns rover + ros2_control controllers
│   │       └── cmd_vel.launch.py    ← Launches vel_parser + motors_command_parser
│   ├── worlds/
│   │   ├── mars.world ← Mars terrain world (SDF)
|   |   |           
│   ├── models/
│   │   ├── mars_terrain/            ← Mars terrain mesh + reddish-sand texture
│   │   ├── crater/                  ← Crater 3D model
│   │   ├── curiosity_path/          ← Curiosity rover path mesh
│   │   ├── jaggedrock/              ← Jagged rock model
│   │   ├── rock9/                   ← Rock model
│   │   ├── rockformation/           ← Rock cluster model
│   │   └── terrain/                 ← Generic terrain
│   ├── src/
│   │   ├── motors_command_parser_node.cpp  ← Translates MotorsCommand → ros2_control
│   │   ├── odometry_node.cpp               ← Wheel odometry from joint_states
│   │   └── ground_truth_remapper_node.cpp  ← Remaps Gazebo ground truth pose
│   └── config/
│       ├── gz_bridge.yaml           ← QoS overrides for Gazebo ↔ ROS 2 bridge
│       └── odometry.yaml            ← Wheel radius + separation params
│
├── rover_localization/              # EKF sensor fusion
│   ├── config/
│   │   └── ekf.yaml                 ← EKF parameters (fuses odom + IMU)
│   └── launch/
│       ├── localization.launch.py   ← Main localization launcher
│       ├── ekf.launch.py            ← EKF node only
│       ├── rgbd_odometry.launch.py  ← RGB-D visual odometry
│       └── rtabmap.launch.py        ← RTABMap SLAM (optional)
│
├── rover_motor_controller/          # Python LX-16A controller
│   └── rover_motor_controller/
│       ├── motor_controller/
│       │   ├── vel_parser_node.py   ← Ackermann kinematics (Python)
│       │   └── controller_node.py   ← LX-16A serial command node
│       └── lx16a/
│           ├── lx16a.py             ← LX-16A servo protocol
│           └── motor_controller.py  ← Servo driver
│
├── rover_motor_controller_cpp/      # C++ LX-16A controller (used in sim)
│   ├── src/
│   │   ├── motor_controller/
│   │   │   ├── vel_parser_node.cpp  ← Ackermann kinematics (C++)
│   │   │   └── controller_node.cpp  ← LX-16A serial command node
│   │   └── lx16a/
│   │       ├── lx16a.cpp            ← LX-16A servo protocol
│   │       ├── motor_controller.cpp ← Servo driver
│   │       └── serial.cpp           ← Serial port handler
│   └── include/
│       ├── lx16a/                   ← Headers
│       └── motor_controller/        ← Headers
│
├── rover_msgs/                      # Custom ROS 2 messages
│   └── msg/
│       └── MotorsCommand.msg        ← drive_motor[6] + corner_motor[4]
│
├── rover_navigation/                # Nav2 config & launch
│   ├── launch/
│   │   ├── bringup.launch.py        ← Full Nav2 bringup
│   │   └── navigation.launch.py     ← Navigation only
│   ├── params/
│   │   ├── common.yaml              ← bt_navigator, smoother, behavior server
│   │   ├── costmaps.yaml            ← Global + local costmap config
│   │   ├── RPP.yaml                 ← Regulated Pure Pursuit controller
│   │   ├── TEB.yaml                 ← TEB controller
│   │   ├── SmacHybrid.yaml          ← Smac Hybrid-A* planner
│   │   └── SmacLattice.yaml         ← Smac Lattice planner
│   └── behavior_trees/
│       └── rover_bt.xml             ← Custom Nav2 behavior tree
│
├── rover_service/                   # Linux systemd autostart service
│   ├── install.sh
│   ├── rover.service
│   └── rover.sh
│
└── rover_teleop/                    # Keyboard + joystick teleop
    ├── rover_teleop/
    │   └── teleop_keyboard_node.py  ← w/a/s/d/x keyboard control
    ├── launch/
    │   └── joy_teleop.launch.py     ← PS3 joystick launch
    └── config/
        └── ps3.yaml                 ← PS3 button/axis mapping
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| **Ubuntu** | **24.04 LTS (Noble Numbat)** |
| **ROS 2** | **Jazzy Jalisco** |
| **Gazebo** | **Harmonic (gz sim)** |
| Python | 3.12 |

> ⚠️ Do NOT use Ubuntu 22.04 or Gazebo Classic. This repo uses `gz sim` (Gazebo Harmonic) and the `ros_gz` bridge — the old `gzserver`/`gazebo` commands will not work.

### Install ROS 2 Jazzy

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common curl
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install -y ros-jazzy-desktop
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

### Install Simulation & Navigation Dependencies

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-robot-localization \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-teleop-twist-joy
```

### Source ROS 2 Jazzy

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Installation & Build

### 1. Create workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. Clone the repo

```bash
git clone https://github.com/sachinmandal3580-rgb/ros2_rover.git
# All packages are inside the src/ folder of the repo
cp -r ros2_rover/src/* .
```

> Or if you prefer to keep the repo structure:
> ```bash
> cd ~/ros2_ws
> colcon build --symlink-install --paths src/ros2_rover/src/*
> ```

### 3. Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src -r -y --rosdistro jazzy
```

### 4. Build

```bash
colcon build --symlink-install
```

### 5. Source the workspace

```bash
source ~/ros2_ws/install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Launch Command

```bash
ros2 launch rover_gazebo mars.launch.py
```

This single command starts the entire simulation stack:

| What starts | Details |
|---|---|
| **Gazebo Harmonic** | `gz sim` with `mars.world` — textured terrain, crater, rock formations |
| **Rover spawn** | `ros_gz_sim create` spawns the rover URDF at pose `(-1, -1, 0.5)` |
| **ros2_control** | Spawns `joint_state_broadcaster`, `position_controller`, `velocity_controller` |
| **robot_state_publisher** | Publishes URDF and all TF frames from joint states |
| **gz bridge** | Bridges `/clock`, `/scan`, `/imu`, `/camera/*`, `/cmd_vel` between Gazebo ↔ ROS 2 |
| **vel_parser_node** | C++ Ackermann kinematics node — converts `/cmd_vel` → `motors_command` |
| **motors_command_parser_node** | Converts `MotorsCommand` → `position_controller` + `velocity_controller` commands |
| **odometry_node** | Computes wheel odometry from `/joint_states` → `/wheel_odom` |
| **EKF localization** | Fuses `/odom_rgbd` + `/imu` → `/odometry/filtered` + TF `odom → base_link` |
| **Nav2 stack** | Full navigation with SmacHybrid planner + RPP controller (default) |
| **RViz2** | Opens with `rover_gazebo/rviz/default.rviz` config |

### Optional launch arguments

```bash
# Change Nav2 planner
ros2 launch rover_gazebo mars.launch.py nav2_planner:=SmacLattice

# Change Nav2 controller
ros2 launch rover_gazebo mars.launch.py nav2_controller:=TEB

# Disable RViz
ros2 launch rover_gazebo mars.launch.py launch_rviz:=False

# Custom spawn position
ros2 launch rover_gazebo mars.launch.py initial_pose_x:=0.0 initial_pose_y:=0.0
```

---

## ROS 2 Topics Reference

### Control

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Subscribe | Main velocity command |
| `/motors_command` | `rover_msgs/msg/MotorsCommand` | Internal | Kinematics output: drive speeds + corner ticks |
| `/velocity_controller/commands` | `std_msgs/msg/Float64MultiArray` | Publish | 6 drive wheel velocities (rad/s) |
| `/position_controller/commands` | `std_msgs/msg/Float64MultiArray` | Publish | 4 corner steering angles (rad) |

### Sensors

| Topic | Type | Description |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Hokuyo LiDAR scan |
| `/imu` | `sensor_msgs/msg/Imu` | IMU data |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | ASUS Xtion RGB image |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | ASUS Xtion depth image |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | RGB-D point cloud |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration info |
| `/joint_states` | `sensor_msgs/msg/JointState` | All 10 joint positions/velocities |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo simulation clock |

### Odometry & Localization

| Topic | Type | Description |
|---|---|---|
| `/wheel_odom` | `nav_msgs/msg/Odometry` | Raw wheel encoder odometry |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | EKF-fused smooth odometry (used by Nav2) |

### Navigation

| Topic | Type | Description |
|---|---|---|
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM-generated or static map |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Nav2 navigation goal (set via RViz2) |
| `/plan` | `nav_msgs/msg/Path` | Global planned path |
| `/local_plan` | `nav_msgs/msg/Path` | Local trajectory |

---

## Teleop & Manual Control

### Custom Keyboard Node (w/a/s/d/x)

```bash
ros2 run rover_teleop teleop_keyboard_node
```

Key bindings (from `teleop_keyboard_node.py`):

```
        w          ← forward
   a    s    d     ← left | stop | right
        x          ← backward
```

### PS3 Joystick Teleop

```bash
ros2 launch rover_teleop joy_teleop.launch.py
```

### Manual Topic Commands

```bash
# Drive forward at 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 10

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.4}}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Monitor MotorsCommand (internal kinematics output)
ros2 topic echo /motors_command

# Monitor live odometry
ros2 topic echo /odometry/filtered

# Check joint states
ros2 topic echo /joint_states
```

---

## Troubleshooting

**`gz sim` not found / Gazebo doesn't launch**
```bash
sudo apt install ros-jazzy-ros-gz
source /opt/ros/jazzy/setup.bash
```

**Mars world takes too long / crashes (GPU issue)**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch rover_gazebo mars.launch.py
```

**Qt / display error on Gazebo Harmonic**
```bash
export QT_QPA_PLATFORM=xcb
ros2 launch rover_gazebo mars.launch.py
```

**Rover spawns but doesn't move**
```bash
# Check controllers are running
ros2 control list_controllers
# Should show: joint_state_broadcaster, position_controller, velocity_controller (all active)

# Check vel_parser_node is up
ros2 node list | grep vel_parser

# Check MotorsCommand is being published
ros2 topic echo /motors_command
```

**`/odometry/filtered` not publishing**
```bash
# Check EKF is running
ros2 node list | grep ekf

# Check input topics exist
ros2 topic hz /odom_rgbd
ros2 topic hz /imu
```

**Nav2 goal doesn't work / rover doesn't navigate**
```bash
# Confirm Nav2 is up
ros2 node list | grep nav2

# Check costmaps are generating
ros2 topic echo /global_costmap/costmap

# Ensure localization is active
ros2 topic hz /odometry/filtered
```

**Build fails: missing rover_msgs**
```bash
# Build rover_msgs first
colcon build --packages-select rover_msgs
source install/setup.bash
colcon build --symlink-install
```

---

## License

MIT License — see [LICENSE](src/LICENSE) for details.

---

*Built with ROS 2 Jazzy Jalisco · Gazebo Harmonic · Ubuntu 24.04 · Nav2 · ros2_control*
