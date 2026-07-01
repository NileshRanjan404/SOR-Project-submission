# TerraROVER — 6-Wheeled Planetary Exploration Rover

## Problem Statement

Planetary exploration demands robots capable of safely traversing harsh, unstructured environments where direct human operation is impossible. Uneven terrain, loose regolith, steep craters, rocky obstacles, and significant communication delays make manual control inefficient and often impractical.

Before deploying autonomous rovers on real planetary missions, engineers require a realistic simulation environment that accurately models rover mechanics, sensing, localization, and navigation. Such a platform enables rapid testing and validation of navigation algorithms, motion control, perception pipelines, and autonomous behaviors without the cost, risk, or logistical challenges of physical hardware.

This project develops a complete **ROS 2 Jazzy** simulation framework for a six-wheeled rocker-bogie Mars rover operating inside a realistic Martian environment. From receiving velocity commands to executing wheel-level control, estimating robot pose, and autonomously navigating toward mission goals, the entire robotics pipeline is implemented using modern ROS 2 technologies.

---

## The Story

After a seven-month journey through deep space, an autonomous rover touches down safely on the surface of Mars. As the dust from its landing settles, its cameras reveal a world unlike any on Earth—towering rock formations, steep crater walls, loose regolith, and endless stretches of unexplored terrain.

There is no GPS to determine its location. No roads to follow. No human pilot waiting with a joystick. Even a simple command from Earth takes several minutes to arrive, making real-time control impossible.

Mission Control can only assign scientific objectives:

Reach the exposed rock formation.
Investigate the nearby crater.
Survey the ridge for signs of past water activity.

How the rover achieves these objectives is entirely up to it.

Before taking its first meter across the Martian surface, the rover must answer a series of critical questions on its own.

"Where am I?"

"How should I steer to reach the destination?"

"Is there a safe path around those rocks?"

"Am I still following the planned route?"

To answer these questions, the rover continuously estimates its position using wheel odometry and inertial measurements, fuses sensor data to build a reliable estimate of its pose, plans collision-free paths through rough terrain, and converts navigation commands into coordinated steering angles and wheel velocities. Its six-wheeled rocker-bogie suspension keeps every wheel in contact with the ground, allowing it to climb obstacles, descend into craters, and maintain stability across terrain that would stop conventional vehicles.

This project recreates that entire autonomy pipeline inside a realistic ROS 2 Jazzy simulation. Every subsystem—from steering control and wheel-level actuation to localization, navigation, and autonomous decision-making—works together exactly as it would on a planetary rover preparing for a real exploration mission.

> **The mission isn't simply to drive across Mars—it's to build the intelligence that allows a rover to explore an unknown world safely, independently, and without human intervention.**
---

## Objective

Develop a complete **ROS 2 Jazzy** software stack capable of autonomously operating a six-wheeled Mars rover inside a realistic simulation environment.

**Overall pipeline:**

```
Navigation Goal → Nav2 Planner → cmd_vel → Steering & Motor Controller → Wheel Commands → ros2_control → Gazebo → Odometry + IMU → EKF Localization → Updated Pose
```

---

## System Overview

The project consists of six major components.

### 1. Gazebo Simulation
Provides a realistic Martian environment where the rover operates under a physics-based simulation. Gazebo simulates wheel-ground interaction, terrain physics, sensors, and robot dynamics, enabling the complete autonomy stack to be tested without physical hardware.

### 2. Rover Description
Defines the six-wheeled rocker-bogie rover using **URDF/Xacro**, including rocker-bogie suspension, steering and drive joints, wheel geometry, inertial properties, sensors, `ros2_control` interfaces, and Gazebo plugins. This serves as the digital twin of the physical rover.

### 3. Steering & Motor Controller
Receives velocity commands (`cmd_vel`) and converts them into individual steering angles and wheel velocities using custom steering kinematics for a six-wheeled rover. The generated wheel commands are executed through **ros2_control**, allowing the rover to move realistically inside Gazebo.

### 4. Keyboard Teleoperation
Publishes velocity commands that allow the rover to be manually driven. These commands provide the input to the steering controller and are useful for testing rover motion before autonomous navigation.

### 5. Localization
Computes wheel odometry from joint state feedback and fuses wheel odometry together with IMU measurements using an **Extended Kalman Filter (EKF)** to estimate the rover's pose throughout the simulation.

### 6. Navigation
Uses the **Nav2** stack to autonomously navigate toward user-defined goals by performing global path planning, local path following, obstacle avoidance, and velocity command generation. The generated `cmd_vel` commands are forwarded to the steering controller, completing the autonomy pipeline.

---

## What You Need To Implement

This repository contains several TODOs distributed across different packages. Complete these implementations to obtain a fully autonomous Mars rover simulation.

---

### 1. Rover Description — `rover_description/robots/rover.urdf.xacro`

**TODO 1 — Configure Wheel Collision Geometry**

Complete the collision geometry for the rover wheels so that Gazebo can correctly detect terrain contact during simulation.

**TODO 2 — Configure Wheel Inertial Properties**

Define the wheel inertial parameters required for realistic physics simulation.

---

### 2. Keyboard Teleoperation — `rover_teleop/rover_teleop/teleop_keyboard_node.py`

**TODO 1 — Implement Forward and Reverse Motion**

Publish linear velocity commands based on keyboard input.

**TODO 2 — Implement Steering Commands**

Handle left and right turning inputs by publishing the appropriate angular velocity.

**TODO 3 — Implement Speed Adjustment**

Allow the user to increase and decrease the rover's driving speed while respecting predefined limits.

---

### 3. Steering & Motor Controller — `rover_motor_controller_cpp/src/motor_controller/vel_parser_node.cpp`

**TODO 1 — Process Incoming Velocity Commands**

Limit the received linear and angular velocities, compute the rover speed, and normalize the commands before converting them into wheel motion.

**TODO 2 — Handle Straight-Line Motion**

Implement the wheel velocity calculation used when the rover drives straight, ensuring all wheels rotate with the correct speed and direction.

**TODO 3 — Assign Steering Angles**

Compute and assign steering angles for the steering wheels according to the requested turning direction.

---

### 4. EKF Localization — `rover_localization/config/ekf.yaml`

**TODO 1 — Configure Wheel Odometry Measurements**

Select which wheel odometry measurements should be fused by the Extended Kalman Filter and observe how different configurations affect localization.

**TODO 2 — Configure IMU Measurements**

Choose which IMU measurements should contribute to the EKF state estimate and evaluate their impact on localization accuracy.

---

### 5. Regulated Pure Pursuit Controller — `rover_navigation/params/RPP.yaml`

**TODO 1 — Tune Lookahead Distance**

Adjust the lookahead parameters to achieve smooth and accurate path tracking.

**TODO 2 — Tune Velocity Regulation**

Modify the controller's velocity regulation parameters to balance navigation speed, stability, and turning performance.

---

### 6. Navigation Costmaps — `rover_navigation/params/costmaps.yaml`

**TODO 1 — Configure the Inflation Layer**

Tune the inflation radius and scaling factor to maintain safe obstacle clearance.

**TODO 2 — Configure the Obstacle Layer**

Adjust obstacle detection parameters to improve environmental representation.

**TODO 3 — Configure the Robot Footprint**

Modify the robot footprint (or robot radius) to ensure accurate collision checking during navigation.

---

## Running the Project

**Pre-requisite packages:**
```bash
sudo apt update

sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-xacro \
  ros-jazzy-robot-localization \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-teleop-twist-joy
```

**Before opening any terminal, build the workspace:**
```bash
cd ~/ros2_rover
colcon build
```

**For every new terminal, source the workspace:**
```bash
source install/setup.bash
```

---

### Step 1 — Launch Gazebo

```bash
ros2 launch rover_gazebo mars.launch.py
```

Wait until the Martian world loads completely.

---

### Step 2 — Drive the Rover

```bash
ros2 run rover_teleop teleop_keyboard_node
```
Drive the rover manually around to build a map of the environment

---

### Step 3 — Launch Navigation

The same file also launches the navigation pipeline so after all lifecycle 

nodes are active, give a goal(use 2D goal pose in Rviz) and let the rover explore...


# Bonus Challenges

Once you've completed all the required TODOs and the rover can successfully navigate to manually selected goals, try extending the project with the following challenges.

---

## Bonus Challenge 1 — Fully Autonomous Random Exploration

### Objective

Currently, the rover waits for a user to provide a **2D Goal Pose** in RViz before it begins navigating.

Modify the project so that the rover automatically generates random navigation goals within the environment and continuously explores the map without requiring any human input.

### What to implement

Create a ROS 2 node that:

- Subscribes to the occupancy grid map (or uses a predefined exploration region).
- Randomly generates valid navigation goals.
- Ensures generated goals lie in free space rather than inside obstacles.
- Sends each goal to the Nav2 **NavigateToPose** action server.
- Waits until the current goal succeeds before generating the next one.
- Repeats this process indefinitely or for a specified number of goals.

### Suggested implementation steps

1. Subscribe to the occupancy map published by Nav2.
2. Randomly sample map coordinates.
3. Reject occupied or unknown cells.
4. Convert valid map cells into `geometry_msgs/PoseStamped`.
5. Send the goal using the `NavigateToPose` action client.
6. Wait for the navigation result.
7. Generate and send the next random goal.

---

## Bonus Challenge 2 — Autonomous Image Collection

### Objective

Extend the rover so that it captures an image every time it successfully reaches a navigation goal, creating an autonomous visual survey of the environment.

### What to implement

After every successful navigation goal:

- Capture the latest image from the onboard RGB camera.
- Save the image to disk.
- Continue navigating to the next waypoint.
- Organize all captured images with sequential or timestamp-based filenames.

### Suggested implementation steps

1. Subscribe to the rover's RGB camera topic.
2. Convert incoming ROS images into OpenCV format using **cv_bridge**.
3. Detect when the Nav2 action reports a successful goal.
4. Save the latest camera frame using OpenCV (`cv2.imwrite()`).
5. Store images inside a dedicated directory such as:

```text
captured_images/
├── image_001.png
├── image_002.png
├── image_003.png
...
```

Optionally include the timestamp, robot pose, or goal number in the filename or as image metadata.

---

Together, these two challenges transform the rover from a robot that simply drives to user-selected goals into an autonomous planetary explorer capable of navigating independently while collecting visual data from its surroundings—much like a real-world Mars exploration rover.

## Deliverables

### 1. Source Code
- Completed implementations for all TODOs
- Functional ROS 2 packages
- Updated launch and configuration files

### 2. Demonstration Video

Show:
- Gazebo simulation
- Manual rover teleoperation
- Steering and motor control
- Wheel odometry generation
- EKF localization
- Autonomous navigation using Nav2
- Goal reaching inside the Martian environment

### 3. Report

Briefly describe:
- Steering kinematics implementation
- Wheel odometry estimation
- EKF sensor fusion configuration
- Navigation parameter tuning
- Challenges encountered

---

## Final Message

Planetary rovers cannot rely on perfect terrain, continuous communication, or external positioning systems. Every successful mission depends on the seamless integration of perception, localization, motion control, and autonomous navigation.

By completing this project, you will implement each stage of that autonomy pipeline — from low-level wheel control to high-level navigation — and gain practical experience with the same ROS 2 technologies used in modern mobile robotics.

The objective is not simply to drive a simulated rover across Mars, but to understand how robotic explorers transform high-level mission objectives into safe, intelligent, and fully autonomous exploration of unknown worlds.