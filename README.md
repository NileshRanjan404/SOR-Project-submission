# Problem Statement

Planetary exploration demands robots capable of safely traversing harsh, unstructured environments where direct human operation is impossible. Uneven terrain, loose regolith, steep craters, rocky obstacles, and significant communication delays make manual control inefficient and often impractical.

Before deploying autonomous rovers on real planetary missions, engineers require a realistic simulation environment that accurately models rover mechanics, sensing, localization, and navigation. Such a platform enables rapid testing and validation of navigation algorithms, motion control, perception pipelines, and autonomous behaviors without the cost, risk, or logistical challenges of physical hardware.

This project develops a complete **ROS 2 Jazzy** simulation framework for a six-wheeled rocker-bogie Mars rover operating inside a realistic Martian environment. From receiving velocity commands to executing wheel-level control, estimating robot pose, and autonomously navigating toward mission goals, the entire robotics pipeline is implemented using modern ROS 2 technologies.

---

# The Story

A rover has just touched down on the surface of Mars.

Surrounding it is an untouched landscape of craters, jagged rocks, loose regolith, and terrain that no human has ever explored. There are no roads, no GPS satellites overhead, and no engineer nearby to take control. Every meter of progress must be achieved through the rover's own ability to perceive its environment, estimate its position, and make intelligent decisions.

Mission control, separated by millions of kilometers, can only transmit high-level objectives:

* *Reach the geological outcrop.*
* *Investigate the nearby crater.*
* *Collect samples from the ridge ahead.*

The rover must determine **how** to accomplish these objectives autonomously.

To achieve this, it continuously estimates its pose using wheel odometry and inertial measurements, interprets data from onboard sensors, plans safe paths around obstacles, and converts navigation goals into precise steering angles and wheel velocities. Its rocker-bogie suspension keeps all six wheels in contact with the uneven Martian surface, allowing it to climb over rocks, descend into craters, and traverse terrain that would immobilize conventional vehicles.

This project recreates that complete autonomy pipeline in simulation. From low-level wheel control and steering kinematics to localization, perception, and autonomous navigation, every subsystem works together to emulate how a planetary rover would operate during a real exploration mission.

Rather than simply driving a robot through a simulated world, the objective is to model the intelligence and engineering that allow robotic explorers to venture where humans cannot—transforming high-level mission objectives into safe, autonomous exploration across an unknown planet.

---

# Objective

Develop a complete **ROS 2** software stack capable of autonomously operating a six-wheeled Mars rover within a realistic simulation environment.

The project guides students through the implementation of fundamental robotics concepts, including:

* Custom Ackermann-inspired steering kinematics for a six-wheeled rocker-bogie rover
* Translation of `cmd_vel` commands into individual steering and wheel motions
* Wheel odometry estimation using joint state feedback
* Sensor fusion with an Extended Kalman Filter (EKF)
* Autonomous navigation using the Nav2 stack
* Integration with **Gazebo Harmonic** and **ros2_control**

---

# Overall Pipeline

```text
    Navigation Goal 
          ↓ 
    Nav2 Planner 
          ↓ 
      cmd_vel 
          ↓
  Steering Kinematics
          ↓ 
Wheel & Steering Commands 
          ↓
    ros2_control
          ↓ 
    Gazebo Rover 
          ↓
    Wheel Odometry 
          ↓
  EKF Localization
          ↓
  Updated Robot Pose
```

# CHANGES 

- rover_teleop/rover_teleop/teleop_keyboard_node.py  - 3 todo's
- rover_navigation/params/RPP.yaml - 2 todo's
- rover_navigation/params/costmaps.yaml - 3 todo's
- 
