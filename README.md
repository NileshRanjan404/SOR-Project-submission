# TerraROVER — 6-Wheeled Planetary Exploration Rover

## What this project is

This is a complete ROS 2 Jazzy simulation of a six wheeled Mars rover that can drive itself around, figure out where it is, and navigate to a goal on its own, all inside Gazebo. It started as a set of TODOs to fill in, but along the way it turned into a full debugging project too, since I ended up fixing a few things in the codebase that weren't part of the original assignment at all.

The rover uses a rocker bogie suspension with four steerable corner wheels and two fixed middle wheels, and the whole pipeline goes from a navigation goal all the way down to individual wheel commands and back up through localization to keep track of where the rover actually is.

## What I actually built

### Steering and motor control
I wrote the logic that takes a velocity command and turns it into six wheel speeds and four steering angles. When driving straight, all six wheels get the same speed with the right side flipped in sign, since those wheels are mounted mirrored. When turning, I calculate a turning radius and scale each wheel's speed based on how far it is from the center of that turn, so the inner wheels go slower and the outer wheels go faster, which stops them from fighting each other. The four corner wheels also get individual steering angles so the inside wheels turn more sharply than the outside ones, similar to how a normal car steers.

### Keyboard driving
I set up the teleop node so you can drive the rover manually with w, a, s, d, and x, with speed building up gradually rather than jumping straight to full speed, and a safety stop that kicks in automatically if you exit the program.

### Localization
The rover fuses wheel odometry and IMU data through an Extended Kalman Filter to keep a stable estimate of its own position. I set it up to only trust the odometry's forward speed and not its position, since that drifts a lot on rough terrain, and to trust the IMU's rotation but not its position, for the same reason.

### Navigation
I tuned the path following controller and the costmaps so the rover can actually plan a path and drive to a goal while staying clear of obstacles, but not so cautious that it refuses to go through anything remotely narrow.

## Bugs I found and fixed along the way

Honestly this ended up being most of the actual work. A few things were broken in the repo before I even started, and a few more I introduced and had to track down myself.

**The URDF was missing most of itself.** The file I was given only had the ros2_control section in it, nothing that actually defined the rover's body, wheels, or sensors. I dug through the git history and found the very first commit had a complete version, so I pulled that and merged it with my own values.

**A tiny typo crashed the entire navigation stack.** After finishing everything, Nav2 just would not start, every node stayed stuck in an unconfigured state and nothing happened when I sent it a goal. I eventually ran the controller node by itself and found I had written the costmap width and height as decimals instead of whole numbers, which Nav2's code doesn't accept. That one typo was silently crashing the very first node the whole system depends on.

**The rover couldn't spin as tight as the planner wanted it to.** Once navigation started working, the rover would sometimes get stuck wiggling its steering back and forth without moving. It turned out the path planner didn't know the rover's actual minimum turning radius, so it kept asking for turns sharper than the steering could physically do. I added that limit and it cleared up.

**The simulation environment itself was slow.** I was running this on software rendering with no real GPU, which made the camera feed too slow for the mapping system to keep up with at first. I tried a few things to work around it, and once the Nav2 crash above was actually fixed, everything ran a lot better anyway.

## How to run it

Build the workspace first:
```bash
cd ~/ros2_rover
colcon build
source install/setup.bash
```

Launch the simulation:
```bash
ros2 launch rover_gazebo mars.launch.py
```

Drive it manually to explore around:
```bash
ros2 run rover_teleop teleop_keyboard_node
```

Once you've built up a map, open RViz, click 2D Goal Pose, click a spot on the map, and the rover will plan a path and drive there on its own.

## What's still a work in progress

The balance between how much clearance the rover keeps from obstacles and how tight a gap it's willing to squeeze through is still something I'm tuning. Right now it favors safety a bit more than maneuverability, so it sometimes avoids paths that would technically fit. This is a tuning trade off rather than a bug, and I'd like to keep refining it.

## Final thought

Most of what I actually learned here didn't come from filling in the TODOs, it came from the parts that were broken and had no obvious explanation. Tracing a missing chunk of a file through git history, or realizing a single decimal point had taken down an entire subsystem, taught me a lot more about how these systems fit together than writing the steering math did. That felt like the real point of the project.
