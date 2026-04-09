# Autonomous Mobile Robotics: Localization, Mapping, and Navigation

[![ROS Version](https://img.shields.io/badge/ROS-Noetic-green.svg)](http://wiki.ros.org/noetic)
[![Language](https://img.shields.io/badge/C%2B%2B-14%2B-blue.svg)](https://en.wikipedia.org/wiki/C%2B%2B)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

This repository contains two robotics projects developed in C++ for the **Robotics** course at *Politecnico di Milano*. The projects cover the full autonomous navigation pipeline: from low-level kinematics and GPS coordinate transformations to 2D SLAM (Simultaneous Localization and Mapping) and autonomous path planning using the ROS Navigation Stack.

---

## Project 1: Robot Odometry & GPS Localization

**Folder:** `first_project/`

### Overview
This project focuses on calculating high-precision vehicle odometry and transforming global GPS coordinates into a local reference frame. Using telemetry data (speed and steering angle) from a vehicle running on the Monza circuit, we compute the robot's pose in real-time.

### Key Features
* **Custom Odometry Node:** Implemented a ROS C++ node (`odometer`) that subscribes to vehicle telemetry (`/speedsteer`) and computes the state using **Runge-Kutta kinematics** (Ackermann steering model).
* **Coordinate Transformations:** Handled the mathematical conversion of dual-GPS coordinates from Latitude/Longitude to ECEF (Earth-Centered, Earth-Fixed), and subsequently to ENU (East-North-Up) local reference frames.
* **TF Broadcasting:** Published the calculated `nav_msgs/Odometry` and the dynamic `tf` transform (`odom` $\rightarrow$ `base_link`).
* **Telemetry Pipeline:** Tracked Monza circuit sector times using Haversine formulas and spatial boundary checking.

### Usage
1. Build the workspace: `catkin_make`
2. Run the odometry node: `rosrun first_project odometer`
3. Play the provided bag file: `rosbag play --clock project.bag`

---

## Project 2: 2D Mapping & Autonomous Navigation (SLAM)

**Folder:** `second_project/`

### Overview
This project implements a complete SLAM pipeline and autonomous navigation system for a custom-sized differential drive robot ($0.54m \times ...$). It fuses noisy odometry with front and rear LiDAR scans to build a clean 2D map, and then uses Action Servers to navigate the robot to target goals within a simulation.

### Key Features
* **LiDAR Sensor Fusion:** Merged `/scan_front` and `/scan_back` topics into a single 360-degree representation using `TF2` coordinate transformations.
* **Self-Collision Filtering:** Implemented a point-filtering algorithm to dynamically remove LiDAR hits that collided with the robot's own chassis, preventing false obstacles in the map.
* **SLAM Pipeline:** Integrated the processed laserscans and odometry data with standard ROS mapping packages to generate a reliable 2D occupancy grid of the environment.
* **Autonomous Navigation:** Configured a realistic robot simulation using **Stage** and the **ROS Navigation Stack**. Sent asynchronous navigation goals using ROS Action Servers.

### Usage
1. Build the workspace: `catkin_make`
2. Launch the mapping/navigation pipeline: `roslaunch second_project main.launch`
3. Play the bag file (with simulated time): `rosbag play --clock robotics2.bag`
4. Use the `RViz` interface or Action Client to send 2D Nav Goals.

---

## Prerequisites & Dependencies

To run these packages, you need an Ubuntu environment with ROS installed (tested on ROS Noetic).

```bash
# Install core ROS navigation and mapping dependencies
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-gmapping
sudo apt-get install ros-noetic-stage-ros
sudo apt-get install ros-noetic-tf2-geometry-msgs
