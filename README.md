# Visual Odometry Project Readme

## Introduction
Welcome to the Visual Odometry project! This repository contains code for implementing visual odometry using a stereo camera setup and an IMU sensor. Visual odometry is a technique used in robotics and computer vision to estimate the motion of a vehicle or robot by analyzing the changes in camera images over time.

## What is Visual Odometry?
Visual odometry is a method used to estimate the motion of a robot or vehicle by analyzing sequential images captured by one or more cameras. By tracking features between consecutive frames and estimating the relative camera motion, visual odometry can provide valuable information for tasks such as navigation, mapping, and localization in robotic systems.

## Project Structure
- **`visual_odometry_node.py`**: This is the main Python script that contains the implementation of visual odometry using ROS (Robot Operating System) and OpenCV. It subscribes to camera images, camera information, and IMU sensor data, and publishes visual odometry data, odometry information, robot position, and camera transforms.
- **`differential_drive_robot_control`**: This directory contains ROS messages and services related to robot control.
- **`launch`**: This directory contains launch files for starting the visual odometry node and other ROS nodes.
- **`config`**: Configuration files for ROS parameters and camera calibration data.
- **`README.md`**: This readme file providing an overview of the project.

## Dependencies
- ROS (Robot Operating System)
- Python 2.7 or later
- OpenCV
- NumPy
- SciPy
- FilterPy (for Kalman filter implementation)
- cv_bridge (ROS package for converting between ROS image messages and OpenCV images)

## How to Use
1. Clone the repository to your ROS workspace.
2. Build the ROS packages using `catkin_make`.
3. Make sure you have a stereo camera setup and an IMU sensor connected to your robot.
4. Update camera calibration parameters in the configuration files if necessary.
5. Launch the visual odometry node using ROS launch files.
6. Monitor the published topics to access visual odometry data, odometry information, and robot position.

## Key Components
- **VisualOdometryNode**: This class initializes ROS nodes, subscribers, publishers, and the VisualOdometry object.
- **VisualOdometry**: This class implements the visual odometry algorithm, feature extraction, feature matching, pose estimation, and motion tracking.
- **Kalman Filter**: Kalman filters are used for sensor fusion and state estimation from IMU data and odometry measurements.
- **Feature Extraction and Matching**: ORB algorithm is used for feature extraction, and Brute-Force Matcher is used for feature matching.
- **Essential Matrix Estimation**: RANSAC is used to estimate the Essential Matrix (E) from matched feature points.
- **Pose Estimation**: RecoverPose estimates the relative pose (rotation and translation) between camera frames using the Essential Matrix.

## ROS Topics
- **Subscribed Topics**:
  - `/camera/image_raw`: Raw camera image data from the primary camera.
  - `/camera/camera_info`: Camera calibration information.
  - `/imu`: IMU sensor data.
  - `/odom`: Odometry data from the robot.
- **Published Topics**:
  - `/visual_odometry`: Visual odometry data.
  - `/odom`: Processed odometry data.
  - `/robot_position`: Estimated robot position.
  - `/camera/image_raw`: Processed camera images with visual odometry overlays.

## Algorithms and Filters
1. **ORB for Feature Extraction:** ORB is chosen for its balance between speed and accuracy, suitable for real-time visual odometry.
2. **RANSAC for Essential Matrix Estimation:** RANSAC is robust to outliers, crucial for accurate pose estimation from feature matches.
3. **Kalman Filters for State Estimation:** Kalman Filters fuse IMU data and odometry for accurate robot position and velocity estimation.
