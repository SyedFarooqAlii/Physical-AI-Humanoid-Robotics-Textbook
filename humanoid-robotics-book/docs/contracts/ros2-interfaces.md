---
sidebar_position: 1
---

# ROS 2 Interface Contracts for Physical AI & Humanoid Robotics

## Overview

This document defines the complete set of ROS 2 interfaces for the Physical AI & Humanoid Robotics system, including topics, services, and actions across all modules.

## Message Types

### JointState
- **Properties**:
  - `name`: Array of joint names
  - `position`: Array of joint positions (float64)
  - `velocity`: Array of joint velocities (float64)
  - `effort`: Array of joint efforts (float64)

### Imu
- **Properties**:
  - `angular_velocity`: x, y, z components (float64)
  - `linear_acceleration`: x, y, z components (float64)
  - `orientation`: x, y, z, w quaternion (float64)

### LaserScan
- **Properties**:
  - `angle_min`, `angle_max`: Scan angle range (float32)
  - `angle_increment`: Angular resolution (float32)
  - `time_increment`: Time between measurements (float32)
  - `scan_time`: Time between scans (float32)
  - `range_min`, `range_max`: Range limits (float32)
  - `ranges`, `intensities`: Array of measurements (float32)

### Twist
- **Properties**:
  - `linear`: x, y, z velocity components (float64)
  - `angular`: x, y, z angular velocity components (float64)

## ROS 2 Topics

### Core Topics
- `/joint_states`: Joint position, velocity, and effort feedback (sensor_msgs/JointState)
  - QoS: Reliable, Volatile, Keep Last (depth: 1)
- `/imu/data`: Inertial measurement unit data (sensor_msgs/Imu)
  - QoS: Reliable, Volatile, Keep Last (depth: 10)
- `/scan`: 2D laser scan data (sensor_msgs/LaserScan)
  - QoS: Reliable, Volatile, Keep Last (depth: 10)
- `/cmd_vel`: Velocity commands for robot base (geometry_msgs/Twist)
  - QoS: Reliable, Volatile, Keep Last (depth: 1)
- `/tf`: Transform tree for coordinate frames (tf2_msgs/TFMessage)
  - QoS: Reliable, Volatile, Keep Last (depth: 100)
- `/tf_static`: Static transform tree (tf2_msgs/TFMessage)
  - QoS: Reliable, Transient Local, Keep Last (depth: 100)

## ROS 2 Services

### Core Services
- `/get_joint_positions`: Get current joint positions (std_srvs/Empty → control_msgs/JointTrajectoryControllerState)
- `/set_control_mode`: Set control mode (std_msgs/String → std_srvs/Trigger)
- `/calibrate_sensors`: Calibrate all robot sensors (std_srvs/Trigger → std_srvs/Trigger)

## ROS 2 Actions

### Core Actions
- `/follow_joint_trajectory`: Execute joint trajectory (control_msgs/FollowJointTrajectory)
- `/move_base`: Navigate robot to pose (nav2_msgs/NavigateToPose)
- `/pick_object`: Pick up an object (manipulation_msgs/PickupObject)

## Module-Specific Interfaces

### Module 1: The Robotic Nervous System (ROS 2)
- **Sensor Processor**: `/sensor_raw`, `/sensor_processed`, `/calibrate_sensor`, `/process_sensor_stream`
- **Joint Controller**: `/joint_commands`, `/joint_feedback`, `/switch_control_mode`, `/execute_trajectory`
- **State Estimator**: `/robot_pose`, `/robot_velocity`, `/reset_estimator`, `/estimate_state`

### Module 2: The Digital Twin (Gazebo & Unity)
- **Gazebo Bridge**: `/gazebo/model_states`, `/gazebo/get_model_state`, `/gazebo/reset_simulation`, `/gazebo/spawn_model`
- **Unity Bridge**: `/unity/robot_state`, `/unity/user_input`, `/unity/load_scene`, `/unity/teleop_command`

### Module 3: The AI-Robot Brain (NVIDIA Isaac)
- **Isaac Detection**: `/isaac/detections`, `/isaac/relocalize`, `/isaac/navigate_to_pose`
- **Cognitive Controller**: `/ai/intentions`, `/ai/plan_task`, `/ai/execute_behavior`

### Module 4: Vision-Language-Action (VLA)
- **VLM Interpreter**: `/vlm/understanding`, `/vlm/query`, `/vlm/execute_command`
- **VLA Executor**: `/vla/actions`, `/vla/generate_action`, `/vla/execute_task`

### Capstone: Autonomous Humanoid
- **Humanoid Coordinator**: `/humanoid/intentions`, `/capstone/execute_command`, `/capstone/autonomous_task`

## QoS Profile Guidelines

- **Reliable Topics**: Use reliable reliability for critical data
- **Volatile Durability**: Use for real-time streaming data
- **Transient Local**: Use for static data that needs to be available to late-joining nodes
- **History Policy**: Keep last N messages based on real-time requirements