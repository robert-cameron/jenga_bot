# JengaBot

## Table of Contents
- [Background and Customers](#background-and-customers)
- [System Structure (Node Graph)](#system-structure-node-graph)
- [Node: Brain Node](#brain-node)
- [End-Effector Configuration](#end-effector-configuration)
- [Installation and Setup](#installation-and-setup)
- [Running the System](#running-the-system)
- [Results and Discussion](#results-and-discussion)
- [Contributors and Roles](#contributors-and-roles)
- [File/Repo Structure](#filerepo-structure)
- [References](#references)

---

# Background and Customers

## Background and Problem Context  
Over 40% of children spend significant hours alone at home each week, often without meaningful interaction. This isolation leads many to rely heavily on screens, which can reduce focus, hinder social development, and negatively affect emotional well‑being. Parents are increasingly concerned about finding safe and engaging alternatives that can keep their children stimulated while also supporting healthy growth.  

## Target Customers/Users  
The primary users are children who need interactive companionship during periods of solitude. Parents are the key customers, as they seek reliable solutions that reduce screen dependency and provide peace of mind. Educational institutions and after‑school programs may also benefit from such systems, using them to enhance learning and social engagement in structured environments.  

## System Purpose  
The robot is designed to provide interactive companionship and stimulating activities for children, offering a safe and engaging alternative to passive screen time. Fostering attention, creativity, and social interaction helps improve emotional well‑being and developmental outcomes. At the same time, it reassures parents that their children are meaningfully engaged even when alone at home.  

[Insert the video here]

---

# System Structure (Node Graph)
- **ROS2 Node Graph**: Insert diagram (from `rqt_graph` or custom schematic).  
- **Node Functions**: Short description of each node’s role.  
- **Topics/Services/Actions**: List and explain key communication interfaces.  
- **Closed-Loop Behavior**: Provide a behavior tree or state machine diagram.  

---

# Technical Components: Computer vision

Our vision pipeline is designed to detect and interpret the state of blocks within a tower structure using a depth camera and ArUco markers. The system subscribes to both RGB and depth image topics, processes them with OpenCV, and integrates the results into ROS2 for downstream robotic control.

The pipeline begins with **image acquisition**, where RGB and aligned depth frames are captured. Using the `cv_bridge`, these frames are converted into OpenCV images for further processing. ArUco markers are detected to establish reference frames and orientations. Once markers are identified, the system computes transformations and broadcasts them via TF, ensuring that the robot has a consistent world model.

A critical part of the pipeline is **block detection and clustering**. Green regions are segmented in HSV color space, contours are extracted, and positions are mapped into global coordinates. K-means and hierarchical clustering are applied to group blocks by horizontal and vertical positions, reconstructing the tower’s occupancy state. This information is published as `PoseArray`, `MarkerArray`, and custom `Tower` messages, enabling the robot to reason about which blocks can be pushed and how the tower is structured.

Key code excerpt for marker pose estimation:
```python
success, rvec, tvec = cv2.solvePnP(obj_points, img_points, cameraMatrix, distCoeffs)
if success:
    rvecs.append(rvec)
    tvecs.append(tvec)

```
---


# Technical Components: Manipulation

---


# Technical Components: Brain node

## Overview

This node monitors gripper force and raises a safety stop when the measured force exceeds a configured threshold.

## Behavior
- Subscribes to `/prongs/force_g` (std_msgs/msg/Float32) — gripper force in grams.  
- Compares each reading to a configured threshold (default: 80 g).  
- Publishes `True` on `/safety/stop` (std_msgs/msg/Bool) while the force is above the threshold (ESTOP).  
- Latches internally:
  - On the first crossing above the threshold, logs a warning.
  - Continues publishing `True` while force remains above the threshold.
  - Optionally resets the latch when force drops below a lower band (for example, 80% of the threshold) if configured.
- Does not command actuators or interact with MoveIt; it only raises a safety flag.

## Build instructions

From your ROS 2 workspace:
```
cd ~/ros2_ws
colcon build --packages-select brain
source install/setup.bash
```

(Note: use setup.bash, not setup.bas.)

## How to run

Assuming all hardware drivers are running and `/prongs/force_g` is being published:
```
ros2 run brain brain
```
Override the threshold via parameters:
```
ros2 run brain brain --ros-args -p threshold_g:=100.0
```
## Topics

### Subscriptions
| Topic | Type | Description |
|---|---:|---|
| `/prongs/force_g` | `std_msgs/msg/Float32` | Force from gripper in grams |

### Publications
| Topic | Type | Description |
|---|---:|---|
| `/safety/stop` | `std_msgs/msg/Bool` | `True` when force exceeds threshold (ESTOP) |

If you need the node to reset automatically, check the node parameters for a hysteresis or reset-band option (e.g., a fraction of the threshold) and set it appropriately.

---


# Technical Components: UI Node

## Overview

ui_node provides a minimal terminal-based interface for human control. It reads keyboard input and publishes high-level commands to the robot during testing and gameplay.

## Keyboard Mapping

| Key      | Published Topic   | Message           | Description                         |
|----------|-------------------|-------------------|-------------------------------------|
| SPACE    | /ui/player_done   | Bool(data=True)   | Starts or signals robot turn        |
| O / o    | /prongs/cmd       | "open"            | Opens gripper                       |
| G / g    | /prongs/cmd       | "close"           | Closes gripper                      |
| F / f    | /prongs/cmd       | "force"           | Requests force reading              |
| Q / q    | (none)            | (none)            | Quits UI node                       |

## Topics

| Direction | Topic            | Type             | Description                          |
|-----------|------------------|------------------|--------------------------------------|
| Publishes | /ui/player_done  | std_msgs/Bool    | Player "start" signal                |
| Publishes | /prongs/cmd      | std_msgs/String  | Manual end-effector commands         |

---


# Technical Components: System Visualisation

The robot features two simultaneous user interaction windows. The first window displays the state and posture of each block, indicating whether a block can be pushed and showing their relative positions. The second window enables basic operations and consolidates input and output states into a single interface, rather than presenting them as tedious, line‑by‑line terminal code. 

![window1](image/window1.png)

![window2](image/window2.png)

# Technical Components: Customized End-Effector 

## Features

**Lightweight 3D-Printed Frame**  
The JengaBot end-effector is built from a **lightweight 3D-printed structure**.  
It includes **two grippers** and **two servos**.  

**Gripper Motion**  
Horizontal movement of the grippers is achieved through a **gear–rack mechanism**.  
This motion is **directly driven by servo rotation**.  

**Force Sensor Integration**  
One gripper tip is equipped with a **flexible force sensor**.  
This sensor determines whether a block can be pushed **without destabilizing the tower**.  

**Servo Control**  
Servos are controlled by an **Arduino**.  
A **servo expansion board** is used to optimize wiring and save I/O pins.  

**Mounting Method**  
The end-effector is mounted using an **existing slot fixture**, ensuring stable integration with the robot system.  

## Iterations

Due to geometric constraints and issues found during development, the design underwent **multiple iterations**. 

Version 1.0: Original design (See figure below) 

![End effector v1](image/endeff1.png)

Version 2.0:  

Enlarged the servo mounting holes and optimized the structure to improve the convenience and accuracy of FDM printing.

Version 3.0 (Final):  

Adjusted the orientation of the mount to better align with the kinematics code, positioning the gripper further forward to reduce obstruction during motion.

![End effector v3](image/endeff2.png)

![Engineering Drawing](image/endeffdrawing.png)


---

# Installation and Setup
- **Dependencies**: List required packages, libraries, and versions.  
- **Workspace Setup**: Step-by-step instructions to build the ROS2 workspace.  
- **Hardware Setup**: UR5e connection, camera, Teensy, etc.  
- **Environment Variables/Calibration**: Any required configuration files or calibration steps.  

---

# Running the System
- **Launch Instructions**: Provide a single command to start the system.  
- **Example Commands**: e.g. `ros2 launch project_name bringup.launch.py`.  
- **Expected Behavior**: Describe what the user should see.  
- **Troubleshooting Notes**: Optional tips for common issues.  

---

# Results and Discussion
- **Performance**: How the system meets design goals.  
- **Quantitative Results**: Accuracy, repeatability, robustness.  
- **Demonstration Media**: Photos, figures, or videos of operation.  
- **Discussion**: Challenges faced, solutions, and opportunities for improvement.  

---

# Contributors and Roles
- **Robert Cameron** — 
- **Thomas Crundwell** —  
- **Akhil Govan** —
- **Haoran Wen** — End effector

---

# References

# References
- External libraries, tutorials, or prior codebases used.  
- Acknowledgements to demonstrators, peers, or collaborators.  
