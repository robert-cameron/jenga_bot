<div align="center">
  <h1 style="font-size:48px; margin:0.2em 0;">JengaBot</h1>
  <p style="font-size:16px; margin:0 0 0.8em 0; color:#555;">Interactive Jenga-playing robot</p>
  <img src="image/jengabot.jpg" alt="jengabot" width="400"/>
</div>

---

# Table of Contents

1. [Project Overview](#1-project-overview)  

2. [System Architecture](#2-system-architecture)  

3. [Technical Components](#31-technical-components-manipulation)  
   3.1. [Manipulation](#31-technical-components-manipulation)  
   3.2. [Computer Vision](#32-technical-components-computer-vision)  
   3.3. [Brain Node](#33-technical-components-brain-node)  
   3.4. [UI Node](#34-technical-components-ui-node)  
   3.5 [Closed Loop Operation](#35-technical-components-closed-loop-operation)  
   3.6 [Custom End-Effector](#36-technical-components-custom-end-effector)  
   3.7 [System Visualisation](#37-technical-components-system-visualisation)

4. [Installation and Setup](#4-installation-and-setup)  

5. [Running the System](#5-running-the-system)

6. [Results and Demonstration](#6-results-and-demonstration)

7. [Discussion and Future Work](#7-discussion-and-future-work)

8. [Contributors and Roles](#8-contributors-and-roles)

9. [Repo Structure](#9-repo-structure)

10. [References](#10-references)


---

# 1. Project Overview

## Background and Problem Context  
Over 40% of children spend significant hours alone at home each week, often without meaningful interaction. This isolation leads many to rely heavily on screens, which can reduce focus, hinder social development, and negatively affect emotional well‑being. Parents are increasingly concerned about finding safe and engaging alternatives that can keep their children stimulated while also supporting healthy growth.  

## Target Customers/Users  
The primary users are children who need interactive companionship during periods of solitude. Parents are the key customers, as they seek reliable solutions that reduce screen dependency and provide peace of mind. Educational institutions and after‑school programs may also benefit from such systems, using them to enhance learning and social engagement in structured environments.  

## System Purpose  
The robot is designed to provide interactive companionship and stimulating activities for children, offering a safe and engaging alternative to passive screen time. Fostering attention, creativity, and social interaction helps improve emotional well‑being and developmental outcomes. At the same time, it reassures parents that their children are meaningfully engaged even when alone at home.  

[Insert the video here]

---

# 2. System Architecture

## Core Functionality
This ROS2-based robot plays Jenga by combining **computer vision**, **decision-making**, and **force-sensitive manipulation**.  
It identifies safe blocks, executes push-pull-place actions, and interacts with a human via a UI.

## System Architecture Diagram

A full System Architecture diagram is shown below, detailing the implemented ROS2 packages, nodes, topics, actions. Interactions between nodes are also indicated.

<div align="center">
  <img src="image/architecture.jpeg" alt="structure" width="1000"/>
</div>

## State Machine Diagram

A State Machine diagram is shown below, indicating the logical flow of the program. This logic is largely implemented within the `brain` node.

<div align="center">
  <img src="image/state_diagram.jpeg" alt="structure" width="500"/>
</div>

## Packages & Nodes

### `ui_node`

The UI allows users to interface with the robot. 

Users are able to:
- begin JengaBot's turn (start push/pull/place sequence)
- manually open, fully and partially close the end effector
- view the current force sensor output

### `brain_node`

The brain node dictates the central logic, decision-making and sequencing. It computes the optimal blocks to remove, and listens to the force sensor to determine whether a block is removable. The brain node also calls **Push**, **Pull** and **Place** moves via the `manipulation_action` action server.

### `object_detect`

The object_detect node uses computer vision to determine the location of the tower. It also determines which blocks are in the tower. 

### `manipulation`

The manipulation node exposes the `manipulation_action` action server. It defines various move types which are used to manipulate the UR5e.

### `end_eff_bridge`

The end_eff_bridge node interfaces with the Arduino via serial communication. It broadcasts the force sensor reading, and controls the positions of the servos.

## Custom Message Types

### `manipulation_action`

```
string action_type
geometry_msgs/Pose pose   # optional, must include either pose or tf
string tf                 # optional, TF frame name

bool result
string feedback
```

- action_type: Defines what manipulation to perform

  e.g. "push_move", "place_move", "approach_move", "linear_move", etc.

- pose: A 6-DoF target pose (position + orientation)

- tf: Name of a TF frame whose pose should be used instead of a fixed Pose

- result: true if the manipulation succeeded; false if it failed or was aborted

- feedback: provides updates of path planning, movement etc.

---

# 3.1. Technical Components: Manipulation

The **Manipulation Node** is responsible for executing robot arm actions using ROS 2, MoveIt, and TF2.  
It provides an **action server** (`manipulation_action`) that accepts goals specifying either a target pose or a TF frame.  
Based on the requested `action_type`, the node dispatches to specialized action classes such as **PushMoveAction**, **PullMoveAction**, **PlaceMoveAction**, **FreeMoveAction**, **LinearMoveAction**, and **ApproachMoveAction**.  

Some actions use other actions within them. For example, a **PullMoveAction** constists of an **ApproachMoveAction**, and a sequence of other **LinearMoveAction**s. 

The `brain` node calls these actions based on its decicion making algorithm.

Actions can also be initiated directly from the terminal by specifying either a pose or tf:

```
# send end effector to a specified position
ros2 action send_goal /manipulation_action manipulation/action/Manipulation  "{action_type: 'linear_move', pose: {position: {x: 0.45, y: 0.45, z: 0.275}, orientation: {x: 0.0, y: 0.0, z: -0.382683, w: 0.923880}}}"

# send end effector to the tf named 'block 23f'
ros2 action send_goal /manipulation_action manipulation/action/Manipulation  "{action_type: 'linear_move', tf: 'block23f'}"

# execute a push move on the block at the tf named 'block 23f'
ros2 action send_goal /manipulation_action manipulation/action/Manipulation  "{action_type: 'push_move', tf: 'block23f'}"
```

The node integrates with MoveIt’s **MoveGroupInterface** to plan and execute trajectories.  
It also sets up **collision objects** (walls, table, ceiling) in the planning scene to ensure safe motion planning.  
Orientation and joint constraints can be applied to enforce specific end-effector orientations or joint limits.  
The node continuously monitors goals, supports cancellation, and reports success or failure back to the client.

### Key Features
- **Action server** for manipulation goals (`manipulation_action`).
- **Multiple action types**: push, pull, place, free, constrained, linear, approach.
- **Safety stopping**: movements stop instantly if force sensor sets /safety/stop to true.
- **TF integration**: transforms target poses from TF frames into world coordinates.
- **Collision-aware planning**: adds walls, table, and ceiling to the planning scene.
- **Constraints**: orientation and joint constraints for safe and precise motion.

---

# 3.2. Technical Components: Computer vision

Our vision pipeline is designed to detect and interpret the state of blocks within a tower structure using a depth camera and ArUco markers. The system subscribes to both RGB and depth image topics, processes them with OpenCV, and integrates the results into ROS2 for downstream robotic control.

<div align="center">
  <img src="image/ComputerVision.png" alt="ComputerVision" width="600"/>
</div>

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


# 3.3. Technical Components: Brain node

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


# 3.4. Technical Components: UI Node

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

# 3.5. Technical Components: Closed Loop operation


---

# 3.6. Technical Components: Custom End-Effector 

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

**Version 0.0 (Prototype for MVP)** 

This is a simple prototype, only functional for pushing the blocks and providing the end effector pose for research and tests. 

[missing images]

**Version 1.0:** 

Original design (See figure below) 

<div align="center">
  <img src="image/endeff1.png" alt="End effector v1" width="300"/>
</div>

**Version 2.0:**  

Enlarged the servo mounting holes and optimized the structure to improve the convenience and accuracy of FDM printing.

**Version 3.0 (Final):**  

Adjusted the orientation of the mount to better align with the kinematics code, positioning the gripper further forward to reduce obstruction during motion.

<div align="center">
  <img src="image/endeff2.png" alt="End effector v3" width="300"/>
</div>

<div align="center">
  <img src="image/endeffdrawing.png" alt="Engineering Drawing" width="300"/>
</div>

Here's the photo of actual end effector: 

<div align="center">
  <img src="image/endeff3.jpg" alt="endeff3" width="300"/>
</div>

---

# 3.7. Technical Components: System Visualisation

The robot features two simultaneous user interaction windows. 

The first window displays the state and posture of each block, indicating whether a block can be pushed and showing their relative positions. 

<div align="center">
  <img src="image/window1.png" alt="window1" width="300"/>
</div>

The second window enables basic operations and consolidates input and output states into a single interface, rather than presenting them as tedious, line‑by‑line terminal code. 

<div align="center">
  <img src="image/window2.png" alt="window2" width="300"/>
</div>

--- 

# 4. Installation and setup

1. Prerequisites
- OS: Ubuntu 22.04  
- ROS 2: Humble 
- Build tools: `colcon`, `python3-colcon-common-extensions`, `git`  
- UR stack: `ur_robot_driver`, `ros2_control`, MoveIt 2  
- Tools: `rviz2`, `tf2_ros`  

Follow the official ROS 2 Humble install guide, then install the UR driver and MoveIt packages on the same machine that will run the JengaBot stack.

2. Workspace setup
- Create workspace and clone the repo:
```bash
mkdir -p ~/jenga_ws/src
cd ~/jenga_ws/src
git clone https://github.com/robert-cameron/jenga_bot.git jenga_bot
cd ~/jenga_ws
```
- Source ROS 2 and build:
```bash
source /opt/ros/humble/setup.bash
cd ~/jenga_ws
colcon build
source ~/jenga_ws/install/setup.bash
```
- Key packages available after build: `brain_node`, `manipulation`, `object_detect`, `ur_description_custom`, `ur_moveit_config_custom`, `end_eff_bridge`, `ui_node`.

3. Hardware setup
3.1 UR5e and network
- Configure ROS2 Humble and the UR5e robot according to the documentation abd ensure both systems are on the same network.


3.2 Camera and vision
- Mount an RGB-D camera with a clear view of the tower and the ArUco Markers.
- The tower scripts (`real_tower.sh` / `fake_tower.sh`) define:
```bash
WORLD_FRAME="world"
TABLE_FRAME="table"
TOWER_BASE_FRAME="tower_base"
TOWER_X=0.6
TOWER_Y=0.2
TOWER_Z=0.03
TOWER_YAW_DEG=45
```
- Edit these values so `tower_base` matches the physical tower centre (after hand–eye calibration). As long as `/vision/tower` publishes in the robot frame (or a fixed TF exists), the brain node will operate.

3.3 End-effector (gripper + force sensor)
- Print the end-effector body (located in the SolidWorks files folder)
- Attach the 2 DSS P05 Servos and the RP-S5-RT force sensor to the end-effector as shown above.
- Firmware (PlatformIO, Arduino Nano):
- Download the code from the following file:
```bash
cd Embedded/"MTRN4231 EndEff/src/main.cpp"
```
Connect the Components according to the Wiring Diagram Below:
<div align="center">
  <img src="image/WiringDiagram.png" alt="WiringDiagram" width="600"/>
</div>

- Firmware functions: drive servos to preset positions and stream force readings (grams) over serial.
- Wiring: mount the Arduino, connect servos to PWM pins defined in `src/main.cpp`, wire force sensor as per schematic, and power from a regulated supply (shared ground with robot/PC).
- Serial bridge: connect the microcontroller to the ROS PC via USB and confirm device path:
```bash
ls /dev/ttyUSB* /dev/ttyACM*
```
- Default bridge command:
```bash
ros2 run end_eff_bridge bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```
- The bridge publishes `/prongs/force_g` (`std_msgs/Float32`) and listens on `/prongs/mode` (`std_msgs/String`) for `"o"`, `"cp"`, `"cf"`.

4. Environment, configuration, and calibration
- Always source ROS and the workspace before running scripts:
```bash
source /opt/ros/humble/setup.bash
source ~/jenga_ws/install/setup.bash
```

- Tower TF and geometry (frames, block sizes, layers) are configured in `real_tower.sh` / `fake_tower.sh`. Adjust to match your physical setup.
- Force threshold parameter on the brain node defaults to 50.0 g. 
- Hand–eye calibration: ensure TF connectivity between UR5e base, table/world, and tower frames. No calibration code is required in this repo if TFs are published correctly.


# 5. Running the System
- **Launch Instructions**: Provide a single command to start the system.  
- **Example Commands**: e.g. `ros2 launch project_name bringup.launch.py`.  
- **Expected Behavior**: Describe what the user should see.  
- **Troubleshooting Notes**: Optional tips for common issues.  

---

# 6. Results and Demonstration
- **Performance**: How the system meets design goals.  
- **Quantitative Results**: Accuracy, repeatability, robustness.  
- **Demonstration Media**: Photos, figures, or videos of operation.  
- **Discussion**: Challenges faced, solutions, and opportunities for improvement.  

---

# 7. Discussion and Future Work

---

# 8. Contributors and Roles
- **Robert Cameron** - Manipulation
- **Thomas Crundwell** - Vision
- **Akhil Govan** - Brain
- **Haoran Wen** - End effector

---

# 9. Repo structure


---

# 10. References

---
