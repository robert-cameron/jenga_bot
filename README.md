<div align="center">
  <h1 style="font-size:48px; margin:0.2em 0;">JengaBot</h1>
  <p style="font-size:16px; margin:0 0 0.8em 0; color:#555;">Interactive Jenga-playing robot</p>
  <img src="image/jengabot.jpg" alt="jengabot" width="400"/>
</div>

---

# Table of Contents

[1. Project Overview](#1-project-overview)  

[2. System Architecture](#2-system-architecture)  

[3. Technical Components](#31-technical-components-manipulation)  
- [3.1. Manipulation](#31-technical-components-manipulation)  
- [3.2. Computer Vision](#32-technical-components-computer-vision)  
- [3.3. Brain Node](#33-technical-components-brain-node)  
- [3.4. UI Node](#34-technical-components-ui-node)  
- [3.5 Closed Loop Operation](#35-technical-components-closed-loop-operation)  
- [3.6 Custom End-Effector](#36-technical-components-custom-end-effector)  
- [3.7 System Visualisation](#37-technical-components-system-visualisation)

[4. Installation and Setup](#4-installation-and-setup)  

[5. Running the System](#5-running-the-system)

[6. Results and Demonstration](#6-results-and-demonstration)

[7. Discussion and Future Work](#7-discussion-and-future-work)

[8. Contributors and Roles](#8-contributors-and-roles)

[9. Repo Structure](#9-repo-structure)

[10. References](#10-references)

---

# 1. Project Overview

## Background and Problem Context  
Over 40% of children spend significant hours alone at home each week, often without meaningful interaction. This isolation leads many to rely heavily on screens, which can reduce focus, hinder social development, and negatively affect emotional well‑being. Parents are increasingly concerned about finding safe and engaging alternatives that can keep their children stimulated while also supporting healthy growth.  

## Target Customers/Users  
The primary users are children who need interactive companionship during periods of solitude. Parents are the key customers, as they seek reliable solutions that reduce screen dependency and provide peace of mind. Educational institutions and after‑school programs may also benefit from such systems, using them to enhance learning and social engagement in structured environments.  

## System Purpose  
The robot is designed to provide interactive companionship and stimulating activities for children, offering a safe and engaging alternative to passive screen time. Fostering attention, creativity, and social interaction helps improve emotional well‑being and developmental outcomes. At the same time, it reassures parents that their children are meaningfully engaged even when alone at home.  

## Solution - JengaBot
JengaBot is a robot that playes Jenga against a human player. The robot is able to find a block that is suitable to remove, push it partially out, then pull and grasp it, removing it from the tower before placing the block on the top of the tower. The robot uses [computer vision](#32-technical-components-computer-vision) to find the location of the tower and the blocks that are in the tower. The robot uses [manipulation](#31-technical-components-manipulation) to execute push-pull-place actions. Further, it uses a custom [force sensor](#36-technical-components-custom-end-effector) to determine whether a block is suitably loose to remove. The [brain node](#33-technical-components-brain-node) combines and executes these components, coordinating an entire closed loop game of the Jenga against the user.

## Video
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

The object_detect node uses computer vision to determine the location of the tower. It also determines which blocks are in the tower. See [computer vision](#32-technical-components-computer-vision) for more details.

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

Based on the requested `action_type`, the node dispatches to specialised action classes, e.g. **PushMoveAction**, **PullMoveAction**.

Some actions use other actions within them. For example, a **PullMoveAction** constists of an **ApproachMoveAction**, and a sequence of other **LinearMoveAction**s. 

The node integrates with MoveIt’s **MoveGroupInterface** to plan and execute trajectories. It also sets up **collision objects** (walls, table, ceiling) in the planning scene to ensure safe motion planning. Orientation and joint constraints can be applied to enforce specific end-effector orientations or joint limits.  

The node continuously monitors goals, supports cancellation, and reports success or failure back to the client.

Movements stop instantly if force sensor sets /safety/stop to true.

### Action types

The following actions are defined as C++ classes:
- `ApproachMove` - Approach any position/block on the tower
- `PushMove` - Push a block out from the tower
- `PullMove` - Grab a block and pull it from the tower
- `PlaceMove` - Place a grabbed block on top of the tower
- `LinearMove` - Move linearly (in Cartesian space) to a position/block
- `ConstrainedMove` - Move in joint space with joint constraints
- `FreeMove` - Move in joint space without constraints

### Usage
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

---

# 3.2. Technical Components: Computer vision

Our vision pipeline is designed to detect and locate the Jenga tower, as well as determine the state of blocks within the tower. The system subscribes to the RGB image topic, processes it with OpenCV, and outputs the results using ROS2 for use in the closed loop control.

<div align="center">
  <img src="image/ComputerVision.png" alt="ComputerVision" width="600"/>
</div>

The pipeline begins with detecting the tower using ArUco markers. Given that there is an ArUco marker located on each side of the tower, two markers will always be visible to the camera. The locations of these markers are determined using solvePNP from the corners of the markers and the known dimensions of the marker. The actual location of the tower is interpolated at the perpendicular intersection point of the two visible markers. Once the location of the tower is determined, the system computes transformations and broadcasts them via TF, ensuring that the robot knows the location of the tower and the blocks that are in the tower.

Following finding the location of the tower, the system begins to detect the blocks that are in the tower. Orange rectangles (which are glued to the end of each block in the tower) are detected with an HSV color mask and contours are extracted. 

To determine the block that corresponds to each contour, the x and y coordinates (in the image) are compared to the closest corner of the tower. They are then grouped into their respective level group and horizontal position group (as coloured in the image above) using K-means and hierarchical clustering. This data is then used to populate an occupancy message which is published to the `/vision/tower` topic in the custom `Tower` message format.

```
# Tower.msg
TowerRow[] rows
```

```
# TowerRow.msg
bool pos1
bool pos2
bool pos3
```

Markers are also output to the `/vision/markers` topic in the `MarkerArray` message format for the visualisation of the blocks in RViz.


# 3.3. Technical Components: Brain node

The Brain node is the central coordinator that links perception, motion, safety, and the human player into an autonomous Jenga-playing system.. It takes in the tower state from the vision pipeline, selects safe blocks, issues the push, pick and place manipulation goals, monitors gripper force to mark immovable blocks and trigger safety stops, and manages turn-taking with the user interface. 

## Block selection Algorithm
- Reads the latest `Tower` message (described above).
- Builds a per-row occupancy model (left/centre/right).
- Maintains `_immovable_blocks` set of (row_num, pos) marked after excessive force triggers.
- Block selection strategy (`get_next_blocks()`):
  - Scan rows from index 1 upward.
  - For a row with 3 blocks: prefer centre → left → right, skipping immovables.
  - For 2 blocks with centre: try side first, then centre.
  - For 2 side blocks only: normally skip, but allow if centre was previously marked immovable.
  - Single-block rows are skipped (unstable).
  - If none found, return empty names and log a warning.
- When a block is chosen, construct TF names:
  - `push_tf = "block{row_num}{pos}b"`
  - `pull_tf = "block{row_num}{pos}f"`

<div align="center">
  <img src="image/BlockAlgorithm.png" alt="BlockAlgorithm" width="600"/>
</div>

Placement logic
- Inspect top row occupancy:
  - If top row full → place on new row above, centre back: `(place_row = top_row + 1, place_pos = 2)`.
  - Else place into the first gap on the current top row, preferring centre → left → right.
- `place_tf = "block{place_row}{place_pos}f"`

Manipulation sequence
- For each robot turn the node sends these actions to the manipulation action server:
  1. `("push_move", push_tf)`
  2. `("pull_move", pull_tf)`
  3. `("place_move", place_tf)`
  4. `("approach_move", WAITING_POS)`
- During `push_move` the node enables force checking (`_force_check_enabled = True`) and records `_current_push_block`. If a force stop occurs during push, that block is added to `_immovable_blocks` and the node selects another candidate.
- Goals are sent with an async action client; the node synchronously waits for completion using threading events.
- Rejected goals or failures:
  - During `push_move`: mark attempt unsuccessful and retry with another block.
  - During `pull_move` or `place_move`: treat as non-recoverable, log error, set `/ui/robot_turn = False`, and exit loop.

Force monitoring and safety
- Subscribes to `/prongs/force_g` (`std_msgs/Float32`).
- Configurable parameter `threshold_g` (default `50.0` g).
- On force callback:
  - If reading > `threshold_g`:
    - Log warning and publish `Bool(data=true)` once on `/safety/stop`.
    - If `_current_push_block` set, add it to `_immovable_blocks`.
- This integrates physical feedback into strategic decisions by avoiding immovable blocks.

Automatic prong closing
- A short-period timer `_try_close_prongs()` checks for subscribers to `/prongs/mode` (the serial bridge).
- On detection it publishes `"cf"` once on `/prongs/mode` to close the gripper fully.

Build instructions
```
cd ~/jenga_ws
colcon build --packages-select brain_node
source install/setup.bash
```

How to run
```
ros2 launch brain_node brain.launch.py
```

Topics and action interface

Subscriptions
- `/vision/tower` — `tower_interfaces/msg/Tower` — tower occupancy.
- `/prongs/force_g` — `std_msgs/msg/Float32` — gripper force (g).
- `/ui/player_done` — `std_msgs/msg/Bool` — player signals done.

Publications
- `/safety/stop` — `std_msgs/msg/Bool` — emits `True` when force exceeds threshold (ESTOP pulse).
- `/prongs/mode` — `std_msgs/msg/String` — sends `"cf"` once at startup to close prongs.
- `/ui/robot_turn` — `std_msgs/msg/Bool` — `True` while robot is executing moves; `False` during player turn.

Actions
- `/manipulation_action` — `manipulation/action/Manipulation` — request `push_move`, `pull_move`, `place_move`, `approach_move` using named TF frames (e.g., `block72b`).

Runtime behavior:
- On startup the node logs it is watching `/prongs/force_g` and waits for `/manipulation_action`.
- Closes prongs when `/prongs/mode` subscriber appears.
- Publishes `/ui/robot_turn = False` and waits for `/ui/player_done`.
- On player trigger: switches to robot turn, selects block, executes the four-step sequence, returns to `WAITING_POS`, sets `/ui/robot_turn = False`, and waits for next signal.
- If force exceeds `threshold_g` during push: a warning is logged, `/safety/stop` pulses `True`, and the block is added to immovable set.

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
This sensor determines whether a block can be pushed **without destabilising the tower**.  

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

The robot features two simultaneous windows used for system visualisation:
  1. A custom UI,
  2. RVIZ

The custom UI indicates the state of the game, i.e. whether it is the User's or Robot's turn to remove a block.

When the 'Start / Next Move' button is clickable, the user is free to take their turn. When it is greyed out, the robot is taking its turn.

<div align="center">
  <img src="image/window2.png" alt="window2" width="300"/>
</div>

RVIZ displays the state, position and orientation of each block in the tower, based on output from the `object_detect` node. The UR5e and attached end effector is displayed, indicative of all current joint positions and orientations. 

<div align="center">
  <img src="image/window1.png" alt="window1" width="300"/>
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
cd embedded/"end_eff/src/main.cpp"
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

### Step 1: Calibration

TODO THOMAS
- double check the list of tabs in Step 2, not sure if you move any of it to the calibration script 

### Step 2: Startup Script

Once calibrated, run the following script from within the `/jenga_bot` directory:

```
./setupRealur5e
```

This script will `colcon build` the solution and open the following tabs in the terminal:

- DriverServer
- MoveitServer
- ManipulationServer
- EndEffServer
- BrainServer
- PlayerUI
- Realsense
- Static Camera
- RealTower
- ObjectDetection

Wait up to 30 seconds for the nodes to start up. The UI and RVIZ will launch automatically when this script is run.

Once all nodes have started, run the `ros` script from the UR5e teach pendent.

The robot is now ready! Interact with it via the UI.

NOTE: alternatively, run `./setupFakeur5e` to use simulated hardware. This script was used for testing and hence, it will only run nodes necessary for manipulation - it will not run computer vision. 

### Troubleshooting

- After running `./setupRealur5e`, it is recommended to manually open/close the end effector via the UI. This ensures that the end_eff_link is running and communicating with the Arduino correctly. 
  - If the end effector does not respond, restarting the Arduino generally fixes this.
- If the robot does not move when expected, stop and restart the `ros` script from the UR5e teach pendent.
- If movement suddenly stops, it is likely that the UR5e joint limits would be exceeded by moving to the next position. Move the jenga tower into a more suitable location and restart the robot.

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

This repository is organized around ROS2 packages, Arduino code for the end effector, and supporting resources:

- `src/`: ROS2 workspace containing packages for the brain, manipulation, vision, UI, end-effector bridge, UR description, and tower message interfaces.
- `embedded/`: Arduino source code and reference images for the custom end effector electronics.
- `image/`: Diagrams and photos used in the README to illustrate system architecture, vision pipeline, and hardware.
- `solidworks/`: CAD assemblies and parts for the robot’s custom gripper and mechanical components.
- Top-level helper scripts (`setup*.sh`, `fake_tower.sh`, `real_tower.sh`) streamline environment setup and tower configuration for testing.

NOTE: after running `colcon build` from the `/jenga_bot` directory, expect the following additional folders to appear
- `build/`
- `log/`
- `install/`

---

# 10. References

---
