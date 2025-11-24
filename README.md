# JengaBot

## Table of Contents
- [Background and Customers](#background-and-customers)
- [System Structure (Node Graph)](#system-structure-node-graph)
- [End-Effector Configuration](#end-effector-configuration)
- [Installation and Setup](#installation-and-setup)
- [Running the System](#running-the-system)
- [Results and Discussion](#results-and-discussion)
- [Contributors and Roles](#contributors-and-roles)
- [File/Repo Structure](#filerepo-structure)
- [References](#references)

---

## Background and Customers
- **Problem Context**: Briefly describe the background of the project and the motivation.  
- **Target Customers/Users**: Who is the intended end-user or customer?  
- **System Purpose**: What problem does the robot solve?  

---

## System Structure (Node Graph)
- **ROS2 Node Graph**: Insert diagram (from `rqt_graph` or custom schematic).  
- **Node Functions**: Short description of each node’s role.  
- **Topics/Services/Actions**: List and explain key communication interfaces.  
- **Closed-Loop Behavior**: Provide a behavior tree or state machine diagram.  

---

## End-Effector Configuration

**Lightweight 3D-Printed Frame**  
The JengaBot end-effector is built from a **lightweight 3D-printed structure**.  
It includes **two grippers** and **two servos**.  

![End Effector CAD Render](docs/end_effector_cad.png)

**Gripper Motion**  
Horizontal movement of the grippers is achieved through a **gear–rack mechanism**.  
This motion is **directly driven by servo rotation**.  

**Force Sensor Integration**  
One gripper tip is equipped with a **flexible force sensor**.  
This sensor determines whether a block can be pushed **without destabilizing the tower**.  

![Force Sensor Placement](docs/force_sensor.png)

**Mechanical Design Iterations**  
The mechanical structure was designed in **SolidWorks**.  
STL files were exported for **3D printing**.  
Due to geometric constraints and issues found during development, the design underwent **multiple iterations**.  

**Servo Control**  
Servos are controlled by an **Arduino**.  
A **servo expansion board** is used to optimize wiring and save I/O pins.  

![Servo Wiring Diagram](docs/servo_wiring.png)

**Mounting Method**  
The end-effector is mounted using an **existing slot fixture**, ensuring stable integration with the robot system.  



---

## Installation and Setup
- **Dependencies**: List required packages, libraries, and versions.  
- **Workspace Setup**: Step-by-step instructions to build the ROS2 workspace.  
- **Hardware Setup**: UR5e connection, camera, Teensy, etc.  
- **Environment Variables/Calibration**: Any required configuration files or calibration steps.  

---

## Running the System
- **Launch Instructions**: Provide a single command to start the system.  
- **Example Commands**: e.g. `ros2 launch project_name bringup.launch.py`.  
- **Expected Behavior**: Describe what the user should see.  
- **Troubleshooting Notes**: Optional tips for common issues.  

---

## Results and Discussion
- **Performance**: How the system meets design goals.  
- **Quantitative Results**: Accuracy, repeatability, robustness.  
- **Demonstration Media**: Photos, figures, or videos of operation.  
- **Discussion**: Challenges faced, solutions, and opportunities for improvement.  

---

## Contributors and Roles
- **Robert Cameron** — 
- **Thomas Crundwell** —  
- **Akhil Govan** —
- **Haoran Wen** — End effector

---

## File/Repo Structure



---

## References
- External libraries, tutorials, or prior codebases used.  
- Acknowledgements to demonstrators, peers, or collaborators.  
