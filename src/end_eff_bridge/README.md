# Overview

end_eff_bridge is a lightweight ROS 2 node that provides a serial interface between ROS 2 and an Arduino-based end-effector controller. It sends simple string commands (e.g. `open`, `close`, `gap 20`) to the Arduino over USB and publishes force-sensor readings back to ROS topics.

## Topics

| Direction   | Topic             | Type               | Description                                               |
|-------------|-------------------|--------------------|-----------------------------------------------------------|
| Subscribes  | `/prongs/cmd`     | `std_msgs/String`  | Commands to control the gripper (e.g. `open`, `close`, `gap <mm>`, `force`) |
| Publishes   | `/prongs/force_g` | `std_msgs/Float32` | Force sensor output in grams                              |

## Parameters

| Name | Type   | Default       | Description                     |
|------|--------|---------------|---------------------------------|
| port | string | `/dev/ttyUSB0`| Serial port connected to Arduino|
| baud | int    | `115200`      | Serial baud rate                |

## Usage

Run the node with optional parameter overrides:

```
ros2 run end_eff_bridge bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

## Example interaction

Send a command to open the gripper:

```
ros2 topic pub /prongs/cmd std_msgs/String "data: 'setLx setRx'"
```

## Commands

The commands that the gripper takes:
       o      -> open  
       cp     -> close for block (partial close)  
       cf     -> close fully  
       setL<num> -> set left servo angle  
       setR<num> -> set right servo angle  
       force  -> print force  
