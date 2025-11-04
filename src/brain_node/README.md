# brain_node

## Overview

brain_node coordinates high-level task logic for the Jenga robot. It waits for the player’s start signal (SPACE via /ui/player_done), listens for block positions from the vision system, and orchestrates the manipulation and end-effector nodes to perform autonomous pick-and-place actions.

## Behaviour Summary

- Idle until the user presses SPACE (/ui/player_done → True).
- Waits for /vision/blocks to publish block poses.
- Executes a pick-sequence for each new pose:
  1. Opens gripper (/prongs/cmd → "gap 30")
  2. Approaches target pose using Manipulation action
  3. Closes gripper (/prongs/cmd → "gap 5")
  4. Optionally lifts object slightly
- Repeats automatically when new vision data arrives.

## Topics & Actions

| Interface     | Name                    | Type                         | Description                                   |
|---------------|-------------------------|------------------------------|-----------------------------------------------|
| Subscribes    | /ui/player_done         | std_msgs/Bool                | Player input (SPACE = start)                  |
| Subscribes    | /vision/blocks          | geometry_msgs/PoseArray      | Detected block poses                          |
| Subscribes    | /prongs/force_g         | std_msgs/Float32             | Feedback from force sensor                    |
| Publishes     | /prongs/cmd             | std_msgs/String              | Commands to open/close gripper                |
| Action Client | /manipulation_action    | manipulation/Manipulation    | Sends approach/lift goals to MoveIt           |

## Parameters

| Parameter        | Default   | Description                                 |
|------------------|----------:|---------------------------------------------|
| open_gap_mm      | 30.0      | Fully open gap                              |
| approach_gap_mm  | 18.0      | Near-block gap                              |
| close_gap_mm     | 5.0       | Grip gap                                    |
| lift_dz          | 0.05      | Vertical lift distance (m)                  |
| approach_action  | linear_move | Manipulation action type for approach    |
| lift_action      | free_move | Manipulation action type for lift           |
| result_timeout_s | 10.0      | Timeout per manipulation goal               |

## Launch

Run: