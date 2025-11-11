# end_eff_bridge

A lightweight ROS 2 node that bridges ROS and an Arduino-based end-effector (servo prongs + force sensor) over USB. It forwards simple text commands to the Arduino and publishes parsed force readings back into ROS.

## Features
- ROS is the source of truth for the open / cp / cf presets (angles per side).
- Update presets at runtime via ROS parameters (no reflashing).
- Trigger preset poses with `/prongs/mode` or send absolute angles via `/prongs/pose_deg`.
- Publishes force in grams to `/prongs/force_g`.
- Exposes raw serial lines on `/prongs/line` for debugging.

## Topics

| Direction   | Topic                | Type                         | Description                                      |
|-------------|----------------------|------------------------------|--------------------------------------------------|
| Subscribes  | `/prongs/cmd`        | `std_msgs/String`            | Raw passthrough lines to Arduino (advanced).     |
| Subscribes  | `/prongs/mode`       | `std_msgs/String`            | `o`, `cp`, or `cf` to move to the preset.        |
| Subscribes  | `/prongs/pose_deg`   | `std_msgs/Float32MultiArray` | `[L, R]` absolute angles in degrees.             |
| Publishes   | `/prongs/line`       | `std_msgs/String`            | Raw serial lines echoed from Arduino.            |
| Publishes   | `/prongs/force_g`    | `std_msgs/Float32`           | Force sensor estimate (grams).                   |

## Parameters

| Name               | Type    | Default   | Description                                             |
|--------------------|---------|-----------|---------------------------------------------------------|
| `port`             | string  | `""`      | Serial device. If empty, node will auto-detect.        |
| `baud`             | int     | `115200`  | Serial baud rate.                                       |
| `auto_hint`        | string  | `"usb"`   | Substring used to pick a port when `port` is empty.     |
| `open_left_deg`    | double  | `150.0`   | Open preset – left servo angle (deg).                   |
| `open_right_deg`   | double  | `30.0`    | Open preset – right servo angle (deg).                  |
| `cp_left_deg`      | double  | `55.0`    | Close-block preset – left angle (deg).                  |
| `cp_right_deg`     | double  | `115.0`   | Close-block preset – right angle (deg).                 |
| `cf_left_deg`      | double  | `0.0`     | Close-full preset – left angle (deg).                   |
| `cf_right_deg`     | double  | `180.0`   | Close-full preset – right angle (deg).                  |

On (re)connect, the node pushes these presets to the Arduino as definitions so the MCU always matches ROS.

## Running

Auto-detect port (recommended)
```
ros2 run end_eff_bridge bridge
```

Or specify explicitly
```
ros2 run end_eff_bridge bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200
```

Note: Only one process can open the serial device at a time. Close other serial monitors before running the node.

## Changing presets via ROS (no reflashing)

Tweak OPEN preset
```
ros2 param set /end_eff_serial_bridge open_left_deg  160
ros2 param set /end_eff_serial_bridge open_right_deg 20
```

Tweak CP (close-block) preset
```
ros2 param set /end_eff_serial_bridge cp_left_deg   60
ros2 param set /end_eff_serial_bridge cp_right_deg 120
```

Tweak CF (close-full) preset
```
ros2 param set /end_eff_serial_bridge cf_left_deg   0
ros2 param set /end_eff_serial_bridge cf_right_deg 180
```

The bridge will push updated definitions to the Arduino automatically; you’ll see confirmation on `/prongs/line`.

## Command examples

Use presets
```
ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'o'"
ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'cp'"
ros2 topic pub -1 /prongs/mode std_msgs/String "data: 'cf'"
```

Absolute angles
```
ros2 topic pub -1 /prongs/pose_deg std_msgs/Float32MultiArray "{data: [55.0, 115.0]}"
```

Raw passthrough (advanced)
```
ros2 topic pub -1 /prongs/cmd std_msgs/String "data: 'pose L60 R120'"
```

## Reading force
Parsed force in grams:
```
ros2 topic echo /prongs/force_g
```
For debugging, also see raw lines:
```
ros2 topic echo /prongs/line
```

The Arduino emits either `~g=<val>` or `time_ms,<val>` lines; the bridge parses both and publishes the numeric value.

## Arduino command protocol (reference)

Presets (sent automatically by the node):
```
def o L<deg> R<deg>
def cp L<deg> R<deg>
def cf L<deg> R<deg>
```

Trigger presets:
```
o
cp
cf
```

Absolute pose:
```
pose L<deg> R<deg>
```

Diagnostics / plot / help:
```
plot on
plot off
force
help
```

By design, you shouldn’t need to send `def ...` yourself—change ROS params instead.
