# RMTT_ROS

ROS workspace for DJI Tello EDU / RMTT with a unified multi-input control pipeline.

This repository integrates:

- `tello_driver`: ROS driver for Tello EDU
- `tello_control_system`: keyboard control, control mux, and helper nodes
- `h264_image_transport`: H.264 image transport support

The current control stack routes multiple velocity sources through a single arbitration layer:

- keyboard input
- joystick / gamepad input
- external agent input

The final velocity command is selected by a control multiplexer with:

- priority arbitration: `keyboard > joy > agent`
- source timeout protection
- zero-command fallback when no valid source is active
- driver-side watchdog protection

## Features

- Unified launch entry for the whole Tello control pipeline
- Multi-input velocity arbitration through a dedicated MUX node
- Keyboard teleoperation for manual control
- Optional joystick teleoperation
- Agent interface for autonomy / learning-based control
- Driver watchdog for safer command timeout behavior
- Root-level `.gitignore`, `LICENSE`, and `NOTICE` for repository publishing

## Repository layout

```text
rmtt_ws/
├── src/
│   ├── tello_driver/
│   ├── tello_control_system/
│   └── h264_image_transport/
├── devel/
├── build/
├── .gitignore
├── LICENSE
├── NOTICE
└── README.md
```

## Control architecture

```text
keyboard_control_node  ---> /tello/cmd_vel_keyboard ---\

gamepad_marshall_node ---> /tello/cmd_vel_joy ------- > tello_control_mux_node ---> /tello/cmd_vel ---> tello_driver_node

agent node ------------> /tello/cmd_vel_agent ------/
```

MUX behavior:

- keyboard priority: `3`
- joy priority: `2`
- agent priority: `1`
- keyboard timeout: `0.3s`
- joy timeout: `0.5s`
- agent timeout: `0.5s`
- arbitration rate: `30 Hz`

Debug topic:

- `/tello/mux/active_source`

## Main topics

### Velocity topics

- `/tello/cmd_vel_keyboard`
- `/tello/cmd_vel_joy`
- `/tello/cmd_vel_agent`
- `/tello/cmd_vel`

### Command topics

- `/tello/takeoff`
- `/tello/land`
- `/tello/reset`
- `/tello/flip`
- `/tello/fast_mode`

### State / image topics

- `/tello/status`
- `/tello/image_raw`
- `/tello/image_raw/compressed`

## Environment

- Ubuntu + ROS Noetic
- Python 3
- Catkin workspace

## Build

```bash
cd /home/qczk/rmtt_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Launch

### Recommended: full system launch

```bash
cd /home/qczk/rmtt_ws
source devel/setup.bash
roslaunch tello_control_system tello_system.launch
```

### Launch with joystick enabled

```bash
cd /home/qczk/rmtt_ws
source devel/setup.bash
roslaunch tello_control_system tello_system.launch enable_joy:=true joy_dev:=/dev/input/js0
```

### Driver-only launch

```bash
cd /home/qczk/rmtt_ws
source devel/setup.bash
roslaunch tello_driver tello_node.launch
```

## Agent integration

External autonomy modules can publish directly to:

- `/tello/cmd_vel_agent`

Message type:

- `geometry_msgs/Twist`

Minimal example:

```python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("simple_agent")
pub = rospy.Publisher("/tello/cmd_vel_agent", Twist, queue_size=20)
rate = rospy.Rate(30)

while not rospy.is_shutdown():
	cmd = Twist()
	cmd.linear.x = 0.3
	pub.publish(cmd)
	rate.sleep()
```

When keyboard or joystick is active, the MUX will temporarily override agent output according to priority.

## Validation / debugging

Check active control source:

```bash
rostopic echo /tello/mux/active_source
```

Check final command output:

```bash
rostopic echo /tello/cmd_vel
```

Check running nodes:

```bash
rosnode list
```

## Publishing notes

This repository is a multi-package workspace. License information may differ by package:

- `tello_driver`: Apache-2.0
- `tello_control_system`: MIT
- `h264_image_transport`: BSD

See `LICENSE` and `NOTICE` at the repository root for repository-level guidance.

