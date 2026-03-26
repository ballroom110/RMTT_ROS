# tello_control_system

A ROS Noetic package for teleoperating a Tello (RMTT) drone via keyboard, with a fake odometry estimator (velocity integration) for RViz visualization.

## 🚀 System Overview

This package provides:

- **Keyboard teleop node** (`keyboard_control_node.py`) to control velocity incrementally
- **Fake odometry node** (`tello_odom_node.py`) that integrates velocity from `/tello/status` into `/tello/odom` + `/tello/path`
- **Launch file** to start the entire system with a single command
- **Configurable parameters** in `config/params.yaml`

## 🧱 Build

```bash
cd ~/rmtt_ws
catkin_make
source devel/setup.bash
```

## ▶️ Run

```bash
roscore
roslaunch tello_control_system bringup.launch
```

## 🎮 Keyboard Controls

- **W/S**: forward/back
- **A/D**: left/right
- **R/F**: up/down
- **Q/E**: yaw left/right
- **T**: takeoff
- **L**: land
- **H**: hover (zero velocity)
- **SPACE**: emergency stop (zero velocity + repeated commands)
- **Z**: slow mode (0.5x)
- **C**: fast mode (1.5x)

## ⚠️ Safety Notes

- Velocities decay automatically when no keys are pressed.
- If no key is pressed for 3 seconds, an automatic land command is triggered.
- Emergency stop sends multiple zero velocity commands.

## 🧩 Notes

- `tello_odom_node.py` publishes odometry based purely on velocity integration; it will drift over time.
- Install required Python packages (e.g., `pynput`) if not already available.
