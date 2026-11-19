# EEG Rover

Driving a mobile robot with a non-invasive brain-computer interface. A motor-imagery EEG classifier
(built by teammates) produces coarse, noisy intent (left / right / forward / backward). This project turns
that intent into safe, useful motion, with a **shared-autonomy** layer in between: the user supplies intent,
the robot supplies safety and detail. Target applications are assistive mobility, e.g. brain-controlled wheelchairs.

## Repo layout

| Path | Runs on | What |
|---|---|---|
| `rover/` | Jetson (ROS 1 Noetic) | `eeg_rover_base` package: driver, watchdog, calibration (`config/rover.yaml`), browser camera stream |
| `laptop/ros2_ws/src/mujoco_car_sim` | Laptop (ROS 2 Humble, WSL) | MuJoCo sim with sensors and obstacles, keyboard teleop, `rover_bridge` |
| `eeg/` | Laptop (Python venv) | Motor-imagery classifiers on BCI Competition IV 2a; replay banks for the sim |
| `tools/xbox_teleop.py` | Windows | Xbox controller driving, no ROS needed |
| `firmware/feather` | Steering board | Servo firmware (reference copy) |
| `docs/` | | Hardware, calibration, and the proposed EEG interface |

## Quick start

**Rover** (over SSH; rover on a box for first runs):
```bash
roslaunch eeg_rover_base rover.launch
```
**Xbox controller** (Windows PowerShell):
```powershell
py tools\xbox_teleop.py --host <jetson-ip>
```
**Keyboard + sim preview** (Ubuntu / WSL):
```bash
ros2 launch mujoco_car_sim rover.launch.py rover_host:=<jetson-ip>
```
**Simulation only:**
```bash
ros2 launch mujoco_car_sim sim.launch.py rviz:=true
```
One-time setup for each machine is in `docs/hardware.md`.

## Credits

The rover hardware and its original drivers come from Yunxin Fan's Diffusion-SAFE experiments
([labicon/HRMotion-RestrictedView](https://github.com/labicon/HRMotion-RestrictedView)). `rover_base.py` is a
rewrite of her `ros2feather.py` and `ros2vesc.py` (same serial protocols and speed conversion), and
`firmware/feather/code.py` is her steering firmware. Her repo has no license, so keep this repo private
unless she agrees otherwise.
