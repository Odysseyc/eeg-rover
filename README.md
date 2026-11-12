# EEG Rover

Driving a mobile robot with a non-invasive brain-computer interface. A motor-imagery EEG classifier produces coarse, 
noisy intent (left / right / forward / backward). This project turns that intent into safe, useful motion, with a 
**shared-autonomy** layer in between: the user supplies intent, the robot supplies safety and detail. Target 
applications are assistive mobility, e.g. brain-controlled wheelchairs.

Everything that decides *what* to do runs on the laptop. The rover only executes `/cmd_vel`
(speed in m/s, yaw rate in rad/s) and stops itself if commands stop for 0.5 s.
The simulator accepts the same `/cmd_vel`, so anything developed in sim drives the real rover unchanged.

## Repo layout

| Path | Runs on | What |
|---|---|---|
| `rover/` | Jetson (ROS 1 Noetic) | `eeg_rover_base` package: driver, watchdog, calibration (`config/rover.yaml`) |
| `laptop/ros2_ws/src/mujoco_car_sim` | Laptop (ROS 2 Humble, WSL) | MuJoCo sim with sensors and obstacles, keyboard teleop, `rover_bridge` |
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

## Roadmap
- [x] MuJoCo simulation (Ackermann car, PID wheel control, camera / lidar / IMU, procedural obstacles)
- [x] Rover driver with working reverse, correct steering direction, ramping and watchdog
- [x] Remote driving from laptop and Xbox controller
- [ ] Calibrate the rover (`docs/calibration.md`)
- [ ] Simulated noisy-EEG experiment: corrupt commands at a set accuracy, add shared autonomy, measure success / collisions / time
- [ ] Agree the EEG interface with the EEG team (`docs/eeg_interface.md`)
- [ ] Shared-autonomy layer (intent inference, lidar-based safety filter, smooth handover)
- [ ] Closed-loop tests with the real EEG classifier

## Credits
The rover hardware and its original drivers come from Yunxin Fan's Diffusion-SAFE experiments
([labicon/HRMotion-RestrictedView](https://github.com/labicon/HRMotion-RestrictedView)). `rover_base.py` is a
rewrite of her `ros2feather.py` and `ros2vesc.py` (same serial protocols and speed conversion), and
`firmware/feather/code.py` is her steering firmware. Her repo has no license, so keep this repo private
unless she agrees otherwise.
