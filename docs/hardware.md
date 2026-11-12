# Hardware and setup

## One-time setup

### Jetson (over SSH)
```bash
cd ~ && git clone https://github.com/<you>/eeg-rover.git          # or: gh repo clone eeg-rover
mkdir -p ~/eeg_ws/src && ln -s ~/eeg-rover/rover ~/eeg_ws/src/eeg_rover_base
cd ~/eeg_ws && catkin_make
echo 'source ~/eeg_ws/devel/setup.bash' >> ~/.bashrc && source ~/.bashrc
```
Only the `rover/` folder is linked into the catkin workspace, so the ROS 2 code is never built here.
Needs `rosbridge_server`, `pyserial` and `pyvesc` (already installed on this rover for Yunxin's setup).

### Laptop (Ubuntu / WSL)
```bash
pip3 install --user roslibpy pynput
cd ~/eeg-rover/laptop/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

### Windows (Xbox controller)
```powershell
py -m pip install pygame roslibpy
```

## Every session
1. Charge check: `/rover/battery` or the Xbox script's display. Recharge below ~10 %.
2. Rover on a box for the first test of the day.
3. Jetson: `roslaunch eeg_rover_base rover.launch`
4. Laptop or Windows: start the Xbox script or `rover.launch.py`.
5. **Watchdog test:** spin the wheels, then close the laptop/Windows program. Wheels must stop within ~0.5 s.

Don't run Yunxin's `ros2feather.py` / `ros2vesc.py` at the same time as `rover.launch`: they use the same serial ports.

## Power
- 3S LiPo (11.1 V nominal, 12.6 V full, 9.9 V empty), XT90 plug, through an XT90-male to XT60-female adapter.
- The rover's XT60 input is a splitter: it powers both the Jetson and the motor controller.
- Charging: SkyRC B6 V2 on its 12 V supply, **LiPo BALANCE** mode, 3S, 2–4 A, main and balance plugs both connected.
  Never charge unattended. Stop using a swollen pack. Store at about 3.8 V/cell (11.4 V).

| Pack voltage | Per cell | Meaning |
|---|---|---|
| 12.6 V | 4.20 V | Full |
| ~11.4–11.6 V | ~3.8–3.85 V | Storage (~50–60 %) |
| 9.9 V | 3.3 V | Stop and recharge |

## Connecting
- SSH over Wi-Fi: `ssh racecar@<jetson-ip>`; over USB-C from Windows PowerShell: `ssh racecar@192.168.55.1`.
- The laptop only connects *out* to the rover (port 9090), so WSL's default networking works.

## Useful checks on the Jetson
```bash
ls /dev/serial/by-id/          # the Feather and the VESC should both appear
rostopic echo /rover/battery   # battery %
rostopic echo /rover/speed     # measured speed, m/s
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}}"   # drive slowly; Ctrl+C and it stops after 0.5 s
```
