"""Drive the rover with an Xbox controller, from Windows (no ROS needed on this computer).

Setup (Windows PowerShell, once):   py -m pip install pygame roslibpy
Run:                                py xbox_teleop.py --host 192.168.0.149
Check the controller's axes:        py xbox_teleop.py --show

Controls (left stick only):
  Push up / down     forward / reverse; further = faster
  Push left / right  steer (left = wheels left, also in reverse)
  Any direction      both at once: up-left = forward while turning left, down-right = reverse with wheels right
  B                  emergency stop: no driving and no steering, wheels straight
  A                  resume (takes effect once the stick is back in the center)
  Start/Menu         quit
A car can't move sideways, so pushing straight left or right only turns the wheels; push diagonally to turn while moving.

Sends geometry_msgs/Twist on /cmd_vel, the same command the simulator and the EEG pipeline use.
The rover (`roslaunch eeg_rover_base rover.launch`) converts it to steering and motor commands and
stops by itself within ~0.5 s if this script stops sending.
"""
import argparse
import math
import sys
import time

# Xbox controller layout under pygame 2 on Windows (SDL2 / XInput)
AXIS_X, AXIS_Y = 0, 1   # left stick; pygame reports up as -1
BTN_A, BTN_B, BTN_START = 0, 1, 7


def radial_deadzone(x, y, dz):
    """Ignore small stick wobble in any direction, then rescale so the edge is still 1."""
    r = math.hypot(x, y)
    if r < dz:
        return 0.0, 0.0
    scale = min(1.0, (r - dz) / (1.0 - dz)) / r
    return x * scale, y * scale


def to_twist(x, y, max_speed, max_steer, wheelbase):
    """Left stick (x right +, y down +) -> (speed m/s, yaw rate rad/s).

    Up/down sets speed, left/right sets the front-wheel angle (stick left = wheels left).
    The yaw rate is computed so the rover's bicycle model recovers exactly that wheel angle,
    forward or in reverse.
    """
    v = -y * max_speed
    delta = -x * max_steer                # stick x is + to the right; + wheel angle means left
    w = v * math.tan(delta) / wheelbase
    if abs(v) < 0.02 and abs(delta) > 0:
        w = math.copysign(1e-2, delta)    # stopped: tiny yaw request so the wheels still point the right way
    return v, w


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='192.168.0.149', help="Jetson's IP address")
    ap.add_argument('--port', type=int, default=9090)
    ap.add_argument('--max-speed', type=float, default=0.6, help='m/s with the stick fully up (start low)')
    ap.add_argument('--max-steer', type=float, default=0.45, help='rad; match max_steer in rover.yaml')
    ap.add_argument('--wheelbase', type=float, default=0.33, help='m; match wheelbase in rover.yaml')
    ap.add_argument('--deadzone', type=float, default=0.12)
    ap.add_argument('--rate', type=float, default=20.0)
    ap.add_argument('--show', action='store_true', help='print raw axes/buttons only; send nothing')
    args = ap.parse_args()

    import pygame
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        sys.exit('No controller found. Plug it in (or pair it) and make sure Windows sees it.')
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f'Controller: {js.get_name()} ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)')

    if args.show:
        print('Move sticks/triggers and press buttons; Ctrl+C to quit.')
        while True:
            pygame.event.pump()
            axes = ' '.join(f'{i}:{js.get_axis(i):+.2f}' for i in range(js.get_numaxes()))
            btns = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
            print(f'\raxes {axes}  buttons {btns}      ', end='', flush=True)
            time.sleep(0.05)

    import roslibpy
    ros = roslibpy.Ros(host=args.host, port=args.port)
    print(f'Connecting to ws://{args.host}:{args.port} ...')
    try:
        ros.run(timeout=5)
    except Exception:
        sys.exit('Could not reach the rover. Is `roslaunch eeg_rover_base rover.launch` running, and is --host right?')
    cmd = roslibpy.Topic(ros, '/cmd_vel', 'geometry_msgs/Twist')
    tele = {'battery': None, 'speed': None}
    for name in tele:
        roslibpy.Topic(ros, f'/rover/{name}', 'std_msgs/Float32', throttle_rate=500).subscribe(
            lambda m, n=name: tele.__setitem__(n, m['data']))

    def send(v, w):
        cmd.publish(roslibpy.Message({'linear': {'x': v, 'y': 0.0, 'z': 0.0},
                                      'angular': {'x': 0.0, 'y': 0.0, 'z': w}}))

    print('Connected. Left stick drives (up/down = speed, left/right = steer). '
          'B = emergency stop, A = resume, Start = quit.')
    # Start in the stopped state until the stick has been centered once, so the rover never jumps on startup.
    stopped, armed, period = True, True, 1.0 / args.rate
    try:
        while True:
            t0 = time.monotonic()
            pygame.event.pump()
            if pygame.joystick.get_count() == 0:
                print('\nController disconnected: stopping.')
                break
            if js.get_button(BTN_START):
                break
            x, y = radial_deadzone(js.get_axis(AXIS_X), js.get_axis(AXIS_Y), args.deadzone)
            centered = x == 0.0 and y == 0.0
            if js.get_button(BTN_B):
                stopped, armed = True, False
            elif js.get_button(BTN_A):
                armed = True                      # resume requested...
            if stopped and armed and centered:
                stopped = False                   # ...granted once the stick is centered

            # E-stop: no speed AND no steering (zero yaw rate makes the rover center its wheels).
            v, w = (0.0, 0.0) if stopped else to_twist(x, y, args.max_speed, args.max_steer, args.wheelbase)
            if ros.is_connected:
                send(v, w)
                b = f"{tele['battery']:.0f}%" if tele['battery'] is not None else '?'
                s = f"{tele['speed']:+.2f}" if tele['speed'] is not None else '?'
                if not stopped:
                    state = 'driving'
                elif armed:
                    state = 'center the stick'
                else:
                    state = 'STOPPED (press A)'
                print(f'\r{state:20s} cmd {v:+.2f} m/s  measured {s} m/s  battery {b}   ', end='', flush=True)
            else:
                print('\rLost connection to rover (it will stop itself)...          ', end='', flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if ros.is_connected:
            for _ in range(3):
                send(0.0, 0.0)
                time.sleep(0.05)
            ros.terminate()
        print('\nStopped.')


if __name__ == '__main__':
    main()
