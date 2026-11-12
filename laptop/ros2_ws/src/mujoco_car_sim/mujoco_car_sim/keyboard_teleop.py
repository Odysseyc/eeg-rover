"""Hold-to-drive keyboard teleop for the MuJoCo car.

Arrow keys drive the car only while they are held down. Hold Shift for full speed.
Publishes geometry_msgs/Twist on /cmd_vel, same as teleop_twist_joy will later.

Unlike teleop_twist_keyboard (which reads the terminal and never sees key releases),
this uses pynput to get real press/release events. By default it only reacts while
the focused window's title contains "MuJoCo", so typing in other windows is ignored.
Arrow keys are used because the MuJoCo viewer binds most letter keys to display toggles.
"""
import threading

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
from rclpy.node import Node

try:
    from Xlib import X, display as xdisplay
except ImportError:  # focus check just gets disabled
    xdisplay = None


class FocusChecker:
    """Returns True if the active X11 window's title contains `needle`."""

    def __init__(self, needle, logger):
        self.needle = needle.lower()
        self.disp = None
        try:
            self.disp = xdisplay.Display()
            self.root = self.disp.screen().root
            self.NET_ACTIVE = self.disp.intern_atom('_NET_ACTIVE_WINDOW')
            self.NET_NAME = self.disp.intern_atom('_NET_WM_NAME')
        except Exception as e:  # no X server (e.g. pure Wayland)
            logger.warn(f'Window focus check unavailable ({e}); keys work in any window.')
            self.disp = None

    def focused(self):
        if self.disp is None:
            return True
        try:
            prop = self.root.get_full_property(self.NET_ACTIVE, X.AnyPropertyType)
            if not prop or not prop.value[0]:
                return False
            win = self.disp.create_resource_object('window', prop.value[0])
            name = win.get_full_property(self.NET_NAME, 0)
            title = name.value if name else win.get_wm_name()
            if isinstance(title, bytes):
                title = title.decode(errors='ignore')
            return self.needle in (title or '').lower()
        except Exception:
            return True  # don't lock the user out because of a flaky X query


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('linear_speed', 1.0)      # m/s with arrow key
        self.declare_parameter('angular_speed', 1.5)     # rad/s (Ackermann: only turns while moving)
        self.declare_parameter('boost', 2.0)             # multiplier while Shift is held
        self.declare_parameter('rate', 30.0)             # Hz while keys are held
        self.declare_parameter('require_viewer_focus', True)
        self.declare_parameter('window_title', 'MuJoCo')

        p = lambda n: self.get_parameter(n).value
        self.lin, self.ang, self.boost = p('linear_speed'), p('angular_speed'), p('boost')
        self.focus = (FocusChecker(p('window_title'), self.get_logger())
                      if p('require_viewer_focus') and xdisplay else None)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.held = set()
        self.lock = threading.Lock()
        self.was_moving = False

        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.create_timer(1.0 / p('rate'), self.tick)
        self.get_logger().info(
            'Click the MuJoCo window, then hold arrow keys to drive (Shift = faster).')

    @staticmethod
    def _name(key):
        if key in (keyboard.Key.up, keyboard.Key.down, keyboard.Key.left, keyboard.Key.right):
            return key.name
        if key in (keyboard.Key.shift, keyboard.Key.shift_l, keyboard.Key.shift_r):
            return 'shift'
        return None

    def on_press(self, key):
        name = self._name(key)
        if name:
            with self.lock:
                self.held.add(name)

    def on_release(self, key):
        name = self._name(key)
        if name:
            with self.lock:
                self.held.discard(name)

    def tick(self):
        with self.lock:
            held = set(self.held)
        if self.focus is not None and not self.focus.focused():
            held = set()

        fwd = ('up' in held) - ('down' in held)
        turn = ('left' in held) - ('right' in held)   # left = +yaw (REP 103)
        scale = self.boost if 'shift' in held else 1.0

        moving = bool(fwd or turn)
        if moving or self.was_moving:  # publish while held, plus one zero on release
            msg = Twist()
            msg.linear.x = fwd * self.lin * scale
            msg.angular.z = turn * self.ang * scale
            self.pub.publish(msg)
        # Going quiet when idle means this node won't fight a joystick teleop later.
        self.was_moving = moving

    def destroy_node(self):
        self.listener.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
