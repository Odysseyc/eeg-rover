#!/usr/bin/env python3
"""Onboard driver for the rover (ROS 1 Noetic, runs on the Jetson).

One node that owns both serial devices:
  Adafruit Feather  -> steering servo (text line with the servo angle in degrees, 30..140, 85 = straight)
  VESC              -> drive motor (RPM control through pyvesc)

Input:   /cmd_vel   geometry_msgs/Twist   linear.x = speed (m/s, + forward)
                                          angular.z = yaw rate (rad/s, + = turn left)
Output:  /rover/battery (%), /rover/voltage (V), /rover/speed (measured m/s), /rover/steer_angle (rad)

Fixes compared with the original ros2feather.py / ros2vesc.py:
  * Reverse works: forward and reverse use the same speed scaling and the same low-speed handling.
    (The original capped throttle at low RPM in a way reverse could never get past.)
  * Left means left: steering direction is a single calibrated parameter (steer_sign).
  * Safety: if /cmd_vel stops for `cmd_timeout` seconds, the rover centers its wheels and stops.
  * Smooth: speed changes are rate-limited (accel / decel limits).

The math lives in plain classes (no ROS imports) so it can be tested off the robot.
"""
import math
import time


class SteeringMap:
    """Turn a desired yaw rate into a front-wheel angle (bicycle model) and a servo angle."""

    def __init__(self, wheelbase=0.33, max_steer=0.45, servo_center=85.0, servo_min=30.0,
                 servo_max=140.0, servo_deg_per_rad=122.0, steer_sign=-1.0):
        self.L, self.max_steer = wheelbase, max_steer
        self.center, self.smin, self.smax = servo_center, servo_min, servo_max
        self.gain, self.sign = servo_deg_per_rad, steer_sign

    def wheel_angle(self, v, w):
        if abs(v) < 0.02:
            # Can't turn in place: point the wheels the way the driver asks, ready to move.
            delta = math.copysign(self.max_steer, w) if abs(w) > 1e-3 else 0.0
        else:
            # Same formula forward and backward, so a given steering input means the same wheel angle.
            delta = math.atan(self.L * w / v)
        return max(-self.max_steer, min(self.max_steer, delta))

    def servo_angle(self, delta):
        """delta > 0 = wheels turned left. On this rover a LOWER servo angle turns left, hence sign -1."""
        a = self.center + self.sign * self.gain * delta
        return max(self.smin, min(self.smax, a))


class SpeedControl:
    """Rate-limited speed command -> VESC electrical RPM, identical for forward and reverse."""

    def __init__(self, erpm_per_mps=2850.6, max_speed=1.5, accel=1.5, decel=3.0,
                 min_erpm=600.0, deadband=0.05):
        # erpm_per_mps comes from ros2vesc.py's own conversion: speed = erpm / 60 / 10 * 0.067 * pi
        self.k, self.vmax = erpm_per_mps, max_speed
        self.accel, self.decel = accel, decel
        self.min_erpm, self.deadband = min_erpm, deadband
        self.v = 0.0  # current (ramped) speed command, m/s

    def step(self, target, dt):
        target = max(-self.vmax, min(self.vmax, target))
        if abs(target) < self.deadband:
            target = 0.0
        # Speeding up (same direction, larger magnitude) uses accel; slowing down or reversing uses decel.
        speeding_up = abs(target) > abs(self.v) and (self.v == 0 or math.copysign(1, target) == math.copysign(1, self.v))
        limit = (self.accel if speeding_up else self.decel) * dt
        self.v += max(-limit, min(limit, target - self.v))
        return self.erpm()

    def erpm(self):
        """None means 'let the motor coast' (no command needed); otherwise an RPM setpoint."""
        if abs(self.v) < self.deadband:
            return None
        e = self.v * self.k
        if abs(e) < self.min_erpm:  # the VESC can't hold very low RPM; give it the smallest it can do
            e = math.copysign(self.min_erpm, e)
        return int(e)


def lipo_percent(voltage, cells=3):
    """Rough state of charge from resting voltage (3.3 V/cell = 0 %, 4.2 V/cell = 100 %)."""
    per_cell = voltage / cells
    return max(0.0, min(100.0, (per_cell - 3.3) / (4.2 - 3.3) * 100.0))


def main():
    import rospy
    import serial
    from geometry_msgs.msg import Twist
    from pyvesc import VESC
    from std_msgs.msg import Float32

    rospy.init_node('rover_base')
    P = lambda name, default: rospy.get_param('~' + name, default)
    steering = SteeringMap(P('wheelbase', 0.33), P('max_steer', 0.45), P('servo_center', 85.0),
                           P('servo_min', 30.0), P('servo_max', 140.0),
                           P('servo_deg_per_rad', 122.0), P('steer_sign', -1.0))
    speed = SpeedControl(P('erpm_per_mps', 2850.6), P('max_speed', 1.5), P('accel', 1.5),
                         P('decel', 3.0), P('min_erpm', 600.0))
    timeout, rate_hz = P('cmd_timeout', 0.5), P('rate', 50.0)

    feather = serial.Serial(P('steer_port', '/dev/serial/by-id/usb-Adafruit_Feather_M4_CAN_A88A3C905339473237202020FF113638-if00'),
                            115200, timeout=0.01)
    vesc = VESC(P('vesc_port', '/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00'))
    time.sleep(2.0)  # the Feather resets when the port opens
    rospy.loginfo('rover_base: serial ports open')

    cmd = {'v': 0.0, 'w': 0.0, 't': 0.0}

    def on_cmd(msg):
        cmd['v'], cmd['w'], cmd['t'] = msg.linear.x, msg.angular.z, time.monotonic()

    rospy.Subscriber('cmd_vel', Twist, on_cmd, queue_size=1)
    pubs = {n: rospy.Publisher('rover/' + n, Float32, queue_size=1)
            for n in ('battery', 'voltage', 'speed', 'steer_angle')}

    rate = rospy.Rate(rate_hz)
    dt, stale_logged, volts = 1.0 / rate_hz, True, []
    try:
        while not rospy.is_shutdown():
            stale = time.monotonic() - cmd['t'] > timeout
            if stale != stale_logged:
                if stale:
                    rospy.logwarn('No /cmd_vel for %.1f s: stopping', timeout)
                else:
                    rospy.loginfo('Receiving /cmd_vel')
                stale_logged = stale
            v, w = (0.0, 0.0) if stale else (cmd['v'], cmd['w'])

            delta = steering.wheel_angle(v, w)
            feather.write(f'{int(round(steering.servo_angle(delta)))}\n'.encode())
            feather.reset_input_buffer()

            erpm = speed.step(v, dt)
            if erpm is None:
                vesc.set_current(0.0)
            else:
                vesc.set_rpm(erpm)

            m = vesc.get_measurements()
            if m is not None:
                volts = (volts + [m.v_in])[-200:]
                vin = sum(volts) / len(volts)
                pubs['voltage'].publish(vin)
                pubs['battery'].publish(lipo_percent(vin))
                pubs['speed'].publish(m.rpm / speed.k)
            pubs['steer_angle'].publish(delta)
            rate.sleep()
    finally:
        try:
            vesc.set_current(0.0)
            feather.write(f'{int(steering.center)}\n'.encode())
        finally:
            feather.close()


if __name__ == '__main__':
    main()
