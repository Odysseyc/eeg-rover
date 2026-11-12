"""MuJoCo Ackermann car simulator exposed to ROS 2.

Subscribes:  /cmd_vel   (geometry_msgs/Twist)   from keyboard_teleop or teleop_twist_joy
Publishes:   /odom, TF odom -> base_link        ground-truth pose and velocity
             /scan, /imu, /camera/*, /joint_states   (see sensors.py)
             /wheel_speed_targets               PID setpoints, for tuning in rqt_plot

PID gains can be changed while running, e.g.
  ros2 param set /mujoco_car_sim pid.kp 0.3
"""
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from mujoco_car_sim.obstacles import ObstacleConfig, build_model
from mujoco_car_sim.sensors import SensorSuite
from mujoco_car_sim.vehicle import WHEELS, AckermannKinematics, Geometry, WheelPID

FRAME_DT = 1.0 / 60.0  # viewer refresh / loop period (s)


class MujocoCarSim(Node):
    def __init__(self):
        super().__init__('mujoco_car_sim')
        dyn = ParameterDescriptor(dynamic_typing=True)  # accept 0 or 0.0 alike

        default_model = os.path.join(
            get_package_share_directory('mujoco_car_sim'), 'models', 'ackermann_car.xml')
        params = {
            'model_path': default_model,
            'body_name': 'chassis',
            'cmd_timeout': 0.3,          # s without /cmd_vel before stopping (0 = never)
            'odom_rate': 50.0,
            'vehicle.max_speed': 3.0,    # m/s
            'vehicle.max_steer': 0.45,   # rad
            'pid.kp': 0.15,              # N·m per (rad/s) of wheel-speed error
            'pid.ki': 2.0,
            'pid.kd': 0.0,
            'pid.max_torque': 0.6,       # N·m per wheel
            'sensors.camera.enabled': True,
            'sensors.camera.width': 320,
            'sensors.camera.height': 240,
            'sensors.camera.rate': 15.0,
            'sensors.lidar.enabled': True,
            'sensors.lidar.beams': 360,
            'sensors.lidar.rate': 10.0,
            'sensors.lidar.range_max': 10.0,
            'sensors.lidar.noise_std': 0.01,
            'sensors.imu.rate': 100.0,   # 0 disables
            'sensors.joint_states.rate': 50.0,
        }
        for name, value in params.items():
            self.declare_parameter(name, value, dyn)
        obstacle_defaults = ObstacleConfig()
        for field, value in vars(obstacle_defaults).items():
            self.declare_parameter(f'obstacles.{field}', value, dyn)
        p = lambda n: self.get_parameter(n).value

        # ---- Build the world: car model + procedural obstacles ----
        cfg = ObstacleConfig(**{f: type(v)(p(f'obstacles.{f}'))
                                for f, v in vars(obstacle_defaults).items()})
        self.model, seed, n = build_model(p('model_path'), cfg, p('body_name'))
        self.data = mujoco.MjData(self.model)
        self.get_logger().info(
            f'Placed {n}/{cfg.count} obstacles (seed {seed}; set seed:={seed} to reproduce).')

        # ---- Look up joints/actuators once by name ----
        m = self.model
        self.body_id = m.body(p('body_name')).id
        self.wheel_joints = [m.joint(f'wheel_{w}') for w in WHEELS]
        self.steer_joints = [m.joint('steer_fl'), m.joint('steer_fr')]
        self.wheel_dofs = np.array([j.dofadr[0] for j in self.wheel_joints])
        self.drive_acts = np.array([m.actuator(f'drive_{w}').id for w in WHEELS])
        self.steer_acts = np.array([m.actuator('steer_fl').id, m.actuator('steer_fr').id])

        # ---- Vehicle geometry is measured from the model, so XML edits stay in sync ----
        rear_l, rear_r = m.body('wheel_rl').pos, m.body('wheel_rr').pos
        geom = Geometry(
            wheelbase=float(m.body('knuckle_fl').pos[0] - rear_l[0]),
            track=float(rear_l[1] - rear_r[1]),
            wheel_radius=float(m.geom_size[m.body('wheel_rl').geomadr[0]][0]),
            max_steer=float(p('vehicle.max_steer')),
            max_speed=float(p('vehicle.max_speed')),
        )
        self.kinematics = AckermannKinematics(geom)
        self.pid = WheelPID(p('pid.kp'), p('pid.ki'), p('pid.kd'), p('pid.max_torque'))
        self.wheel_targets = np.zeros(4)
        self.add_on_set_parameters_callback(self.on_params)
        self.get_logger().info(
            f'Car: wheelbase {geom.wheelbase:.3f} m, track {geom.track:.3f} m, '
            f'wheel radius {geom.wheel_radius:.3f} m.')

        # ---- ROS interfaces ----
        self.cmd = Twist()
        self.last_cmd_time = time.monotonic()
        self.cmd_timeout = p('cmd_timeout')
        self.odom_period = 1.0 / p('odom_rate')
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.target_pub = self.create_publisher(JointState, 'wheel_speed_targets', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sensors = SensorSuite(self, m, self.data, self.body_id, self.wheel_joints,
                                   self.steer_joints, lambda: self.pid.last_torque)

    # ---------- callbacks ----------
    def on_cmd_vel(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_time = time.monotonic()

    def on_params(self, params):
        """Live PID tuning via `ros2 param set`."""
        for prm in params:
            if prm.name in ('pid.kp', 'pid.ki', 'pid.kd', 'pid.max_torque'):
                setattr(self.pid, prm.name.split('.')[1], float(prm.value))
                self.get_logger().info(f'{prm.name} = {prm.value}')
        return SetParametersResult(successful=True)

    # ---------- control ----------
    def update_targets(self):
        """Once per frame: turn the latest Twist into steering angles and wheel-speed targets."""
        stale = self.cmd_timeout > 0 and time.monotonic() - self.last_cmd_time > self.cmd_timeout
        v = 0.0 if stale else self.cmd.linear.x
        w = 0.0 if stale else self.cmd.angular.z
        steer_l, steer_r, self.wheel_targets = self.kinematics(v, w)
        self.data.ctrl[self.steer_acts] = (steer_l, steer_r)  # position servos track these

    def physics_step(self):
        """Every physics step: run the wheel PID, then advance MuJoCo by one timestep."""
        measured = self.data.qvel[self.wheel_dofs]
        self.data.ctrl[self.drive_acts] = self.pid(self.wheel_targets, measured, self.model.opt.timestep)
        mujoco.mj_step(self.model, self.data)

    # ---------- odometry ----------
    def publish_odom(self):
        stamp = self.get_clock().now().to_msg()
        pos = self.data.xpos[self.body_id]
        w, x, y, z = self.data.xquat[self.body_id]  # MuJoCo order is (w, x, y, z)

        vel = np.zeros(6)  # [angular(3), linear(3)] in world frame
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.body_id, vel, 0)
        R = self.data.xmat[self.body_id].reshape(3, 3)
        ang_b, lin_b = R.T @ vel[:3], R.T @ vel[3:]  # rotate into the car's frame

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'odom'
        odom.child_frame_id = 'base_link'
        pp = odom.pose.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, pos)
        pp.orientation.w, pp.orientation.x, pp.orientation.y, pp.orientation.z = map(float, (w, x, y, z))
        tw = odom.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = map(float, lin_b)
        tw.angular.x, tw.angular.y, tw.angular.z = map(float, ang_b)
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, pos)
        tf.transform.rotation = pp.orientation
        self.tf_broadcaster.sendTransform(tf)

        targets = JointState()
        targets.header.stamp = stamp
        targets.name = [j.name for j in self.wheel_joints]
        targets.velocity = [float(t) for t in self.wheel_targets]
        self.target_pub.publish(targets)

    # ---------- main loop ----------
    def run(self):
        steps_per_frame = max(1, round(FRAME_DT / self.model.opt.timestep))
        last_odom = 0.0
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
            viewer.cam.trackbodyid = self.body_id
            viewer.cam.distance, viewer.cam.elevation = 2.0, -25.0
            while viewer.is_running() and rclpy.ok():
                frame_start = time.monotonic()
                rclpy.spin_once(self, timeout_sec=0.0)  # handle /cmd_vel and param changes

                self.update_targets()
                for _ in range(steps_per_frame):
                    self.physics_step()
                viewer.sync()

                if frame_start - last_odom >= self.odom_period:
                    self.publish_odom()
                    last_odom = frame_start
                self.sensors.update()

                remaining = FRAME_DT - (time.monotonic() - frame_start)
                if remaining > 0:
                    time.sleep(remaining)


def main():
    rclpy.init()
    node = MujocoCarSim()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.sensors.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
