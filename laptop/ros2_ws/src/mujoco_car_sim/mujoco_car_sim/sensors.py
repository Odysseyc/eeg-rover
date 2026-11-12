"""Simulated sensors, published as standard ROS 2 messages.

  /camera/image_raw, /camera/camera_info  front RGB camera (rendered offscreen by MuJoCo)
  /scan                                   2D 360-degree lidar (MuJoCo raycasting)
  /imu                                    gyro + accelerometer + orientation (MuJoCo sensors)
  /joint_states                           steering angles, wheel angles/speeds/torques

Each sensor's mounting point lives in the model (a <site> or <camera> on the chassis),
so moving a sensor means editing ackermann_car.xml, not this file. Static TF frames
for every sensor are generated from those same model poses.
"""
import math
import time

import mujoco
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_ros import StaticTransformBroadcaster


def _quat_from_mat(mat):
    q = np.zeros(4)
    mujoco.mju_mat2Quat(q, np.asarray(mat, dtype=float).flatten())
    return q  # (w, x, y, z)


class _Rate:
    """Tells you when it's time to publish again (wall-clock based)."""

    def __init__(self, hz):
        self.period = 1.0 / hz if hz > 0 else math.inf
        self.next = 0.0

    def due(self, now):
        if now >= self.next:
            self.next = now + self.period
            return True
        return False


class SensorSuite:
    def __init__(self, node, model, data, body_id, wheel_joints, steer_joints, get_wheel_torques):
        self.node, self.m, self.d = node, model, data
        self.body_id = body_id
        self.wheel_joints, self.steer_joints = wheel_joints, steer_joints
        self.get_wheel_torques = get_wheel_torques
        p = lambda n: node.get_parameter(n).value

        static_tfs = []

        # ---- Lidar: rays cast from the 'lidar' site, in its horizontal plane ----
        self.lidar = None
        if p('sensors.lidar.enabled'):
            n = int(p('sensors.lidar.beams'))
            angles = -math.pi + np.arange(n) * (2 * math.pi / n)
            self.lidar = dict(
                site=model.site('lidar').id, n=n,
                dirs=np.stack([np.cos(angles), np.sin(angles), np.zeros(n)], axis=1),
                range_max=float(p('sensors.lidar.range_max')),
                noise=float(p('sensors.lidar.noise_std')),
                rate=_Rate(p('sensors.lidar.rate')),
                geomid=np.zeros(n, dtype=np.int32), dist=np.zeros(n),
                pub=node.create_publisher(LaserScan, 'scan', 10),
            )
            static_tfs.append(self._site_tf('lidar', 'lidar_link'))
            self.rng = np.random.default_rng()

        # ---- IMU: MuJoCo gyro / accelerometer / framequat sensors on the 'imu' site ----
        self.imu = None
        if p('sensors.imu.rate') > 0:
            adr = lambda name: model.sensor(name).adr[0]
            self.imu = dict(gyro=adr('imu_gyro'), accel=adr('imu_accel'), quat=adr('imu_quat'),
                            rate=_Rate(p('sensors.imu.rate')),
                            pub=node.create_publisher(Imu, 'imu', 10))
            static_tfs.append(self._site_tf('imu', 'imu_link'))

        # ---- Camera: offscreen render from the 'front_camera' camera ----
        self.camera = None
        if p('sensors.camera.enabled'):
            w, h = int(p('sensors.camera.width')), int(p('sensors.camera.height'))
            try:
                renderer = mujoco.Renderer(model, h, w)
            except Exception as e:
                node.get_logger().warn(
                    f'Camera disabled, offscreen rendering failed: {e}. '
                    'Try launching with MUJOCO_GL=egl set.')
            else:
                cam_id = model.camera('front_camera').id
                fy = (h / 2) / math.tan(math.radians(model.cam_fovy[cam_id]) / 2)
                info = CameraInfo(width=w, height=h, distortion_model='plumb_bob',
                                  d=[0.0] * 5,
                                  k=[fy, 0.0, w / 2, 0.0, fy, h / 2, 0.0, 0.0, 1.0],
                                  r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                                  p=[fy, 0.0, w / 2, 0.0, 0.0, fy, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0])
                info.header.frame_id = 'camera_optical_frame'
                self.camera = dict(renderer=renderer, name='front_camera', w=w, h=h, info=info,
                                   rate=_Rate(p('sensors.camera.rate')),
                                   img_pub=node.create_publisher(Image, 'camera/image_raw', 10),
                                   info_pub=node.create_publisher(CameraInfo, 'camera/camera_info', 10))
                static_tfs.append(self._camera_tf(cam_id))

        # ---- Joint states (for RViz robot models and plotting the PID in rqt_plot) ----
        self.joint_rate = _Rate(p('sensors.joint_states.rate'))
        self.joint_pub = node.create_publisher(JointState, 'joint_states', 10)

        if static_tfs:
            self.static_tf = StaticTransformBroadcaster(node)
            self.static_tf.sendTransform(static_tfs)

    # ---------- static transforms from the model's mounting poses ----------
    def _tf(self, child, pos, quat_wxyz):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos)
        r = t.transform.rotation
        r.w, r.x, r.y, r.z = map(float, quat_wxyz)
        return t

    def _site_tf(self, site_name, frame):
        s = self.m.site(site_name)
        return self._tf(frame, s.pos, s.quat)  # site pose relative to the chassis body

    def _camera_tf(self, cam_id):
        # MuJoCo cameras: x right, y up, looking down -z.
        # ROS optical frames: x right, y down, looking down +z. Flip y and z.
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, self.m.cam_quat[cam_id])
        R_opt = R.reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
        return self._tf('camera_optical_frame', self.m.cam_pos[cam_id], _quat_from_mat(R_opt))

    # ---------- publishing ----------
    def update(self):
        now = time.monotonic()
        stamp = None

        def get_stamp():
            nonlocal stamp
            stamp = stamp or self.node.get_clock().now().to_msg()
            return stamp

        if self.lidar and self.lidar['rate'].due(now):
            self._publish_lidar(get_stamp())
        if self.imu and self.imu['rate'].due(now):
            self._publish_imu(get_stamp())
        if self.camera and self.camera['rate'].due(now):
            self._publish_camera(get_stamp())
        if self.joint_rate.due(now):
            self._publish_joints(get_stamp())

    def _publish_lidar(self, stamp):
        L = self.lidar
        R = self.d.site_xmat[L['site']].reshape(3, 3)
        vec = (L['dirs'] @ R.T).flatten()  # beam directions in world frame
        mujoco.mj_multiRay(self.m, self.d, self.d.site_xpos[L['site']].copy(), vec,
                           None, 1, self.body_id,  # include static geoms; ignore our own chassis
                           L['geomid'], L['dist'], None, L['n'], L['range_max'])
        ranges = L['dist'].copy()
        hit = ranges >= 0  # MuJoCo returns -1 for "no hit"
        if L['noise'] > 0:
            ranges[hit] += self.rng.normal(0.0, L['noise'], hit.sum())
        ranges[~hit] = math.inf

        msg = LaserScan()
        msg.header.stamp, msg.header.frame_id = stamp, 'lidar_link'
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / L['n']
        msg.angle_max = msg.angle_min + msg.angle_increment * (L['n'] - 1)
        msg.scan_time = L['rate'].period
        msg.range_min, msg.range_max = 0.05, L['range_max']
        msg.ranges = ranges.astype(np.float32).tolist()
        L['pub'].publish(msg)

    def _publish_imu(self, stamp):
        I, s = self.imu, self.d.sensordata
        msg = Imu()
        msg.header.stamp, msg.header.frame_id = stamp, 'imu_link'
        w, x, y, z = s[I['quat']:I['quat'] + 4]
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = map(float, (w, x, y, z))
        g = s[I['gyro']:I['gyro'] + 3]
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, g)
        a = s[I['accel']:I['accel'] + 3]  # includes gravity (+9.81 z at rest), like a real IMU
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, a)
        I['pub'].publish(msg)

    def _publish_camera(self, stamp):
        C = self.camera
        C['renderer'].update_scene(self.d, camera=C['name'])
        pixels = C['renderer'].render()  # (h, w, 3) uint8, top row first
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, 'camera_optical_frame'
        msg.height, msg.width, msg.encoding = C['h'], C['w'], 'rgb8'
        msg.step = C['w'] * 3
        msg.data.frombytes(pixels.tobytes())
        C['img_pub'].publish(msg)
        C['info'].header.stamp = stamp
        C['info_pub'].publish(C['info'])

    def _publish_joints(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        torques = self.get_wheel_torques()
        for j in self.steer_joints:
            msg.name.append(j.name)
            msg.position.append(float(self.d.qpos[j.qposadr[0]]))
            msg.velocity.append(float(self.d.qvel[j.dofadr[0]]))
            msg.effort.append(0.0)
        for j, tau in zip(self.wheel_joints, torques):
            msg.name.append(j.name)
            msg.position.append(float(self.d.qpos[j.qposadr[0]]))
            msg.velocity.append(float(self.d.qvel[j.dofadr[0]]))
            msg.effort.append(float(tau))
        self.joint_pub.publish(msg)

    def close(self):
        if self.camera:
            self.camera['renderer'].close()
