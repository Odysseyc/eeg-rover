"""Forward /cmd_vel from the laptop (ROS 2) to the real rover (ROS 1 on the Jetson).

  laptop:  keyboard / EEG / shared autonomy -> /cmd_vel -> rover_bridge ==websocket==> Jetson rosbridge
  Jetson:  /cmd_vel -> rover_base (steering, motor, watchdog)

The rover does all unit conversion and safety itself, so this node only relays commands
(plus a speed cap) and brings the rover's telemetry back as /rover/battery and /rover/speed.
The laptop only makes an outgoing connection, so WSL's default networking works.
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

try:
    import roslibpy
except ImportError:
    roslibpy = None


class RoverBridge(Node):
    def __init__(self):
        super().__init__('rover_bridge')
        for k, v in {'rover_host': '192.168.0.149', 'rover_port': 9090, 'rate': 20.0,
                     'cmd_timeout': 0.3, 'max_speed': 1.0}.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in
                  ('rover_host', 'rover_port', 'rate', 'cmd_timeout', 'max_speed')}
        self.cmd, self.last_cmd, self.connected = Twist(), 0.0, False
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.telemetry = {n: self.create_publisher(Float32, f'rover/{n}', 10) for n in ('battery', 'speed')}
        self.ros = self.cmd_topic = None
        self.connect()
        self.create_timer(1.0 / self.p['rate'], self.tick)

    def connect(self):
        if roslibpy is None:
            self.get_logger().error('roslibpy is not installed: run  pip3 install --user roslibpy')
            return
        host, port = self.p['rover_host'], int(self.p['rover_port'])
        self.ros = roslibpy.Ros(host=host, port=port)
        self.cmd_topic = roslibpy.Topic(self.ros, '/cmd_vel', 'geometry_msgs/Twist')
        for name in ('battery', 'speed'):
            t = roslibpy.Topic(self.ros, f'/rover/{name}', 'std_msgs/Float32', throttle_rate=500)
            t.subscribe(lambda m, n=name: self.telemetry[n].publish(Float32(data=float(m['data']))))
        self.get_logger().info(f'Connecting to rover at ws://{host}:{port} ...')
        try:
            self.ros.run(timeout=5)
        except Exception:
            self.get_logger().warn('Rover not reachable yet; will keep retrying. '
                                   'Is `roslaunch eeg_rover_base rover.launch` running on the Jetson?')

    def on_cmd(self, msg):
        self.cmd, self.last_cmd = msg, time.monotonic()

    @staticmethod
    def twist_dict(v, w):
        return {'linear': {'x': float(v), 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': float(w)}}

    def tick(self):
        if self.ros is None:
            return
        if self.ros.is_connected != self.connected:
            self.connected = self.ros.is_connected
            if self.connected:
                self.get_logger().info('Connected to rover.')
            else:
                self.get_logger().warn('Lost connection to rover (it will stop itself).')
        if not self.connected:
            return
        fresh = time.monotonic() - self.last_cmd < self.p['cmd_timeout']
        cap = self.p['max_speed']
        v = max(-cap, min(cap, self.cmd.linear.x)) if fresh else 0.0
        w = self.cmd.angular.z if fresh else 0.0
        self.cmd_topic.publish(roslibpy.Message(self.twist_dict(v, w)))

    def stop(self):
        if self.ros is not None and self.ros.is_connected:
            for _ in range(3):
                self.cmd_topic.publish(roslibpy.Message(self.twist_dict(0.0, 0.0)))
                time.sleep(0.05)
            self.ros.terminate()


def main():
    rclpy.init()
    node = RoverBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
