import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Obstacle knobs you can override from the command line, e.g.
#   ros2 launch mujoco_car_sim sim.launch.py count:=80 cluster_count:=2 seed:=42
OBSTACLE_ARGS = {
    'count': '60',            # how many obstacles (0 = none)
    'cluster_count': '4',     # 0 = uniform scatter
    'cluster_spread': '0.6',  # metres; smaller = tighter clusters
    'seed': '-1',             # -1 = new layout each launch
}


def generate_launch_description():
    share = get_package_share_directory('mujoco_car_sim')
    keyboard = LaunchConfiguration('keyboard')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        # keyboard:=false when driving with teleop_twist_joy; rviz:=true to see the sensors.
        DeclareLaunchArgument('keyboard', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        *[DeclareLaunchArgument(n, default_value=v) for n, v in OBSTACLE_ARGS.items()],

        Node(
            package='mujoco_car_sim',
            executable='sim_node',
            output='screen',
            additional_env={'MUJOCO_GL': 'egl'},  # camera renders via EGL, separate from the viewer
            parameters=[{
                'cmd_timeout': 0.3,
                'vehicle.max_speed': 3.0,
                'vehicle.max_steer': 0.45,
                # Wheel-speed PID (tune live: ros2 param set /mujoco_car_sim pid.kp 0.3)
                'pid.kp': 0.15,
                'pid.ki': 2.0,
                'pid.kd': 0.0,
                'pid.max_torque': 0.6,
                # Sensors
                'sensors.camera.enabled': True,
                'sensors.camera.width': 320,
                'sensors.camera.height': 240,
                'sensors.camera.rate': 15.0,
                'sensors.lidar.enabled': True,
                'sensors.lidar.beams': 360,
                'sensors.lidar.rate': 10.0,
                'sensors.lidar.range_max': 10.0,
                'sensors.lidar.noise_std': 0.01,
                'sensors.imu.rate': 100.0,
                'sensors.joint_states.rate': 50.0,
                # Obstacles
                **{f'obstacles.{n}': LaunchConfiguration(n) for n in OBSTACLE_ARGS},
                'obstacles.arena_half_size': 6.0,
                'obstacles.min_size': 0.05,
                'obstacles.max_size': 0.25,
                'obstacles.min_height': 0.05,
                'obstacles.max_height': 0.30,
                'obstacles.spawn_clearance': 1.0,
                'obstacles.min_gap': 0.0,
            }],
        ),
        Node(
            package='mujoco_car_sim',
            executable='keyboard_teleop',
            output='screen',
            condition=IfCondition(keyboard),
            parameters=[{'linear_speed': 1.0, 'angular_speed': 1.5, 'boost': 2.0,
                         'require_viewer_focus': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'car.rviz')],
            condition=IfCondition(rviz),
        ),
    ])
