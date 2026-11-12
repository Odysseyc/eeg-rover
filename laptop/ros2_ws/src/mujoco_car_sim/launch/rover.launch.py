"""Drive the real rover from the laptop.

  ros2 launch mujoco_car_sim rover.launch.py rover_host:=192.168.0.149

Starts rover_bridge (/cmd_vel -> rover) and keyboard teleop, plus the simulator as a live preview
(click its window to give the keyboard focus). For the EEG pipeline later: sim:=false keyboard:=false,
and have the EEG / shared-autonomy node publish /cmd_vel.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim, keyboard = LaunchConfiguration('sim'), LaunchConfiguration('keyboard')
    return LaunchDescription([
        DeclareLaunchArgument('rover_host', default_value='192.168.0.149'),
        DeclareLaunchArgument('rover_port', default_value='9090'),
        DeclareLaunchArgument('max_speed', default_value='1.0'),
        DeclareLaunchArgument('sim', default_value='true'),
        DeclareLaunchArgument('keyboard', default_value='true'),
        Node(package='mujoco_car_sim', executable='rover_bridge', output='screen',
             parameters=[{'rover_host': LaunchConfiguration('rover_host'),
                          'rover_port': LaunchConfiguration('rover_port'),
                          'max_speed': LaunchConfiguration('max_speed')}]),
        Node(package='mujoco_car_sim', executable='sim_node', output='screen',
             condition=IfCondition(sim), additional_env={'MUJOCO_GL': 'egl'},
             parameters=[{'obstacles.count': 0, 'sensors.camera.enabled': False}]),
        Node(package='mujoco_car_sim', executable='keyboard_teleop', output='screen',
             condition=IfCondition(keyboard),
             parameters=[{'linear_speed': 0.5, 'angular_speed': 1.0, 'boost': 2.0,
                          'require_viewer_focus': True}]),
    ])
