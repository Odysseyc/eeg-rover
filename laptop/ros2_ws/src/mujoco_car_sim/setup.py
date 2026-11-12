from glob import glob
from setuptools import setup

package_name = 'mujoco_car_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.xml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'sim_node = mujoco_car_sim.sim_node:main',
        'keyboard_teleop = mujoco_car_sim.keyboard_teleop:main',
        'rover_bridge = mujoco_car_sim.rover_bridge:main',
    ]},
)
