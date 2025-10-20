from setuptools import setup
import os
from glob import glob

package_name = 'mission_sime'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
        ('share/' + package_name + '/config', ['scripts/config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sabimou',
    maintainer_email='sabimou@ur.univ-rouen.fr',
    description='Système multi-robots ROS2 pour le projet Master SIME',
    license='MIT',
    entry_points={
        'console_scripts': [
            'r2d2_master = mission_sime.r2d2_master:main',
            'c3po_follower = mission_sime.c3po_follower:main',
            'enemy_spawner = mission_sime.enemy_spawner:main',
            'mission_supervisor = mission_sime.mission_supervisor:main',
        ],
    },
)
