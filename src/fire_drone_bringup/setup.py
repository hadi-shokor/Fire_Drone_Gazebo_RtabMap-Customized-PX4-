from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'fire_drone_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*')
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')
        ),
        (
            os.path.join('share', package_name, 'meshes'),
            glob('meshes/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='RoboLI@aub.edu.lb',
    description='Fire drone bringup package',
    license='TODO',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'px4_odom_tf_publisher = fire_drone_bringup.px4_odom_tf_publisher:main',
            'generate_runtime_urdf = fire_drone_bringup.runtime_sdf_robot_state_publisher:main',
            'velodyne_time_adapter = fire_drone_bringup.velodyne_time_adapter:main',
            'px4_odom_to_ros = fire_drone_bringup.px4_odom_to_ros:main',
            'initial_scan_mission = fire_drone_bringup.initial_scan_mission:main',
        ],
    },
)
