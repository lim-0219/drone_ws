from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'drone_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Installed so that `ros2 launch drone_control ...` and the default
        # waypoint file resolve through the package share directory.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lim',
    maintainer_email='joan0219@yonsei.ac.kr',
    description='PX4 offboard waypoint navigation and survey mission for ROS 2 Humble.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_navigator = drone_control.waypoint_navigator:main',
            'rangefinder_params = drone_control.rangefinder_params:main',
            'check_range_fusion = drone_control.check_range_fusion:main',
        ],
    },
)
