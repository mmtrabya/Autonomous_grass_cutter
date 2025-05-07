from setuptools import setup
import os
from glob import glob

package_name = 'launch_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='Launch package for simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_publisher = launch_pkg.velocity_publisher:main',
            'location_subscriber = launch_pkg.location_subscriber:main',
            'sim = launch_pkg.launch_pkg.sim:main',
            'ultrasonic_node = launch_pkg.ultrasonic_node:main',
            'path_planning_spiral = launch_pkg.path_planning_spiral:main',
            'boundary_publisher = launch_pkg.boundary_publisher:main',
            'turtle_pose_to_gps = launch_pkg.turtle_pose_to_gps:main',
        ],
    },
)