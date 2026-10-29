import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'flight_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AeroTHON team',
    maintainer_email='harrydybalaforever123@gmail.com',
    description='Single owner of ArduPilot GUIDED/AUTO control (via MAVROS): Takeoff, GoTo, Land actions, body velocity commands and the /flight_controller/pose vehicle state',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flight_controller_node = flight_controller.flight_controller_node:main',
        ],
    },
)
