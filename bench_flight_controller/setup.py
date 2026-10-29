import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bench_flight_controller'

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
    description='Drop-in bench replacement for flight_controller (simulated kinematics, optional real motor spin in GUIDED_NOGPS) and a synthetic downward camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bench_controller_node = bench_flight_controller.bench_controller_node:main',
            'sim_camera_node = bench_flight_controller.sim_camera_node:main',
        ],
    },
)
