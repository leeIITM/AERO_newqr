import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'siyi_camera'

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
    description='Zero-latency SIYI A8 mini RTSP stream -> /camera/image_raw',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtsp_camera_node = siyi_camera.rtsp_camera_node:main',
        ],
    },
)
