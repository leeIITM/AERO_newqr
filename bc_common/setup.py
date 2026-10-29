import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bc_common'

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
    description='Shared helpers: geometry, camera stream worker, image conversion, flight_controller client',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
