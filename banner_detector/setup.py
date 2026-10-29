import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'banner_detector'

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
    description='Colour-first, strict-shape green/blue banner detector with async RapidOCR; vision tuning tool',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'banner_detector_node = banner_detector.banner_detector_node:main',
            'vision_test = banner_detector.vision_test:main',
        ],
    },
)
