from setuptools import find_packages, setup

package_name = 'red_zone_detector'

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
    description='HSV red / restricted-zone detector (red_zone_node from Shiva007-RAndom/Qr-and-red-zone-detection)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'red_zone_detector_node = red_zone_detector.red_zone_detector_node:main',
        ],
    },
)
