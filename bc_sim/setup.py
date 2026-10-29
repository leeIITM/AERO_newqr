import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'bc_sim'


def tree(src):
    """data_files entries installing src/ (models with textures) recursively under share/bc_sim/src."""
    out = {}
    for root, _dirs, files in os.walk(src):
        if files:
            out[os.path.join('share', package_name, root)] = [os.path.join(root, f) for f in files]
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ] + tree('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AeroTHON team',
    maintainer_email='harrydybalaforever123@gmail.com',
    description='Gazebo + ArduCopter SITL simulation environment for the banner-corridor mission',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gz_camera_relay = bc_sim.gz_camera_relay:main',
        ],
    },
)
