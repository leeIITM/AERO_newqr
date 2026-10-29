from setuptools import find_packages, setup

package_name = 'payload_drop'

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
    description='Payload drop (two PWM commands) and geotagged survey frames for the target search',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'payload_drop_node = payload_drop.payload_drop_node:main',
            'survey_capture_node = payload_drop.survey_capture_node:main',
        ],
    },
)
