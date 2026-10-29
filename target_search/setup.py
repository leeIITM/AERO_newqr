from setuptools import find_packages, setup

package_name = 'target_search'

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
    description='Delivery-zone QR search with red-zone avoidance (suas_aer target_search + Shiva007-RAndom avoidance)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_search_node = target_search.target_search_node:main',
            'lawnmower_search_node = target_search.lawnmower_search_node:main',
        ],
    },
)
