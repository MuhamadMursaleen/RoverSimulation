from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover_simulation_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name,'models'), glob('models/*.config')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='muhammadmursaleen09@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_rover = rover_simulation_control.test_rover:main',
            'rps_publisher_controller = rover_simulation_control.rps_publisher_controller:main',
            'cmd_publisher_controller = rover_simulation_control.cmd_publisher_controller:main',
            'digital_twin_node = rover_simulation_control.digital_twin_node:main'
        ],
    },
)
