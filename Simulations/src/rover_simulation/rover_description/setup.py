from setuptools import find_packages, setup
import os
from glob import glob
from pathlib import Path

package_name = 'rover_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    use_scm_version=True,
    setup_requires=['setuptools_scm', 'setuptools>=42'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add all path from your packages' directories below
        (os.path.join('share', package_name,'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name,'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name,'sdf'), glob('sdf/*')),
        (os.path.join('share', package_name,'scripts'), glob('scripts/*')),
        (os.path.join('share', package_name,'config'), glob('config/*')),
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='muhammad mursaleen',
    maintainer_email='muhammadmursaleen09@gmail.com',
    description='TODO: This package contains the description for 4 wheel rover robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           
        ],
    },
)
