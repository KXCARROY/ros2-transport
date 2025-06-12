import os
from glob import glob
from setuptools import setup

package_name = 'transport_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
       ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name), ['resource/' + package_name]),
       (os.path.join('share', package_name, 'action'), glob('action/*.action')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carroy',
    maintainer_email='kcarroyarvy@gmail.com',
    description='Camera, Arm nodes and mouvement',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
           'camera_node = transport_system.camera_node:main',
           'arm_node = transport_system.arm_node:main',
           'robot_node = transport_system.robot_node:main',
        ],
    },
)
