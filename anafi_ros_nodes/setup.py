#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup

package_name = 'anafi_ros_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('scripts/*.sh'))
    ],
    install_requires=['setuptools'],  # pip install setuptools==58.2.0
    zip_safe=True,
    maintainer='andriy',
    maintainer_email='andriy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'anafi = anafi_ros_nodes.anafi:main',
        	'test_anafi = anafi_ros_nodes.test_anafi:main',
        	'sphinx = anafi_ros_nodes.sphinx:main',
        	'example = anafi_ros_nodes.example:main',
        ],
    },
)
