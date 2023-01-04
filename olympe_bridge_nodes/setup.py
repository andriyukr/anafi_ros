#!/usr/bin/env python3

import os
from glob import glob
from setuptools import setup

package_name = 'olympe_bridge_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
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
        	'anafi = olympe_bridge_nodes.anafi:main',
        	'test_anafi = olympe_bridge_nodes.test_anafi:main',
        	'collect_gimbal = olympe_bridge_nodes.collect_gimbal:main',
        	'collect_yaw = olympe_bridge_nodes.collect_yaw:main',
        ],
    },
)
