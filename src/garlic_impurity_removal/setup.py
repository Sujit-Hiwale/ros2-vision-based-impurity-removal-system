from setuptools import setup
import os
from glob import glob

package_name = 'garlic_impurity_removal'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='sujit',
    maintainer_email='sujit@example.com',
    description='Garlic impurity removal robotic system using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),

        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
    ],
    entry_points={
        'console_scripts': [
            'patchcore_node = garlic_impurity_removal.patchcore_node:main',
            'coordinate_node = garlic_impurity_removal.coordinate_node:main',
            'tracking_node = garlic_impurity_removal.tracking_node:main',
            'motion_planner_node = garlic_impurity_removal.motion_planner_node:main',
            'delta_hand_node = garlic_impurity_removal.delta_hand_node:main',

            # Camera nodes
            'camera_node = garlic_impurity_removal.camera_node:main',
            
            #'scan_scheduler_node = garlic_impurity_removal.scan_scheduler_node:main',
            
            # Debug view
            'camera_view_node = garlic_impurity_removal.camera_view_node:main',
        ],
    },
)