import os
import glob
from setuptools import find_packages, setup

package_name = 'ros2_exploration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob.glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'rviz'),
         glob.glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhi Yan',
    maintainer_email='zhi.yan@ensta.fr',
    description='A simple ROS 2 frontier-based exploration package.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'exploration_node = ros2_exploration.exploration_node:main',
            'twist_converter = ros2_exploration.twist_converter:main' 
        ],
    },
)
