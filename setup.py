import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_autorace_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='ywh@robotis.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_trajectory_publisher = turtlebot3_autorace_arm.joint_trajectory_publisher:main',
            'velocity_command_publisher = turtlebot3_autorace_arm.velocity_command_publisher:main',
        ],
    },
)
