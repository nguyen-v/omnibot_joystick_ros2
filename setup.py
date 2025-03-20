from setuptools import setup
import os
from glob import glob

package_name = 'joy_to_cmd_vel'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A node that maps joy messages to TwistStamped commands for omnibot.',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_cmd_vel_node = joy_to_cmd_vel.joy_to_cmd_vel_node:main',
        ],
    },
)
