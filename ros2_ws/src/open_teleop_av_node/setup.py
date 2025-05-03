import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'open_teleop_av_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Add config file installation if needed later
        # (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Open Teleop Maintainer',
    maintainer_email='maintainer@example.com',
    description='Open Teleop A/V Encoding Node',
    license='Apache-2.0', # Or your chosen license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'av_node = open_teleop_av_node.av_node:main',
        ],
    },
) 