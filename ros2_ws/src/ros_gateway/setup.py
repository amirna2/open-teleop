import os
from setuptools import setup, find_packages

package_name = 'ros_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [os.path.join('launch', f) for f in os.listdir('launch') if f.endswith('.py')]),
        (os.path.join('share', package_name, 'config'), [os.path.join('config', f) for f in os.listdir('config') if f.endswith('.yaml')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Open-Teleop Team',
    maintainer_email='maintainer@example.com',
    description='ROS Gateway for Open-Teleop - A bridge between ROS2 and the Open-Teleop Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gateway_node = ros_gateway.gateway_node:main',
        ],
    },
) 
