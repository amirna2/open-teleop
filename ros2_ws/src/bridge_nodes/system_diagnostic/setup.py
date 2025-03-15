from setuptools import find_packages, setup
import os

package_name = 'system_diagnostic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [os.path.join('launch', 'system_diagnostics.launch.py')]),
    ],
    install_requires=['setuptools', 'psutil', 'flatbuffers', 'pyzmq'],
    zip_safe=True,
    maintainer='amir',
    maintainer_email='amir@todo.todo',
    description='ROS2 node for collecting and sending system diagnostics to the Go Controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_diagnostics_node = system_diagnostic.system_diagnostics_node:main'
        ],
    },
) 