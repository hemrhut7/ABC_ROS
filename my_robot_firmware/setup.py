from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_firmware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # We can also support launch files inside my_robot_firmware if needed
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hank',
    maintainer_email='hank@todo.todo',
    description='Firmware interface and serial telemetry nodes for my_robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_serial_node = my_robot_firmware.esp32_serial_node:main'
        ],
    },
)
