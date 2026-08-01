from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_robot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hank',
    maintainer_email='hemrhut7@gmail.com',
    description='Vision and image processing nodes for my_robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_splitter_node = my_robot_perception.image_splitter_node:main',
            'test_camera = my_robot_perception.test_camera:main',
            'n10_lidar_node = my_robot_perception.n10_lidar_node:main'
        ],
    },
)
