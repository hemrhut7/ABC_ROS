from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ins_ekf'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        'ins_ekf',
        'nav_ekf',
        'nav_ekf.core',
        'nav_ekf.filters',
        'nav_ekf.framework',
        'nav_ekf.sensors',
        'nav_ekf.utils',
        'nav_ekf.utils.plot_utils',
    ],
    package_dir={
        'ins_ekf': 'ins_ekf',
        'nav_ekf': '../INS_python/nav_ekf',
        'nav_ekf.core': '../INS_python/nav_ekf/core',
        'nav_ekf.filters': '../INS_python/nav_ekf/filters',
        'nav_ekf.framework': '../INS_python/nav_ekf/framework',
        'nav_ekf.sensors': '../INS_python/nav_ekf/sensors',
        'nav_ekf.utils': '../INS_python/nav_ekf/utils',
        'nav_ekf.utils.plot_utils': '../INS_python/nav_ekf/utils/plot_utils',
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.22.0',
        'scipy>=1.8.0',
        'matplotlib',
        'pandas',
        'numba',
        'pygeomag',
        'pymavlink',
        'tqdm',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='hank',
    maintainer_email='hank@todo.todo',
    description='INS/GNSS Error State Kalman Filter state estimation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ins_ekf_node = ins_ekf.ins_ekf_node:main'
        ],
    },
)
