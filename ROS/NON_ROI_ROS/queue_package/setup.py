from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'queue_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        ('install/' + package_name + '/lib/python3.12/site-packages/odrive/lib/libodrive-linux-x86_64.so', ['../../gahh/lib/python3.12/site-packages/odrive/lib/libodrive-linux-x86_64.so'])
    ],
    install_requires=['setuptools', 'odrive', 'sensor_msgs'],
    zip_safe=True,
    maintainer='Ethan',
    maintainer_email='162376649+Piglet337@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = queue_package.control_node:main',
            'mux_control = queue_package.mux_control:main',
            'fifo_queue = queue_package.fifo_queue:main',
            'macro_lut = queue_package.macro_lut:main',
            'feedback_processor = queue_package.feedback_processor:main',
            'macro_conflicts = queue_package.macro_conflicts:main',
            'ODriveOrin = queue_package.ODriveOrin:main'
        ],
    },
) 