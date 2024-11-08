from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'rs_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        # Install configuration files to be used by ROS
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'), [f'{package_name}']),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='morgan',
    author_email='author.morgan@saintdenisgraveyards.net',
    description='Remote Station (RS) Package for Publishing Joystick Inputs',
    license='Top Secret',
    entry_points={
        'console_scripts': [
            'joystick_tracks = rs_package:main',  # This links to the `main` function in rs_package/__init__.py
        ],
    },
)
