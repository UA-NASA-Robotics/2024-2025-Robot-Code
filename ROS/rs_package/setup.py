from setuptools import setup
import os
from glob import glob

package_name = 'rs_package'

# Helps build & install w/ "setuptools". 
# (Similar to CMakeLists.txt for C++)
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Find/Install launch & config files on your system.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Remote Station (RS) Package for Publishing Joystick Inputs',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)