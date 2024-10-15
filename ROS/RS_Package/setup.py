from setuptools import setup
import os
from glob import glob

package_name = 'RS_Package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Find/Install launch & config files on your system.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='morgan',
    author_email='author.morgan@saintdenisgraveyards.net',
    description='Remote Station (RS) Package for Publishing Joystick Inputs',
    license='Top Secret',
    entry_points={
        'console_scripts': [],
    },
)