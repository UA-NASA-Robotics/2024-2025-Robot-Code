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
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Find/Install launch & config files on your system.
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='morgan',
    author_email='author.morgan@saintdenisgraveyards.net',
    description='Remote Station (RS) Package for Publishing Joystick Inputs',
    license='Top Secret',
    entry_points={
        'console_scripts': [
            "skid_steer = rs_package.skid_steer:main",
            "separate_twistplus = rs_package.twist_plus_separation:main"
        ],
    },
)