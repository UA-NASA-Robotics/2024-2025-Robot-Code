from setuptools import find_packages, setup

package_name = 'que_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ethan',
    maintainer_email='162376649+Piglet337@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = que_pkg.control_node:main' #old, still here for references, will be deleted
            'mux_control = que_pkg.mux_control:main'
            'fifo_queue = que_pkg.fifo_queue:main'
            'macro_lut = que_pkg.macro_lut:main'
            'request_handler = que_pkg.request_handler:main'
            'feedback_processor = que_pkg.feedback_processor:main'
        ],
    },
)