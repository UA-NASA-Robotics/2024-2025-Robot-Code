from setuptools import find_packages, setup

package_name = 'queue_package'

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
            'control_node = que_packagege.control_node:main',
            'mux_control = que_packagege.mux_control:main',
            'fifo_queue = que_packagege.fifo_queue:main',
            'macro_lut = que_packagege.macro_lut:main',
            'request_handler = que_packagege.request_handler:main',
            'feedback_processor = que_packagege.feedback_processor:main'
        ],
    },
) 