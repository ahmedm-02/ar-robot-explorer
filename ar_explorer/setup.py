import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ar_explorer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='ahmed@todo.todo',
    description='AR search-and-rescue pipeline',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'iphone_camera_bridge = ar_explorer.iphone_camera_bridge:main',
            'tag_to_marker        = ar_explorer.tag_to_marker:main',
            'calibration_server   = ar_explorer.calibration_server:main',
            'calibrated_forwarder = ar_explorer.calibrated_forwarder:main',
            'calibration_check    = ar_explorer.calibration_check:main',
            'run_calibration      = ar_explorer.run_calibration:main',
            'ar_marker_publisher  = ar_explorer.ar_marker_publisher:main',
        ],
    },
)
