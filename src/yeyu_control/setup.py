from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'yeyu_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', 'yeyu_waypoint_nav', 'waypoints'), glob('waypoints/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeeun',
    maintainer_email='cuhote0218@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'parking_node = yeyu_control.parking_node:main',
            'function_node = yeyu_control.function_node:main',
            'main_node = yeyu_control.main_node:main',
            'signal_test_node = yeyu_control.signal_test_node:main',
            'driving_waypoint_node = yeyu_control.driving_waypoint_node:main',
            'charging_dock_node = yeyu_control.charging_dock_node:main',


        ],
    },
)
