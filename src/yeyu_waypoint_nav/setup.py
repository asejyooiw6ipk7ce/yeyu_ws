from setuptools import find_packages, setup

package_name = 'yeyu_waypoint_nav'

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
    maintainer='ktel',
    maintainer_email='asejyooiw6ipk7ce@gmail.com',
    description='YEYU waypoint recording/following utilities',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'amcl_waypoint_recorder = yeyu_waypoint_nav.amcl_waypoint_recorder:main',
        ],
    },
)
