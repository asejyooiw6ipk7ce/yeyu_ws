from setuptools import find_packages, setup

package_name = 'yeyu_gui'

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
    maintainer='yeeun',
    maintainer_email='cuhote0218@gmail.com',
    description='PyQt5 dashboard GUI for monitoring and controlling driving_node',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dashboard_node = yeyu_gui.dashboard_node:main',
        ],
    },
)
