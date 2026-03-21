import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mobile_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='javier',
    maintainer_email='javier.diazrivera551@gmail.com',
    description='TODO: Package description',
    license='LGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mobile_base = mobile_base.mobile_base:main',
            'speed_keyboard = mobile_base.speed_keyboard:main',
            'remote_control = mobile_base.remote_control:main',
            'odometry_listener = mobile_base.odometry_listener:main',
            'path_planner = mobile_base.path_planner:main',
            'objective_path = mobile_base.objective_path:main',
        ],
    },
)
