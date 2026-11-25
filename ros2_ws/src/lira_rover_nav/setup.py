import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lira_rover_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Maximiliano Guzman Ortiz',
    maintainer_email='lira@ingenieria.unam.edu',
    description='Simulacion e Implementacion de Mapas de Costos 2D para el Rover LIRA',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_rover_node = lira_rover_nav.sim_rover_node:main',
        ],
    },
)
