from setuptools import find_packages, setup

package_name = 'rosmaster'

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
    maintainer='ducky_ubuntu',
    maintainer_email='ducky_ubuntu@todo.todo',
    description='Paquete IMU para Rosmaster',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	 'imu = rosmaster.imu_node:main',
        	 'mag = rosmaster.mag_node:main',
        	 'gyro = rosmaster.gyro_node:main',
        	 'gyro_cal = rosmaster.gyro_calibrated_node:main',
        	 
        ],
    },
)
