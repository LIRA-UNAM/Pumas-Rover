from setuptools import setup

package_name = 'rover'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', [
    'launch/rover_launch.py',
    'launch/rover_launch2.py'
]))
data_files.append(('share/' + package_name + '/worlds', ['worlds/4_wheeled_robot.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_rover.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='angel.galicia@ingenieria.unam.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_rover_driver = rover.my_rover_driver:main',
            'rover_ros2 = rover.rover_ros2:main',
            'camera_publisher = rover.camera:main'
        ],
    },
)