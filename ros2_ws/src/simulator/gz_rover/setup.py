from setuptools import find_packages, setup

package_name = 'gz_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/spawn_rover.launch.py']),
        ('share/' + package_name, ['resource/Robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danielgrioja',
    maintainer_email='angel.galicia@ingenieria.unam.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
