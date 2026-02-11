from setuptools import find_packages, setup

package_name = 'yolo_realsense'

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
            'yolo_rs_test = yolo_realsense.yolo_rs_test:main',
            'yolo_id = yolo_realsense.yolo_id:main',
        ],
    },
)
