from setuptools import find_packages, setup

package_name = 'team52'

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
    maintainer='celian',
    maintainer_email='celian@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = team52.node:main',
            'lidar_converter = team52.lidar_converter:main',
            'gps_converter = team52.gps_converter:main',
            'allies_detection = team52.allies_detection:main',
            'beacon_converter = team52.beacon_converter:main',
            'pid = team52.Pid:main',
            'coord = team52.coord_pub_debug:main',
            'pathfinder = team52.pathfinder:main',
            'brain = team52.brain:main',
            'camera = team52.camera:main',
        ],
    },
)
