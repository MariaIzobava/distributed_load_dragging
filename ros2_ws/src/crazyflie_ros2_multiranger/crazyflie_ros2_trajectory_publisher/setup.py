from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_trajectory_publisher'

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
    maintainer='maryia',
    maintainer_email='m.izobava@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = crazyflie_ros2_trajectory_publisher.trajectory_publisher:main',
            'load_path_publisher = crazyflie_ros2_trajectory_publisher.load_path_publisher:main'
        ],
    },
)
