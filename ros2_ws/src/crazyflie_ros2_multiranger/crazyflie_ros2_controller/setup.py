from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_controller'

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
            'mpc = crazyflie_ros2_controller.mpc:main',
            'mpc_graph = crazyflie_ros2_controller.mpc_with_factor_graph:main'
        ],
    },
)
