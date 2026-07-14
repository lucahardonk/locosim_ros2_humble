from setuptools import setup, find_packages

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michele Focchi',
    maintainer_email='michele.focchi@unitn.it',
    description='Locosim robot control library (ROS2 Humble port).',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_controller = robot_control.base_controller:main',
            'quadruped_controller = robot_control.quadruped_controller:main',
        ],
    },
)
