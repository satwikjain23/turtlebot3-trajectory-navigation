from setuptools import find_packages, setup
from glob import glob

package_name = 'navigation_assignment'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='satwik',
    maintainer_email='satwik@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_smoothing_node = navigation_assignment.path_smoothing_node:main',
            'trajectory_generator_node = navigation_assignment.trajectory_generator_node:main',
            'controller_node = navigation_assignment.controller_node:main',
            'trajectory_visualizer = navigation_assignment.trajectory_visualizer_node:main',
            'obstacle_avoidance = navigation_assignment.obstacle_avoidance:main',
        ],
    },
)
