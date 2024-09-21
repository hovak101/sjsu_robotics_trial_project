from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_finder'

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
    maintainer='hovak',
    maintainer_email='hovak@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "goal_and_start_node = path_finder.goal_and_start_node:main",
            "path_node = path_finder.path_node:main",
            "map_node = path_finder.map_node:main",
            "viz_node = path_finder.viz_node:main"
        ],
    },
)
