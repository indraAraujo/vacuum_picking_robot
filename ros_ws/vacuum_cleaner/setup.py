from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vacuum_cleaner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='indra',
    maintainer_email='indra.araujo.santos@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "subscriber_velocity = vacuum_cleaner.velocity_node:main",
            "lidar_node = vacuum_cleaner.lidar:main",
            "tf_node_slam_toolbox = vacuum_cleaner.tf_node:main",
            "odometry_node = vacuum_cleaner.odometry_node:main",
        ],
    },
)
