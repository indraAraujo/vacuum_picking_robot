from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'picking_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='indra',
    maintainer_email='indra.araujo.santos@gmail.com',
    description='Robô aspirador com a garra do youBot: cena, ponte ROS 2 e teleoperação',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge = picking_robot.bridge:main',
            'teleop = picking_robot.teleop:main',
            'build_scene = picking_robot.build_scene:main',
        ],
    },
)
