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
            'behaviour_server = vacuum_cleaner.behaviour_server:main',
            'myrobot_server = vacuum_cleaner.myrobot_server:main',
            'cleaning_server = vacuum_cleaner.cleaning_server:main',
            'autodocking_server = vacuum_cleaner.autodocking_server:main',
            'charging_server = vacuum_cleaner.charging_server:main',
            'charged_server = vacuum_cleaner.charged_server:main',
            'cmd_vel_mux = vacuum_cleaner.cmd_vel_mux:main',
        ],
    },
)
