import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'human_interaction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='indra',
    maintainer_email='indra.araujo.santos@gmail.com',
    description='Interação humano-robô: comandos de voz que viram destinos no Nav2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_listener = human_interaction.voice_listener:main',
            'command_parser = human_interaction.command_parser:main',
            'command_console = human_interaction.command_console:main',
            'voice_commander = human_interaction.voice_commander:main',
            'house_mapper = human_interaction.house_mapper:main',
        ],
    },
)
