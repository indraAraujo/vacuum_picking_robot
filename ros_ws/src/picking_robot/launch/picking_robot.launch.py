"""Robô com garra: ponte, bananas, Nav2 e o recolhimento das bananas.

A cena simulation/scene_house_arm.ttt precisa estar aberta no CoppeliaSim.

    ros2 launch picking_robot picking_robot.launch.py              # tudo
    ros2 launch picking_robot picking_robot.launch.py picker:=false # sem recolher
    ros2 launch picking_robot picking_robot.launch.py bananas:=0    # sem bananas

O teleop roda à parte, num terminal (ros2 run picking_robot teleop). Use-o
com picker:=false, senão os dois disputam o robô.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('picking_robot')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    port = ParameterValue(LaunchConfiguration('sim_port'), value_type=int)
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('sim_port', default_value='23000',
                              description='Porta da ZeroMQ Remote API do CoppeliaSim'),
        DeclareLaunchArgument('bananas', default_value='8',
                              description='Quantas bananas espalhar pelo chão'),
        DeclareLaunchArgument('picker', default_value='true',
                              description='Recolher as bananas sozinho'),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='Semente do sorteio das bananas (-1: aleatório)'),
        DeclareLaunchArgument('positions', default_value='[0.0]',
                              description='Bananas em posições fixas no map, no lugar do '
                                          'sorteio: "[x1, y1, x2, y2]"'),

        Node(package='picking_robot', executable='bridge', name='picking_bridge',
             output='screen', parameters=[{'sim_port': port}]),
        Node(package='picking_robot', executable='world', name='world', output='screen',
             parameters=[sim_time, {
                 'sim_port': port,
                 'count': ParameterValue(LaunchConfiguration('bananas'), value_type=int),
                 'seed': ParameterValue(LaunchConfiguration('seed'), value_type=int),
                 'positions': ParameterValue(LaunchConfiguration('positions'))}]),

        # O frame map é a pose inicial do robô, e a odometria vem da cena: a
        # TF map -> odom é a identidade. A cena não tem laser, mas o Nav2
        # (config/nav2_params.yaml, adaptado do vacuum_cleaner) espera um: a
        # ponte publica um /scan vazio em laser_link.
        Node(package='tf2_ros', executable='static_transform_publisher', name='map_to_odom',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[sim_time]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--z', '0.12', '--frame-id', 'base_link', '--child-frame-id', 'laser_link'],
             parameters=[sim_time]),
        # O mapa (/map) vem do nó world: o da casa, com o cesto como obstáculo.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items(),
        ),
        Node(package='picking_robot', executable='picker', name='picker', output='screen',
             condition=IfCondition(LaunchConfiguration('picker')),
             parameters=[sim_time]),
    ])
