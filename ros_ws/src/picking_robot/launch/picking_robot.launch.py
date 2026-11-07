"""Sobe a ponte do robô com garra. A cena simulation/scene_house_arm.ttt
precisa estar aberta no CoppeliaSim. O teleop roda à parte, num terminal:

    ros2 launch picking_robot picking_robot.launch.py
    ros2 run picking_robot teleop
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_port', default_value='23000',
            description='Porta da ZeroMQ Remote API do CoppeliaSim'),
        Node(package='picking_robot', executable='bridge', name='picking_bridge',
             output='screen',
             parameters=[{'sim_port': ParameterValue(
                 LaunchConfiguration('sim_port'), value_type=int)}]),
    ])
