"""Teleoperação por teclado do robô aspirador.

Abre o teleop_twist_keyboard em uma janela xterm (ele precisa de um terminal
interativo para ler as teclas) e publica geometry_msgs/Twist em
/cmd_vel_teleop. O nó cmd_vel_mux dá prioridade a esse tópico sobre o /cmd_vel
do Nav2 e repassa o resultado para o subscriber_velocity.

Uso:
    ros2 launch vacuum_cleaner teleop.launch.py
    ros2 launch vacuum_cleaner teleop.launch.py speed:=0.3 turn:=1.5
    ros2 launch vacuum_cleaner teleop.launch.py terminal_prefix:=''  # roda no próprio terminal
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    terminal_prefix = LaunchConfiguration('terminal_prefix')

    return LaunchDescription([
        DeclareLaunchArgument(
            'speed', default_value='0.2',
            description='Velocidade linear inicial (m/s)'),
        DeclareLaunchArgument(
            'turn', default_value='1.0',
            description='Velocidade angular inicial (rad/s)'),
        DeclareLaunchArgument(
            'cmd_vel_topic', default_value='/cmd_vel_teleop',
            description='Tópico de entrada de teleop do cmd_vel_mux'),
        DeclareLaunchArgument(
            'terminal_prefix', default_value='xterm -title teleop -e',
            description="Prefixo para abrir o teleop em outro terminal ('' para usar o atual)"),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix=terminal_prefix,
            parameters=[{
                'speed': ParameterValue(speed, value_type=float),
                'turn': ParameterValue(turn, value_type=float),
                'stamped': False,
            }],
            remappings=[('cmd_vel', cmd_vel_topic)],
        ),
    ])
