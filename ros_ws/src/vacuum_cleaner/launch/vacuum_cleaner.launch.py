"""
bringup.launch.py
-----------------
Starts all nodes and the Nav2 lifecycle manager. The manager configures and
activates nodes; behaviour_server controls a *separate* operational mode.

Tip: This example is didactic. Replace the stub logic inside each node with
real robot logic while keeping the same topic/service contracts.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('vacuum_cleaner'),
        'config',
        'nav2_params.yaml'
    )

    pkg_dir = get_package_share_directory('vacuum_cleaner')
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')

    slam = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim_port', default_value='23000',
            description='Porta da ZeroMQ Remote API do CoppeliaSim'),
        DeclareLaunchArgument(
            'slam', default_value='false',
            description='true mapeia com o SLAM Toolbox; false usa o mapa salvo em map'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_dir, 'maps', 'casa.yaml'),
            description='Mapa carregado quando slam:=false'),
        DeclareLaunchArgument(
            'cleaning_sensors', default_value='map',
            description='Onde a limpeza acha as paredes: map (sem o lidar) ou lidar'),
        # O robô recebe a saída do mux: o teleop tem prioridade sobre o resto.
        Node(package='vacuum_cleaner', executable='myrobot_server', name='myrobot', output='screen',
             remappings=[('/myRobot/cmd_vel', '/myRobot/cmd_vel_mux')],
             parameters=[{'sim_port': ParameterValue(
                 LaunchConfiguration('sim_port'), value_type=int)}]),
        Node(package='vacuum_cleaner', executable='cmd_vel_mux', name='cmd_vel_mux', output='screen'),
        Node(package='vacuum_cleaner', executable='behaviour_server', name='behaviour_server', output='screen'),
        # Varredura em faixas: segue o relógio da simulação, como o original.
        Node(package='vacuum_cleaner', executable='cleaning_server', name='cleaning_server', output='screen',
             parameters=[{'use_sim_time': True,
                          'sensors': LaunchConfiguration('cleaning_sensors')}]),
        Node(package='vacuum_cleaner', executable='autodocking_server', name='autodocking_server', output='screen'),
        Node(package='vacuum_cleaner', executable='charging_server', name='charging_server', output='screen'),
        Node(package='vacuum_cleaner', executable='charged_server', name='charged_server', output='screen'),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[cfg],
        ),
        # Laser no centro do robô, sem rotação em relação a base_link
        # (medido na cena simulation/scene_house.ttt).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['--x', '0.003', '--z', '0.12',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser_link'],
            parameters=[{'use_sim_time': True}],
        ),
        # Mapa pronto (slam:=false): o map_server publica o mapa salvo, e a TF
        # map -> odom é a identidade, porque a odometria vem da própria cena e
        # o frame map é a pose inicial do robô, a mesma do mapeamento.
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            condition=UnlessCondition(slam),
            parameters=[{'yaml_filename': LaunchConfiguration('map'),
                         'use_sim_time': True}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            condition=UnlessCondition(slam),
            parameters=[{'autostart': True, 'node_names': ['map_server'],
                         'use_sim_time': True}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            output='screen',
            condition=UnlessCondition(slam),
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
            parameters=[{'use_sim_time': True}],
        ),
        # Mapeamento (slam:=true): o SLAM Toolbox constrói o mapa e a TF
        # map -> odom enquanto o robô anda.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(slam),
            launch_arguments={
                'slam_params_file': os.path.join(
                    pkg_dir, 'config', 'mapper_params_online_async.yaml'),
                'use_sim_time': 'true',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': cfg,
                'use_sim_time': 'true',
                'autostart': 'true',
            }.items(),
        ),
    ])
