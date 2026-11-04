"""
bringup.launch.py
-----------------
Starts all nodes and the Nav2 lifecycle manager. The manager configures and
activates nodes; behaviour_server controls a *separate* operational mode.

Tip: This example is didactic. Replace the stub logic inside each node with
real robot logic while keeping the same topic/service contracts.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

    return LaunchDescription([
        Node(package='vacuum_cleaner', executable='myrobot_server', name='myrobot', output='screen'),
        Node(package='vacuum_cleaner', executable='behaviour_server', name='behaviour_server', output='screen'),
        Node(package='vacuum_cleaner', executable='cleaning_server', name='cleaning_server', output='screen'),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
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
