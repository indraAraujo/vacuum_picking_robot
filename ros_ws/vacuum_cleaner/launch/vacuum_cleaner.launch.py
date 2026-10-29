from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():  
    remappings = [
          ('/myRobot/cmd_vel', '/cmd_vel')]
     
    return LaunchDescription([
        Node(
            package='vacuum_cleaner',
            executable='lidar_node',
            output='screen',
        ),
        Node(
            package='vacuum_cleaner',
            executable='odometry_node',
            output='screen',
        ),
        Node(
            package='vacuum_cleaner',
            executable='subscriber_velocity',
            remappings=remappings,
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=["0.0778096911698893", "0", "0.24980849049326312", "0.0", "0.0016353724914138718", "0", "0.9999999", "base_link", "laser_link"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': PathJoinSubstitution([
                    FindPackageShare('vacuum_cleaner'),
                    'config',
                    'mapper_params_online_async.yaml'
                ]),
                'use_sim_time': 'true'
            }.items()
        )
    ])