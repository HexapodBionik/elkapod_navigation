import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    elkapod_share = get_package_share_directory('elkapod_elevation_mapping')
    cupy_share = get_package_share_directory('elevation_mapping_cupy')

    core_param_path = os.path.join(cupy_share, 'config', 'core', 'core_param.yaml')
    default_config_path = os.path.join(elkapod_share, 'config', 'elkapod_setup.yaml')
    occupancy_grid_config_path = os.path.join(elkapod_share, 'config', 'occupancy_grid.yaml')
    default_rviz_path = os.path.join(elkapod_share, 'rviz', 'elevation_mapping.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')

    elevation_mapping_node = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            core_param_path,
            LaunchConfiguration('config'),
            {
                'use_sim_time': use_sim_time,
            },
        ],
    )

    occupancy_grid_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        output='screen',
        emulate_tty=True,
        parameters=[
            occupancy_grid_config_path,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock'),
        DeclareLaunchArgument(
            'config', default_value=default_config_path,
            description='Elkapod elevation mapping setup yaml, overrides core_param.yaml from elevation_mapping_cupy'),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false',
            description='Start RViz with the elevation map displays.'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz_path,
            description='RViz config file.'),
        elevation_mapping_node,
        occupancy_grid_node,
        rviz_node,
    ])
