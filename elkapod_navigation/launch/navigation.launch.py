import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

SLAM_ARGUMENTS = [
    'use_sim_time',
    'lidar_topic',
    'imu_topic',
    'base_frame',
    'voxel_size',
    'expected_update_rate',
    'assembling_time',
    'localization',
    'external_odom',
    'database_path',
    'delete_db',
]


def generate_launch_description():
    pkg_dir = get_package_share_directory('elkapod_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    elkapod_slam_dir = get_package_share_directory('elkapod_slam')
    elevation_mapping_dir = get_package_share_directory('elkapod_elevation_mapping')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_topic = LaunchConfiguration('lidar_topic')

    params_file = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'topic': lidar_topic},
        convert_types=True)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulated clock.'),

        DeclareLaunchArgument(
            'lidar_topic', default_value='/scan',
            description='Name of the lidar PointCloud2 topic.'),

        DeclareLaunchArgument(
            'imu_topic', default_value='/imu',
            description='Name of an IMU topic.'),

        DeclareLaunchArgument(
            'base_frame', default_value='base_link',
            description='Base frame of the robot.'),

        DeclareLaunchArgument(
            'voxel_size', default_value='0.05',
            description='Voxel size (m) of the downsampled lidar point cloud.'),

        DeclareLaunchArgument(
            'expected_update_rate', default_value='15.0',
            description='Expected lidar frame rate, slightly higher than actual.'),

        DeclareLaunchArgument(
            'assembling_time', default_value='1.0',
            description='How long (s) scans are assembled before being sent to rtabmap.'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Localization mode (reuse database_path instead of mapping).'),

        DeclareLaunchArgument(
            'external_odom', default_value='false',
            description='Use odom->base_link TF and /odom from the robot/sim instead of icp_odometry.'),

        DeclareLaunchArgument(
            'database_path', default_value='~/.ros/rtab_map.db',
            description='RTAB-Map database path.'),

        DeclareLaunchArgument(
            'delete_db', default_value='false',
            description='Delete the RTAB-Map database at startup (fresh map).'),

        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_dir, 'config', 'isaac_nav_params.yaml'),
            description='Nav2 parameters file.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(elkapod_slam_dir, 'launch', 'elkapod_rtabmap_slam.launch.py')),
            launch_arguments={
                name: LaunchConfiguration(name) for name in SLAM_ARGUMENTS
            }.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(elevation_mapping_dir, 'launch', 'elevation_mapping.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': params_file,
                'autostart': 'true',
            }.items()),
    ])
