import os

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_topic = LaunchConfiguration('lidar_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    base_frame = LaunchConfiguration('base_frame')

    voxel_size = float(LaunchConfiguration('voxel_size').perform(context))
    localization = LaunchConfiguration('localization').perform(context).lower() == 'true'
    # If the robot/sim already publishes odom->base_link, skip icp_odometry
    external_odom = LaunchConfiguration('external_odom').perform(context).lower() == 'true'
    delete_db = LaunchConfiguration('delete_db').perform(context).lower() == 'true'
    database_path = os.path.expanduser(LaunchConfiguration('database_path').perform(context))
    os.makedirs(os.path.dirname(database_path), exist_ok=True)

    icp_parameters = {
        'Icp/VoxelSize': str(voxel_size),
        'Icp/PointToPlane': 'true',
        'Icp/PointToPlaneK': '20',
        'Icp/PointToPlaneRadius': '0',
        'Icp/Iterations': '10',
        'Icp/Epsilon': '0.001',
        'Icp/MaxTranslation': '2',
        'Icp/MaxCorrespondenceDistance': str(voxel_size * 10.0),
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/Strategy': '1',
        'Icp/OutlierRatio': '0.7',
    }

    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': base_frame,
        'odom_frame_id': 'odom',
        'wait_for_transform': 0.2,
        'approx_sync': False,
        **icp_parameters,
    }

    icp_odometry_parameters = {
        'publish_tf': True,
        'wait_imu_to_init': True,
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'Odom/ScanKeyFrameThr': '0.4',
        'OdomF2M/ScanSubtractRadius': str(voxel_size),
        'OdomF2M/ScanMaxSize': '15000',
        'OdomF2M/BundleAdjustment': 'false',
    }

    rtabmap_parameters = {
        'map_frame_id': 'map',
        'publish_tf': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan_cloud': True,
        'subscribe_odom_info': not external_odom,
        'database_path': database_path,
        'topic_queue_size': 40,
        'sync_queue_size': 40,
        # Lidar-only registration and loop closure refinement
        'Reg/Strategy': '1',
        'Reg/Force3DoF': 'true',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/ProximityMaxGraphDepth': '0',
        'RGBD/ProximityPathMaxNeighbors': '1',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/CreateOccupancyGrid': 'true',
        'Mem/NotLinkedNodesKept': 'false',
        'Mem/STMSize': '30',
        # 3D occupancy grid
        'Grid/Sensor': '0',
        'Grid/3D': 'true',
        'Grid/RayTracing': 'true',
        'Grid/CellSize': '0.05',
        'Grid/RangeMax': '10.0',
        'Grid/MaxGroundHeight': '0.0',
        'Grid/MaxObstacleHeight': '1.0',
        'Grid/NormalsSegmentation': 'true',
        'Grid/MaxGroundAngle': '30',
        'Grid/MinClusterSize': '10',
        'Grid/ClusterRadius': '0.1',
        'Grid/NormalK': '20',
        'Grid/MinGroundHeight': '-0.3',
        'Grid/NoiseFilteringRadius': '0.1',
        'Grid/NoiseFilteringMinNeighbors': '5',
        'Grid/RangeMin': '0.4',
    }

    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
    else:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'True'
        if delete_db:
            arguments.append('-d')

    cloud_assembler_parameters = {
        'use_sim_time': use_sim_time,
        'queue_size': 15,
        'fixed_frame_id': 'odom' if external_odom else '',
        'frame_id': '',
        'max_clouds': 0,
        'assembling_time': LaunchConfiguration('assembling_time'),
        'range_min': 0.3,
        'range_max': 15.0,
        'voxel_size': voxel_size,
    }

    nodes = [
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[cloud_assembler_parameters],
            remappings=[('cloud', lidar_topic),
                        ('odom', '/odom')]),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters],
            remappings=[('scan_cloud', 'assembled_cloud'),
                        ('imu', imu_topic),
                        ('odom', '/odom'),
                        ('map', '/map')],
            arguments=arguments),
    ]

    if not external_odom:
        nodes.insert(0, Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[shared_parameters, icp_odometry_parameters],
            remappings=[('scan_cloud', lidar_topic),
                        ('imu', imu_topic),
                        ('odom', '/odom')]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulated clock.'),

        DeclareLaunchArgument(
            'lidar_topic', default_value='/lidar_data_points',
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

        OpaqueFunction(function=launch_setup),
    ])
