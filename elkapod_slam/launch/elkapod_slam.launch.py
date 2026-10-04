import datetime
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):
    namespace = 'perception'

    elkapod_slam_dir = get_package_share_directory('elkapod_slam')

    database_name = LaunchConfiguration('rtab_db').perform(context)
    database_file_name = datetime.datetime.strftime(
        datetime.datetime.now(), "%d_%m_%Y_%H_%M") + ".db"
    if database_name:
        database_file_name = database_name if database_name.endswith(
            ".db") else database_name + '.db'
    os.makedirs('/elkapod_sim_ws/data', exist_ok=True)
    # print(f"Using {database_file_name}")

    rtabmap_ini = LaunchConfiguration('rtabmap_ini').perform(context)
    rtabmap_config_path = rtabmap_ini if rtabmap_ini else os.path.join(elkapod_slam_dir,'config','rtabmap.ini')
    print(rtabmap_config_path)
    frame_id = LaunchConfiguration('frame_id')

    external_odom_frame_id = LaunchConfiguration(
        'external_odom_frame_id').perform(context)

    fixed_frame_from_imu = False
    fixed_frame_id = LaunchConfiguration('fixed_frame_id').perform(context)
    if not fixed_frame_id:
        if external_odom_frame_id:
            fixed_frame_id = external_odom_frame_id
        else:
            fixed_frame_from_imu = True
            fixed_frame_id = frame_id.perform(context) + "_stabilized"

    imu_topic = LaunchConfiguration('imu_topic')

    rgbd_image_topic = LaunchConfiguration('rgbd_image_topic')
    rgbd_images_topic = LaunchConfiguration('rgbd_images_topic')
    rgbd_image_used = rgbd_image_topic.perform(
        context) != '' or rgbd_images_topic.perform(context) != ''
    rgbd_cameras = 0 if rgbd_images_topic.perform(context) != '' else 1

    lidar_topic = LaunchConfiguration('lidar_topic')
    lidar_topic_value = lidar_topic.perform(context)
    lidar_topic_deskewed = lidar_topic_value + "/deskewed"

    voxel_size = LaunchConfiguration('voxel_size')
    voxel_size_value = float(voxel_size.perform(context))

    use_sim_time = LaunchConfiguration('use_sim_time')

    localization = LaunchConfiguration('localization').perform(context)
    localization = localization == 'true' or localization == 'True'

    # Rule of thumb:
    # max_correspondence_distance = voxel_size_value * 10.0

    shared_parameters = {
        'use_sim_time': use_sim_time,
        'frame_id': frame_id,
        'qos': LaunchConfiguration('qos'),
        'approx_sync': rgbd_image_used,
        'wait_for_transform': 0.2,
        # RTAB-Map's internal parameters are strings:
    }

    icp_odometry_parameters = {
        'expected_update_rate': LaunchConfiguration('expected_update_rate'),
        'wait_imu_to_init': True,
        'odom_frame_id': 'icp_odom',
        'guess_frame_id': fixed_frame_id,
        # RTAB-Map's internal parameters are strings:
    }

    cloud_assembler_parameters = {
        'use_sim_time': use_sim_time,
        "queue_size" : 15,
        'fixed_frame_id': (external_odom_frame_id if external_odom_frame_id else ""),
        "frame_id" : "",
        "max_clouds" : 0,
        'assembling_time': LaunchConfiguration('assembling_time'),
        "skip_clouds":  0,
        "circular_buffer":  False,
        "linear_update":  0.0,
        "angular_update":  0.0,
        "wait_for_transform_duration":  0.1,
        "range_min":  0.5,
        "range_max":  15.0,
        "voxel_size":  voxel_size_value,
        "noise_radius":  0.0,
        "noise_min_neighbors":  5
    }

    rtabmap_parameters = {
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_odom_info': not external_odom_frame_id,
        'subscribe_scan_cloud': True,
        'odom_frame_id': "icp_odom",
        # This will adjust camera position based on difference between lidar and camera stamps.
        'odom_sensor_sync': True,
    }

    remappings = [('imu', imu_topic),
                  ('odom', 'icp_odom')]
    if rgbd_image_used:
        if rgbd_cameras == 1:
            remappings.append(
                ('rgbd_image', LaunchConfiguration('rgbd_image_topic')))
        else:
            remappings.append(
                ('rgbd_images', LaunchConfiguration('rgbd_images_topic')))

    arguments = []
    if localization:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'False'
        rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'True'
        rtabmap_parameters['Rtabmap/MemoryThr'] = '0'
    else:
        rtabmap_parameters['Mem/IncrementalMemory'] = 'True'
        rtabmap_parameters['Rtabmap/MemoryThr'] = '200'
        # rtabmap_parameters['Mem/InitWMWithAllNodes'] = ''
        arguments.append('-d')

    if external_odom_frame_id:
        viz_topic = lidar_topic_deskewed
    else:
        viz_topic = 'odom_filtered_input_scan'

    nodes = [
        # Assemble deskewed scans based on icp odometry
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[cloud_assembler_parameters],
            remappings=[('cloud', lidar_topic),
                        ('odom', 'icp_odom')],
            namespace=namespace),

        # Update the map
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'config_path': rtabmap_config_path,
                         'subscribe_rgbd': rgbd_image_used,
                         'rgbd_cameras': rgbd_cameras,
                         'topic_queue_size': 40,
                         'sync_queue_size': 40,
                        #  'odom': 'odometry/filtered',
                         'database_path': f'/elkapod_sim_ws/data/{database_file_name}', }],
            remappings=[('imu', imu_topic),
                    ('odom', 'icp_odom'),
                    ('scan_cloud', 'assembled_cloud')],
            arguments=arguments,
            namespace=namespace),

        # Just for visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[shared_parameters, rtabmap_parameters,
                        {'config_path': rtabmap_config_path, }],
            condition=IfCondition(LaunchConfiguration('use_rtabmap_viz')),
            remappings=remappings + [('scan_cloud', viz_topic)],
            namespace=namespace),

        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[shared_parameters, icp_odometry_parameters,
                        {'config_path': rtabmap_config_path,
                         'publish_tf': True
                        }],
            remappings= [('imu', imu_topic),
                         ('odom', 'icp_odom'),
                         ('scan_cloud', lidar_topic)],
            arguments=['--ros-args', '--log-level', 'warn'],
            namespace=namespace),

        Node(
            package='rtabmap_util', executable='imu_to_tf', output='screen',
            parameters=[{'config_path': rtabmap_config_path,
                         'use_sim_time': use_sim_time,
                         'fixed_frame_id': fixed_frame_id,
                         'base_frame_id': frame_id,
                         'wait_for_transform_duration': 0.001}],
            remappings=[('imu/data', imu_topic)],
            namespace=namespace)
    ]
    odom_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            elkapod_slam_dir, 'launch', 'elkapod_odom_fusion.launch.py'
        )]),
        launch_arguments={
        'namespace': namespace,
        'sim_mode': use_sim_time}.items()
        )

    multiple_odoms = LaunchConfiguration('fuse_odoms').perform(context)

    if multiple_odoms.lower() in ('false', '0'):
        return [*nodes]
    else:
        print("Launching leg odometry")
        return [*nodes, odom_fusion]



def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulated clock.'),

        DeclareLaunchArgument(
            'frame_id', default_value='base_link',
            description='Base frame of the robot.'),

        DeclareLaunchArgument(
            'fixed_frame_id', default_value='',
            description='Fixed frame used for lidar deskewing. If not set, we will generate one from IMU or external_odom_frame_id if not null.'),

        DeclareLaunchArgument(
            'external_odom_frame_id', default_value='',
            description='Provide external odometry with TF, disabling icp_odometry.'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Localization mode.'),

        DeclareLaunchArgument(
            'lidar_topic', default_value='/lidar_data_points',
            description='Name of the lidar PointCloud2 topic.'),

        DeclareLaunchArgument(
            'imu_topic', default_value='/imu',
            description='Name of an IMU topic.'),

        DeclareLaunchArgument(
            'rgbd_image_topic', default_value='',
            description='RGBD image topic (ignored if empty). Would be the output of a rtabmap_sync\'s rgbd_sync, stereo_sync or rgb_sync node.'),

        DeclareLaunchArgument(
            'rgbd_images_topic', default_value='',
            description='RGBD images topic (ignored if empty, override "rgbd_image_topic" if set). Would be the output of a rtabmap_sync\'s rgbdx_sync node.'),

        DeclareLaunchArgument(
            'voxel_size', default_value='0.05',
            description='Voxel size (m) of the downsampled lidar point cloud. For indoor, set it between 0.1 and 0.3. For outdoor, set it to 0.5 or over.'),

        DeclareLaunchArgument(
            'min_loop_closure_overlap', default_value='0.2',
            description='Minimum scan overlap pourcentage to accept a loop closure.'),

        DeclareLaunchArgument(
            'expected_update_rate', default_value='15.0',
            description='Expected lidar frame rate. Ideally, set it slightly higher than actual frame rate, like 15 Hz for 10 Hz lidar scans.'),

        DeclareLaunchArgument(
            'assembling_time', default_value='1.0',
            description='How much time (sec) we assemble lidar scans before sending them to mapping node.'),

        DeclareLaunchArgument(
            'rtabmap_ini', default_value='',
            description='Path to the rtabmap ini file, configuring its nodes'
        ),

        DeclareLaunchArgument(
            'use_rtabmap_viz', default_value='False',
            description='Use RTABMap\'s vizualization tool'),

        DeclareLaunchArgument(
            'rtab_db', default_value='',
            description='Name of the database to save results of slam, or used for localization'),

        DeclareLaunchArgument(
            'fuse_odoms', default_value='False',
            description='Launch multiple odometry nodes'),

        DeclareLaunchArgument(
            'qos', default_value='1',
            description='Quality of Service: 0=system default, 1=reliable, 2=best effort.'),

        OpaqueFunction(function=launch_setup),
    ])
