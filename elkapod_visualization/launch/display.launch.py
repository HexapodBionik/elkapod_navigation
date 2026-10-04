import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vis_package = FindPackageShare(package="elkapod_visualization").find(
        "elkapod_visualization"
    )

    default_rviz_config_path = os.path.join(vis_package, "config/config.rviz")


    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", default_rviz_config_path],
            parameters=[{'use_sim_time': True},],
            additional_env={'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so'}
        )
    )

    return ld

