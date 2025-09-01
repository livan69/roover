from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('roover_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_config],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gps_tf_pub',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'gps_link']
        )
    ])
