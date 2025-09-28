from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('roover_description'),
        'urdf',
        'roover_pi.urdf.xacro'
    )
    robot_description = {'robot_description': xacro.process_file(urdf_file).toxml()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
    ])
