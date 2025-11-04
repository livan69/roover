from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roover_lidar',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'device': '/dev/lidar',
                'baudrate': 230400
            }],
            emulate_tty=True,
            respawn=False
        )
    ])