from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Argumenten zodat je ze via de commandline kan aanpassen
        DeclareLaunchArgument(
            'device', default_value='/dev/ttyUSB0',
            description='Seriële poort van de GPS module'
        ),
        DeclareLaunchArgument(
            'baudrate', default_value='115200',
            description='Baudrate van de GPS module'
        ),

        # GPS node
        Node(
            package='roover_gps',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
            }]
        )
    ])
