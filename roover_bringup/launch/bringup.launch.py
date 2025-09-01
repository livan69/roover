# ~/ros2_ws/src/roover_bringup/launch/bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 1) Joystick-invoer
        Node(
            package='roover_f710_input',
            executable='f710_node',
            name='f710_node',
        ),

        # 2) Teleop → cmd_vel
        Node(
           package='roover_teleop',
           executable='teleop_node',
           name='teleop_node',
        ),

        # 3) Serial command → UART
        Node(
            package='roover_serial_cpp',
            executable='serial_command_node',
            name='serial_command_node',
            parameters=[
                {'device': '/dev/ttyAMA0'},
                {'baudrate': 115200},
            ]
        ),

        # 4) Serial feedback ← UART
        Node(
            package='roover_serial_cpp',
            executable='serial_feedback_node',
            name='serial_feedback_node',
            parameters=[
                {'device': '/dev/ttyAMA0'},
                {'baudrate': 115200},
            ]
        ),
        # 5) Odometry node
        Node(
            package='roover_odometry',
            executable='roover_odom_node',
            name='roover_odom_node',
            parameters=[
                {'wheel_base': 0.54},          # afstand tussen de wielen in meter
                {'ticks_per_rev': 90},
                {'wheel_diameter': 0.165},
            ]
        ),
        # 6) GPS node
        Node(
            package='roover_gps',
            executable='gps_node',
            name='gps_node',
            output='screen',
            parameters=[{
                'device': '/dev/ttyUSB0',
                'baudrate': 115200
            }]
        ),
        
        # 7) localization node
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['/home/livan/ros2_ws/src/roover_bringup/config/ekf.yaml'],
            output='screen'
        ),
        # 8) transfom localization node
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=['/home/livan/ros2_ws/src/roover_bringup/config/navsat.yaml'],
            output='screen'
        ),
        # 9) Map server (luchtfoto als achtergrond)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': '/home/livan/ros2_ws/src/roover_bringup/config/mymap.yaml'
            }]
        ),
        # 10) Lifecycle manager to auto-start map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server']
            }]
        )
    ])
