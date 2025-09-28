# ~/ros2_ws/src/roover_bringup/launch/bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # URDF inladen
    urdf_path = os.path.join(
        get_package_share_directory('roover_description'),
        'urdf',
        'roover.urdf.xacro'
    )
    robot_description = {'robot_description': xacro.process_file(urdf_path).toxml()}
    
    return LaunchDescription([
        # 0) Robot State Publisher (URDF → TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),
        
         # 1) Joystick-invoer
        Node(package='roover_f710_input', executable='f710_node', name='f710_node'),

        # 2) Teleop → cmd_vel
        Node(package='roover_teleop', executable='teleop_node', name='teleop_node'),


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
        
        # 7) GPS Path node
        Node(
            package='roover_gps',
            executable='gps_path_node',
            name='gps_path_node',
            output='screen'
        ),

        # 8) IMU node
        Node(package='roover_imu', executable='imu_node', name='imu_node', output='screen'),

        
        # 9) IMU filter (Madgwick) 
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            remappings=[('imu/data_raw', '/imu/data_raw'),('imu/data', '/imu/data')],
            parameters=[{'use_mag': False,'publish_tf': False}],
            output='screen'
        ),
        
        # 9b) [Fallback] Statische transform: base_link → imu_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_base_to_imu',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        # ),
        # [Fallback] Statische transform: base_link → gps_link
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf_base_to_gps',
        #     arguments=['0.25', '0', '0.095', '0', '0', '0', 'base_link', 'gps_link']
        # ),


        # 10) EKF node (fuseert odom + imu)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=['/home/livan/ros2_ws/src/roover_bringup/config/ekf.yaml'],
            output='screen'
        ),
        
        # 11) Navsat transform (GPS + odom + imu) 
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=['/home/livan/ros2_ws/src/roover_bringup/config/navsat.yaml'],
            remappings=[('imu', '/imu/data'),('gps/fix', '/fix'),('odometry/filtered', '/odometry/filtered')],
            output='screen'
        ),
        
        # 12) Map server (luchtfoto als achtergrond)
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': '/home/livan/ros2_ws/src/roover_bringup/config/mymap.yaml'}]
        ),
        
        # 13) Lifecycle manager to auto-start map_server
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
        ),
        
        # 14) Path logger node
        Node(
            package='roover_path',
            executable='path_logger_node',
            name='path_logger_node',
            output='screen'
        ),
    ])
