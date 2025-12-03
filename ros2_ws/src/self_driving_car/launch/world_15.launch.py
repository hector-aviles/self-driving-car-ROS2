from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('max_speed', default_value='20'),
        DeclareLaunchArgument('k_following', default_value='10'),
        DeclareLaunchArgument('dist_to_car', default_value='20'),
        DeclareLaunchArgument('goal_dist', default_value='200'),
        DeclareLaunchArgument('no_gui', default_value='false'),
        
        SetEnvironmentVariable('WEBOTS_HOME', '/usr/local/webots'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('webots_ros2_driver'),
                    'launch',
                    'webots_launch.py'
                ])
            ]),
            launch_arguments={
                'mode': 'realtime',
                'no_gui': LaunchConfiguration('no_gui'),
                'world': PathJoinSubstitution([
                    FindPackageShare('self_driving_car'),
                    'worlds',
                    'world_15.wbt'
                ])
            }.items()
        ),
        
        Node(
            package='self_driving_car',
            executable='lane_detector',
            name='lane_detector',
            output='screen'
        ),
        
        Node(
            package='self_driving_car',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
        
        Node(
            package='self_driving_car',
            executable='behaviors',
            name='behaviors',
            output='screen',
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed'),
                'k_following': LaunchConfiguration('k_following'),
                'dist_to_car': LaunchConfiguration('dist_to_car')
            }]
        ),
        
        Node(
            package='self_driving_car',
            executable='success',
            name='success',
            output='screen'
        ),
        
        Node(
            package='self_driving_car',
            executable='stop',
            name='stop',
            output='screen'
        ),
        
        Node(
            package='self_driving_car',
            executable='lane_identification',
            name='lane_identification',
            output='screen'
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='car_to_lidar',
            arguments=['0', '0', '2.0', '0', '0', '0', 'car_link', 'lidar_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='car_to_camera',
            arguments=['1.2', '0', '1.5', '0', '0', '0', 'car_link', 'camera_link']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='car_to_gyro',
            arguments=['0', '0', '0.1', '0', '0', '0', 'car_link', 'gyro_link']
        ),
        
        Node(
            package='self_d