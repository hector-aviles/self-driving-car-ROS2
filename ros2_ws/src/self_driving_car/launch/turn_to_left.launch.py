from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    package_dir = FindPackageShare('self_driving_car').find('self_driving_car')
    world_path = os.path.join(package_dir, 'worlds', 'turn_to_left.wbt')

    return LaunchDescription([

        DeclareLaunchArgument('max_speed', default_value='20'),
        DeclareLaunchArgument('k_following', default_value='10'),
        DeclareLaunchArgument('dist_to_car', default_value='20'),
        DeclareLaunchArgument('goal_dist', default_value='200'),
        DeclareLaunchArgument('no_gui', default_value='false'),

        # Set WEBOTS_HOME 
        SetEnvironmentVariable('WEBOTS_HOME', '/usr/local/webots'),

        # Launch Webots directly (Jazzy-compatible)
        ExecuteProcess(
            cmd=[
                'webots',
                '--mode=realtime',
                world_path
            ],
            output='screen'
        ),

        # Your ROS2 nodes
        Node(package='self_driving_car', executable='lane_detector', output='screen'),
        Node(package='self_driving_car', executable='obstacle_detector', output='screen'),
        Node(
            package='self_driving_car',
            executable='behaviors',
            output='screen',
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed'),
                'k_following': LaunchConfiguration('k_following'),
                'dist_to_car': LaunchConfiguration('dist_to_car'),
            }]
        ),
        Node(package='self_driving_car', executable='success', output='screen'),
        Node(package='self_driving_car', executable='stop', output='screen'),
        Node(package='self_driving_car', executable='lane_identification', output='screen'),

        # TF static transforms
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','2.0','0','0','0','car_link','lidar_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['1.2','0','1.5','0','0','0','car_link','camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0.1','0','0','0','car_link','gyro_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0.1','0','0','0','car_link','accel_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0','0','0.1','0','0','0','car_link','gps_link']),
    ])

