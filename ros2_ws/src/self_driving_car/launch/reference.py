import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    # ----------------------------------------------------------
    # Package paths
    # ----------------------------------------------------------
    pkg_dir = get_package_share_directory('self_driving_car')
    world_path = os.path.join(pkg_dir, 'worlds', 'reference.wbt')

    # ----------------------------------------------------------
    # Webots launcher
    # ----------------------------------------------------------
    webots = WebotsLauncher(
        world=world_path,
        mode='realtime'
    )

    # ----------------------------------------------------------
    # Webots driver for the car (robot name must match Webots)
    # ----------------------------------------------------------
    car_driver = WebotsController(
        robot_name='BMW_X5',           # <-- Change if your Webots robot name differs
        parameters=[]
    )

    # ----------------------------------------------------------
    # Argument definitions
    # ----------------------------------------------------------
    max_speed = DeclareLaunchArgument('max_speed', default_value='20')
    k_following = DeclareLaunchArgument('k_following', default_value='10.0')
    dist_to_car = DeclareLaunchArgument('dist_to_car', default_value='20')
    goal_dist = DeclareLaunchArgument('goal_dist', default_value='200')
    no_gui = DeclareLaunchArgument('no_gui', default_value='false')

    # ----------------------------------------------------------
    # Custom nodes (your perception + behavior stack)
    # ----------------------------------------------------------
    nodes = [
       Node(
           package='self_driving_car',
           executable='lane_detector_canny_hough',
           name='lane_detector_canny_hough',
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
  ]

    # ----------------------------------------------------------
    # Static TFs
    # ----------------------------------------------------------
    tfs = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '2.0', '0', '0', '0',
                       'car_link', 'lidar_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1.2', '0', '1.5', '0', '0', '0',
                       'car_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0',
                       'car_link', 'gyro_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0',
                       'car_link', 'accel_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0',
                       'car_link', 'gps_link']
        ),
    ]

    # ----------------------------------------------------------
    # Shutdown Webots when simulation exits
    # ----------------------------------------------------------
    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(
                event=launch.events.Shutdown())]
        )
    )

    # ----------------------------------------------------------
    # Environment variable for Webots
    # ----------------------------------------------------------
    webots_home = SetEnvironmentVariable('WEBOTS_HOME', '/usr/local/webots')

    # ----------------------------------------------------------
    # Launch description
    # ----------------------------------------------------------
    return LaunchDescription([
        # Arguments
        max_speed,
        k_following,
        dist_to_car,
        goal_dist,
        no_gui,

        # Environment
        webots_home,

        # Launch Webots
        webots,
        car_driver,

        # Custom nodes
        *nodes,

        # Static TFs
        *tfs,

        # Shutdown handler
        shutdown_event
    ])

