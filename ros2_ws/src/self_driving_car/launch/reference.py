#!/usr/bin/env python3
"""
Simple Webots launch file - no external dependencies.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Simple launch without complex utilities."""
    
    # Get package share directory (install location)
    package_dir = get_package_share_directory('self_driving_car')
    
    print(f"\n=== Found package at: {package_dir} ===")
    
    # Find world file
    world_path = None
    possible_locations = [
        os.path.join(package_dir, 'worlds', 'reference.wbt'),
        os.path.join(os.path.dirname(package_dir), '..', 'src', 'self_driving_car', 'worlds', 'reference.wbt'),
        os.path.join(os.path.expanduser('~'), 'source_code', 'self-driving-car-ROS2', 'ros2_ws', 
                     'src', 'self_driving_car', 'worlds', 'reference.wbt'),
    ]
    
    for path in possible_locations:
        if os.path.exists(path):
            world_path = path
            print(f"Found world file: {world_path}")
            break
    
    if not world_path:
        return LaunchDescription([
            LogInfo(msg="ERROR: World file not found!"),
            LogInfo(msg="Tried:"),
            LogInfo(msg=f"  {possible_locations[0]}"),
            LogInfo(msg=f"  {possible_locations[1]}"),
        ])
    
    # Setup Webots environment
    env = os.environ.copy()
    
    # CRITICAL: Find where controllers actually are
    # Look for controllers in source directory
    source_dir = os.path.join(os.path.dirname(package_dir), '..', 'src', 'self_driving_car')
    
    if os.path.exists(os.path.join(source_dir, 'controllers')):
        # Use source directory for WEBOTS_PROJECT_PATH
        env['WEBOTS_PROJECT_PATH'] = source_dir
        working_dir = source_dir
        print(f"Using SOURCE directory: {source_dir}")
    else:
        # Fallback to install directory
        env['WEBOTS_PROJECT_PATH'] = package_dir
        working_dir = package_dir
        print(f"Using INSTALL directory: {package_dir}")
    
    print(f"Working directory: {working_dir}")
    print(f"WEBOTS_PROJECT_PATH: {env['WEBOTS_PROJECT_PATH']}\n")
    
    # Launch Webots
    webots_process = ExecuteProcess(
        cmd=[
            'webots',
            '--mode=realtime',
            '--stdout',
            '--stderr',
            world_path
        ],
        output='screen',
        cwd=working_dir,  # This is KEY - sets current directory
        env=env,
        name='webots_simulation'
    )
    
    # ROS2 nodes (simplified version)
    nodes = [
        Node(
            package='self_driving_car',
            executable='lane_detector_canny_hough',
            name='lane_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='self_driving_car',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Add other nodes as needed...
    ]
    
    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            'max_speed',
            default_value='20.0',
            description='Maximum speed (km/h)'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Show Webots GUI'
        ),
    ]
    
    return LaunchDescription(
        launch_args +
        [webots_process] +
        nodes
    )
