#!/usr/bin/env python3
"""
Portable ROS2 launch file for self-driving car simulation.
This works across different computers and setups.
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Import our utilities
try:
    from .launch_utils import (
        find_package_source_directory,
        find_world_file,
        setup_webots_environment,
        create_webots_symlinks,
    )
except ImportError:
    # Fallback: try absolute import
    try:
        from self_driving_car.launch_utils import (
            find_package_source_directory,
            find_world_file,
            setup_webots_environment,
            create_webots_symlinks,
        )
    except ImportError:
        # Create minimal versions for import issues
        import sys
        from pathlib import Path
        
        def find_package_source_directory(package_name="self_driving_car"):
            """Fallback implementation."""
            launch_dir = Path(__file__).parent
            package_dir = launch_dir.parent
            if (package_dir / 'package.xml').exists():
                return package_dir
            raise FileNotFoundError("Could not find package directory")
        
        def find_world_file(package_dir, world_name="reference.wbt"):
            """Fallback implementation."""
            world_path = package_dir / 'worlds' / world_name
            if world_path.exists():
                return world_path
            raise FileNotFoundError(f"World file not found: {world_path}")
        
        def setup_webots_environment(package_dir):
            """Fallback implementation."""
            env = os.environ.copy()
            env['WEBOTS_PROJECT_PATH'] = str(package_dir)
            return env
        
        def create_webots_symlinks(package_dir):
            """Fallback implementation."""
            pass  # Skip for now

def generate_launch_description():
    """Generate launch description for portable simulation."""
    
    # -------------------------------
    # Find paths dynamically
    # -------------------------------
    try:
        package_dir = find_package_source_directory("self_driving_car")
        world_path = find_world_file(package_dir, "reference.wbt")
    except FileNotFoundError as e:
        # Return error message if paths not found
        return LaunchDescription([
            LogInfo(msg=f"ERROR: {e}"),
            LogInfo(msg="Please ensure:"),
            LogInfo(msg="1. You are in a ROS2 workspace"),
            LogInfo(msg="2. You have sourced setup.bash: 'source install/setup.bash'"),
            LogInfo(msg="3. The package is built: 'colcon build --packages-select self_driving_car'"),
        ])
    
    # Setup environment
    webots_env = setup_webots_environment(package_dir)
    
    # Debug information
    debug_logs = [
        LogInfo(msg="=== SELF-DRIVING CAR SIMULATION ==="),
        LogInfo(msg=f"Package directory: {package_dir}"),
        LogInfo(msg=f"World file: {world_path}"),
        LogInfo(msg(f"World exists: {world_path.exists()}"),
        LogInfo(msg=f"WEBOTS_PROJECT_PATH: {webots_env.get('WEBOTS_PROJECT_PATH', 'Not set')}"),
        LogInfo(msg=f"WEBOTS_HOME: {webots_env.get('WEBOTS_HOME', 'Not set')}"),
    ]
    
    # -------------------------------
    # Create Webots symlinks
    # -------------------------------
    create_symlinks = ExecuteProcess(
        cmd=[
            'python3', '-c',
            f'''
import sys
sys.path.insert(0, "{package_dir}")
from launch_utils import create_webots_symlinks
from pathlib import Path
create_webots_symlinks(Path("{package_dir}"))
print("Webots symlinks created successfully")
            '''
        ],
        output='screen',
        env=webots_env
    )
    
    # -------------------------------
    # Launch Webots
    # -------------------------------
    webots_process = ExecuteProcess(
        cmd=[
            'webots',
            '--mode=realtime',
            '--stdout',
            '--stderr',
            str(world_path)
        ],
        output='screen',
        # Critical: Set working directory to package directory
        cwd=str(package_dir),
        # Use our custom environment
        env=webots_env,
        name='webots_simulation'
    )
    
    # -------------------------------
    ROS2 Configuration
    # -------------------------------
    launch_args = [
        DeclareLaunchArgument(
            'max_speed',
            default_value='20.0',
            description='Maximum speed of the vehicle (km/h)'
        ),
        DeclareLaunchArgument(
            'k_following',
            default_value='10.0',
            description='Following distance coefficient'
        ),
        DeclareLaunchArgument(
            'dist_to_car',
            default_value='20.0',
            description='Minimum distance to maintain from other cars (m)'
        ),
        DeclareLaunchArgument(
            'goal_dist',
            default_value='200.0',
            description='Goal distance for mission completion (m)'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch Webots GUI (true/false)'
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='realtime',
            description='Webots simulation mode (realtime/fast)'
        ),
    ]
    
    # -------------------------------
    # ROS2 Nodes
    # -------------------------------
    nodes = [
        Node(
            package='self_driving_car',
            executable='lane_detector_canny_hough',
            name='lane_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
        Node(
            package='self_driving_car',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
        Node(
            package='self_driving_car',
            executable='behaviors',
            name='behavior_controller',
            output='screen',
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed'),
                'k_following': LaunchConfiguration('k_following'),
                'dist_to_car': LaunchConfiguration('dist_to_car'),
                'use_sim_time': True,
            }]
        ),
        Node(
            package='self_driving_car',
            executable='lane_identification',
            name='lane_identification',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
        Node(
            package='self_driving_car',
            executable='success',
            name='success_monitor',
            output='screen',
            parameters=[{
                'goal_distance': LaunchConfiguration('goal_dist'),
                'use_sim_time': True,
            }]
        ),
        Node(
            package='self_driving_car',
            executable='stop',
            name='emergency_stop',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
    ]
    
    # -------------------------------
    # TF Static Transforms
    # -------------------------------
    tfs = [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '2.0', '0', '0', '0', 'car_link', 'lidar_link'],
            name='lidar_tf_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1.2', '0', '1.5', '0', '0', '0', 'car_link', 'camera_link'],
            name='camera_tf_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'car_link', 'imu_link'],
            name='imu_tf_publisher'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'car_link', 'gps_link'],
            name='gps_tf_publisher'
        ),
    ]
    
    # -------------------------------
    # Shutdown Handler
    # -------------------------------
    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=webots_process,
            on_exit=[
                LogInfo(msg="Webots simulation ended. Shutting down ROS2 nodes..."),
                webots_process,
            ]
        )
    )
    
    # -------------------------------
    # Return Launch Description
    # -------------------------------
    return LaunchDescription(
        debug_logs +
        launch_args +
        [create_symlinks] +
        [webots_process] +
        nodes +
        tfs +
        [shutdown_event]
    )


if __name__ == '__main__':
    # This allows running the launch file directly for testing
    from launch import LaunchService
    
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
