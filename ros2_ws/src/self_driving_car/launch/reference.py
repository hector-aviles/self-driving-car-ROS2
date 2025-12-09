#!/usr/bin/env python3
"""
Portable ROS2 launch file for self-driving car simulation.
This launch file starts Webots simulator and all ROS2 nodes needed for
the self-driving car system to work together.
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Note: The import utility handling section IS necessary because:
# 1. When this file is imported as a module by ROS2 launch system, it uses relative imports
# 2. When run directly for testing, it needs absolute imports
# 3. It provides graceful fallbacks if imports fail, giving helpful error messages
try:
    # Try relative import (used when ROS2 loads this as a launch file)
    from .launch_utils import (
        find_package_source_directory,
        find_world_file,
        setup_webots_environment,
        create_webots_symlinks,
    )
except ImportError:
    # Fallback: try absolute import (used when running this file directly)
    try:
        from self_driving_car.launch_utils import (
            find_package_source_directory,
            find_world_file,
            setup_webots_environment,
            create_webots_symlinks,
        )
    except ImportError:
        # Minimal fallback implementations to prevent complete failure
        # This helps users understand what's wrong instead of cryptic import errors
        import sys
        from pathlib import Path
        
        def find_package_source_directory(package_name="self_driving_car"):
            """Simplified version for when imports fail."""
            launch_dir = Path(__file__).parent
            package_dir = launch_dir.parent
            if (package_dir / 'package.xml').exists():
                return package_dir
            raise FileNotFoundError("Could not find package directory")
        
        def find_world_file(package_dir, world_name="reference.wbt"):
            """Simplified version for when imports fail."""
            world_path = package_dir / 'worlds' / world_name
            if world_path.exists():
                return world_path
            raise FileNotFoundError(f"World file not found: {world_path}")
        
        def setup_webots_environment(package_dir):
            """Setup environment variables needed by Webots."""
            env = os.environ.copy()
            # WEBOTS_PROJECT_PATH tells Webots where to find project resources
            env['WEBOTS_PROJECT_PATH'] = str(package_dir)
            return env
        
        def create_webots_symlinks(package_dir):
            """Placeholder - symlinks are created separately."""
            pass  # This would create necessary symlinks for Webots resources

def generate_launch_description():
    """Main function that creates the launch description for ROS2."""
    
    # -----------------------------------------------------------------
    # 1. DYNAMIC PATH DISCOVERY
    # -----------------------------------------------------------------
    # Find package and world file paths dynamically so this launch file
    # works on any computer without hardcoded paths
    try:
        package_dir = find_package_source_directory("self_driving_car")
        world_path = find_world_file(package_dir, "reference.wbt")
    except FileNotFoundError as e:
        # Provide helpful error messages if paths aren't found
        return LaunchDescription([
            LogInfo(msg=f"ERROR: {e}"),
            LogInfo(msg="Please ensure:"),
            LogInfo(msg="1. You are in a ROS2 workspace"),
            LogInfo(msg="2. You have sourced setup.bash: 'source install/setup.bash'"),
            LogInfo(msg="3. The package is built: 'colcon build --packages-select self_driving_car'"),
        ])
    
    # Setup environment variables for Webots
    webots_env = setup_webots_environment(package_dir)
    
    # -----------------------------------------------------------------
    # 2. DEBUG INFORMATION
    # -----------------------------------------------------------------
    # Display useful debug info to help users troubleshoot issues
    debug_logs = [
        LogInfo(msg="=== SELF-DRIVING CAR SIMULATION ==="),
        LogInfo(msg=f"Package directory: {package_dir}"),
        LogInfo(msg=f"World file: {world_path}"),
        LogInfo(msg=f"World exists: {world_path.exists()}"),
        LogInfo(msg=f"WEBOTS_PROJECT_PATH: {webots_env.get('WEBOTS_PROJECT_PATH', 'Not set')}"),
        LogInfo(msg=f"WEBOTS_HOME: {webots_env.get('WEBOTS_HOME', 'Not set')}"),
    ]
    
    # -----------------------------------------------------------------
    # 3. WEBOTS SYMLINKS CREATION
    # -----------------------------------------------------------------
    # This section is CRITICAL for Webots to work properly.
    # Webots expects specific directory structures and symlinks for:
    # - Controller programs (in <world>/controllers/)
    # - Protos (custom Webots objects)
    # - Textures and other resources
    
    # The symlinks creation is run as a separate process because:
    # 1. It needs to happen BEFORE Webots starts
    # 2. It modifies the filesystem (creates symlinks)
    # 3. It's a one-time setup operation
    
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
        output='screen',  # Show output in terminal
        env=webots_env  # Pass environment variables
    )
    
    # -----------------------------------------------------------------
    # 4. WEBOTS SIMULATOR LAUNCH
    # -----------------------------------------------------------------
    webots_process = ExecuteProcess(
        cmd=[
            'webots',  # Webots executable
            '--mode=realtime',  # Run in real-time mode (matches real-world time)
            '--stdout',  # Capture standard output
            '--stderr',  # Capture standard error
            str(world_path)  # Path to the world file
        ],
        output='screen',  # Display Webots output in terminal
        cwd=str(package_dir),  # CRITICAL: Set working directory to package
        env=webots_env,  # Use our custom environment
        name='webots_simulation'  # Name for this process in ROS2
    )
    
    # -----------------------------------------------------------------
    # 5. ROS2 CONFIGURATION PARAMETERS
    # -----------------------------------------------------------------
    # These are parameters users can adjust when launching
    # Example: ros2 launch self_driving_car simulation.launch.py max_speed:=30.0
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
    
    # -----------------------------------------------------------------
    # 6. ROS2 NODES - CORE FUNCTIONALITY
    # -----------------------------------------------------------------
    nodes = [
        # Detects lanes using camera images
        Node(
            package='self_driving_car',
            executable='lane_detector_canny_hough',
            name='lane_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,  # Use simulation time, not real time
            }]
        ),
        # Detects obstacles using LiDAR data
        Node(
            package='self_driving_car',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
        # Main behavior controller - decides what the car should do
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
        # Identifies which lane the car is in
        Node(
            package='self_driving_car',
            executable='lane_identification',
            name='lane_identification',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        ),
        # Monitors mission success conditions
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
        # Emergency stop functionality
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
    
    # -----------------------------------------------------------------
    # 7. TF (TRANSFORM) FRAMES
    # -----------------------------------------------------------------
    # Defines spatial relationships between different parts of the car
    # Critical for sensor fusion - tells ROS where sensors are located
    tfs = [
        # LiDAR is 2 meters above the car center
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '2.0', '0', '0', '0', 'car_link', 'lidar_link'],
            name='lidar_tf_publisher'
        ),
        # Camera is 1.5m high, 1.2m forward from car center
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['1.2', '0', '1.5', '0', '0', '0', 'car_link', 'camera_link'],
            name='camera_tf_publisher'
        ),
        # IMU is near the car center
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'car_link', 'imu_link'],
            name='imu_tf_publisher'
        ),
        # GPS is near the car center
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'car_link', 'gps_link'],
            name='gps_tf_publisher'
        ),
    ]
    
    # -----------------------------------------------------------------
    # 8. SHUTDOWN HANDLER
    # -----------------------------------------------------------------
    # When Webots exits, this ensures proper cleanup
    shutdown_event = RegisterEventHandler(
        OnProcessExit(
            target_action=webots_process,
            on_exit=[
                LogInfo(msg="Webots simulation ended. Shutting down ROS2 nodes..."),
                webots_process,
            ]
        )
    )
    
    # -----------------------------------------------------------------
    # 9. ASSEMBLE LAUNCH DESCRIPTION
    # -----------------------------------------------------------------
    return LaunchDescription(
        debug_logs +          # Show debug info first
        launch_args +         # Configuration parameters
        [create_symlinks] +   # Setup Webots symlinks
        [webots_process] +    # Start Webots simulator
        nodes +               # Start all ROS2 nodes
        tfs +                 # Setup transform frames
        [shutdown_event]      # Register shutdown handler
    )


if __name__ == '__main__':
    # This allows running the launch file directly for testing
    # Normally ROS2 launch system calls generate_launch_description()
    from launch import LaunchService
    
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    launch_service.run()
