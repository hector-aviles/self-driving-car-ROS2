#!/usr/bin/env python3
"""
Webots ROS2 Jazzy Launch File
Improved version using modern ROS2 launch conventions.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    LogInfo
)
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def find_world_file(package_dir):
    """Locate head_on_1.wbt in typical project locations."""
    search_paths = [
        os.path.join(package_dir, "worlds", "head_on_1.wbt"),
        os.path.join(os.path.dirname(package_dir), "..", "src",
                     "self_driving_car", "worlds", "head_on_1.wbt"),
        os.path.join(os.path.expanduser("~"), "source_code",
                     "self-driving-car-ROS2", "ros2_ws", "src",
                     "self_driving_car", "worlds", "head_on_1.wbt"),
    ]

    for p in search_paths:
        if os.path.exists(p):
            return p

    return None


def find_project_path(package_dir):
    """Determine the correct WEBOTS_PROJECT_PATH."""
    src_dir = os.path.join(
        os.path.dirname(package_dir), "..", "src", "self_driving_car"
    )

    controllers_src = os.path.join(src_dir, "controllers")
    if os.path.exists(controllers_src):
        return src_dir

    return package_dir


def generate_launch_description():

    pkg_dir = get_package_share_directory("self_driving_car")
    world_file = find_world_file(pkg_dir)

    if not world_file:
        return LaunchDescription([
            LogInfo(msg="[reference.py] ERROR: head_on_1.wbt not found!")
        ])

    project_dir = find_project_path(pkg_dir)
     
    set_webots_env = SetEnvironmentVariable(
       name="WEBOTS_PROJECT_PATH",
       value=project_dir
    )    

    # Environment variable for Webots
    set_webots_env = SetEnvironmentVariable(
        name="WEBOTS_PROJECT_PATH",
        value=project_dir
    )

    log_env = LogInfo(
        msg=f"[reference.py] WEBOTS_PROJECT_PATH = {project_dir}"
    )

    # Launch Webots itself
    webots_process = ExecuteProcess(
        cmd=[
            "webots",
            "--mode=realtime",
            "--stdout",
            "--stderr",
            world_file
        ],
        cwd=project_dir,
        output="screen"
    )

    # TF static transforms
    Node(
       package="tf2_ros",
       executable="static_transform_publisher",
       name="tf_car",
       arguments=[
          "0", "0", "0",   # x, y, z
          "0", "0", "0",   # roll, pitch, yaw
          "map", "car_link"
       ]
    )    
        
    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_camera",
        output="screen",
        arguments=[
            "1.2", "0.0", "1.5",
            "0", "0", "0",
            "car_link", "camera_link"
        ]
    )

    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_lidar",
        output="screen",
        arguments=[
            "0.0", "0.0", "2.0",
            "-0.2618", "0", "0",
            "car_link", "lidar_link"
        ]
    )

    static_tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu",
        output="screen",
        arguments=[
            "0", "0", "0.1",
            "0", "0", "0",
            "car_link", "imu_link"
        ]
    )
    
    robot_state_pub = Node(
       package="robot_state_publisher",
       executable="robot_state_publisher",
       parameters=[{
             "robot_description": open(
             os.path.join(project_dir, "urdf", "bmw.urdf")).read()
       }]
    )      
    
    # ROS 2 Nodes
    ros_nodes = [  
    
        Node(package="self_driving_car", executable="lane_detector_canny_hough",
             name="lane_detector", output="screen",
             parameters=[{"use_sim_time": True}]),

        Node(package="self_driving_car", executable="obstacle_detector",
             name="obstacle_detector", output="screen",
             parameters=[{"use_sim_time": True}]),

        Node(package="self_driving_car", executable="behaviors",
             name="behaviors", output="screen",
             parameters=[{"use_sim_time": True}]),

        Node(package="self_driving_car", executable="lane_identification",
             name="lane_identification", output="screen",
             parameters=[{"use_sim_time": True}]),

        Node(package="self_driving_car", executable="stop",
             name="stop", output="screen",
             parameters=[{"use_sim_time": True}]),

        Node(package="self_driving_car", executable="success",
             name="success", output="screen",
             parameters=[{"use_sim_time": True}]),
    ]

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Show Webots GUI."
        ),
        DeclareLaunchArgument(
            "max_speed",
            default_value="20.0",
            description="Max speed for controller."
        ),
    ]

    return LaunchDescription([
       *launch_args,
       set_webots_env,
       log_env,
       webots_process,
       static_tf_camera,
       static_tf_lidar,
       static_tf_imu,
       robot_state_pub,
       *ros_nodes
    ])


