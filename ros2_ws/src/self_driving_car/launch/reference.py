#!/usr/bin/env python3
"""
Webots ROS2 Jazzy Launch File
Includes internal launch_utils logic + all nodes + TF static transforms.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node


def find_world_file(package_dir):
    """Search for reference.wbt in several common locations."""

    search_paths = [
        os.path.join(package_dir, "worlds", "reference.wbt"),
        os.path.join(
            os.path.dirname(package_dir), "..", "src",
            "self_driving_car", "worlds", "reference.wbt"
        ),
        os.path.join(
            os.path.expanduser("~"),
            "source_code", "self-driving-car-ROS2", "ros2_ws",
            "src", "self_driving_car", "worlds", "reference.wbt"
        ),
    ]

    for path in search_paths:
        if os.path.exists(path):
            print(f"[reference.py] Found world: {path}")
            return path

    print("[reference.py] ERROR: world not found!")
    for p in search_paths:
        print(" Tried:", p)
    return None


def find_webots_project_path(package_dir):
    """
    Determine where controllers/ exists.
    Priority:
    1. Source directory
    2. Install directory
    """
    source_dir = os.path.join(os.path.dirname(package_dir), "..", "src", "self_driving_car")
    controllers_src = os.path.join(source_dir, "controllers")

    if os.path.exists(controllers_src):
        print(f"[reference.py] Using SOURCE controllers: {controllers_src}")
        return source_dir

    print(f"[reference.py] Using INSTALL controllers: {package_dir}")
    return package_dir


def generate_launch_description():

    package_dir = get_package_share_directory("self_driving_car")
    world_file = find_world_file(package_dir)

    if world_file is None:
        return LaunchDescription([
            LogInfo(msg="World file not found. Cannot start Webots."),
        ])

    # Determine WEBOTS_PROJECT_PATH and working directory
    project_dir = find_webots_project_path(package_dir)

    env = os.environ.copy()
    env["WEBOTS_PROJECT_PATH"] = project_dir
    print("[reference.py] WEBOTS_PROJECT_PATH =", project_dir)

    # Launch Webots
    webots = ExecuteProcess(
        cmd=[
            "webots",
            "--mode=realtime",
            "--stdout",
            "--stderr",
            world_file
        ],
        cwd=project_dir,
        env=env,
        output="screen",
        name="webots_simulation"
    )

    # -----------------------------
    # TF STATIC PUBLISHERS (important!)
    # -----------------------------

    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_camera",
        output="screen",
        arguments=[
            "0.2", "0.0", "1.0",    # x y z
            "0", "0", "0",          # roll pitch yaw
            "base_link", "camera_link"
        ]
    )

    static_tf_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_lidar",
        output="screen",
        arguments=[
            "0.1", "0.0", "1.2",
            "0", "0", "0",
            "base_link", "lidar_link"
        ]
    )

    static_tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_imu",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "base_link", "imu_link"
        ]
    )

    # -----------------------------
    # ROS2 NODES (all your executables)
    # -----------------------------

    nodes = [
        Node(
            package="self_driving_car",
            executable="lane_detector_canny_hough",
            name="lane_detector",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
        Node(
            package="self_driving_car",
            executable="obstacle_detector",
            name="obstacle_detector",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
        Node(
            package="self_driving_car",
            executable="behaviors",
            name="behaviors",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
        Node(
            package="self_driving_car",
            executable="lane_identification",
            name="lane_identification",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),      
        Node(
            package="self_driving_car",
            executable="stop",
            name="stop",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
        Node(
            package="self_driving_car",
            executable="success",
            name="success",
            output="screen",
            parameters=[{"use_sim_time": True}]
        )
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

    return LaunchDescription(
        launch_args +
        [webots] +
        [static_tf_camera, static_tf_lidar, static_tf_imu] +
        nodes
    )

