#!/usr/bin/env python3
"""
Webots ROS2 Jazzy Launch File
Improved version using modern ROS2 launch conventions.
"""

import os
import pathlib
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
    """Locate reference.wbt in typical project locations."""
    search_paths = [
        os.path.join(package_dir, "worlds", "velodyne.wbt"),
        os.path.join(os.path.dirname(package_dir), "..", "src",
                     "simple", "worlds", "velodyne.wbt"),
        os.path.join(os.path.expanduser("~"), "source_code",
                     "self-driving-car-ROS2", "ros2_ws", "src",
                     "simple", "worlds", "velodyne.wbt"),
    ]

    for p in search_paths:
        if os.path.exists(p):
            return p

    return None


def find_project_path(package_dir):
    """Determine the correct WEBOTS_PROJECT_PATH."""
    src_dir = os.path.join(
        os.path.dirname(package_dir), "..", "src", "simple"
    )

    controllers_src = os.path.join(src_dir, "controllers")
    if os.path.exists(controllers_src):
        return src_dir

    return package_dir


def generate_launch_description():

    pkg_dir = get_package_share_directory("simple")
    world_file = find_world_file(pkg_dir)

    if not world_file:
        return LaunchDescription([
            LogInfo(msg="[velodyne.py] ERROR: velodyne.wbt not found!")
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
        msg=f"[velodyne.py] WEBOTS_PROJECT_PATH = {project_dir}"
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
    
    # ROS 2 Nodes
    ros_nodes = [  
    
    ]

    # Launch arguments
    launch_args = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Show Webots GUI."
        ),
    ]

    return LaunchDescription([
       *launch_args,
       set_webots_env,
       log_env,
       webots_process,
       *ros_nodes
    ])


