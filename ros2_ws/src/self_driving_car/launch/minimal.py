import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(__file__))
    world_path = os.path.join(pkg_dir, 'worlds', 'reference.wbt')
    
    return LaunchDescription([
        LogInfo(msg=f"Launching from: {pkg_dir}"),
        
        ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'cd "{pkg_dir}" && '
                f'export WEBOTS_PROJECT_PATH="{pkg_dir}" && '
                f'export WEBOTS_HOME="/usr/local/webots" && '
                f'echo "Environment set, launching Webots..." && '
                f'webots --mode=realtime worlds/reference.wbt'
            ],
            output='screen',
            shell=True
        )
    ])
