import os
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess

def generate_launch_description():
    # Get the value
    webots_val = os.environ.get('WEBOTS_PROJECT_PATH')
    
    # Debug output
    actions = [
        LogInfo(msg="=== WEBOTS_PROJECT_PATH DEBUG ==="),
        LogInfo(msg=f"Type: {type(webots_val)}"),
        LogInfo(msg=f"Value (repr): {repr(webots_val)}"),
        LogInfo(msg=f"Value (str): {str(webots_val)}"),
        LogInfo(msg=f"Is None? {webots_val is None}"),
        LogInfo(msg=f"Is '<not set>'? {webots_val == '<not set>'}"),
    ]
    
    # If it's the literal string '<not set>'
    if webots_val == '<not set>':
        actions.append(LogInfo(msg="WARNING: Variable is literally set to string '<not set>'"))
        actions.append(LogInfo(msg="Fix with: export WEBOTS_PROJECT_PATH='/your/actual/path'"))
    
    # Shell verification
    actions.extend([
        ExecuteProcess(
            cmd=['bash', '-c', 'echo "--- Shell verification ---"'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['bash', '-c', 'echo "WEBOTS_PROJECT_PATH=$WEBOTS_PROJECT_PATH"'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['bash', '-c', 'if [ "$WEBOTS_PROJECT_PATH" = "<not set>" ]; then echo "It is literally <not set>"; fi'],
            output='screen'
        )
    ])
    
    return LaunchDescription(actions)
