#!/usr/bin/env python3
"""
Utilities for portable ROS2 launch files.
Useful for finding package paths across different systems.
"""

import os
import sys
from pathlib import Path

def find_package_source_directory(package_name: str = None) -> Path:
    """
    Find the source directory of a ROS2 package.
    
    This function tries multiple methods to find the package source directory
    in a portable way that works across different computers and setups.
    
    Args:
        package_name: Name of the package. If None, tries to auto-detect.
    
    Returns:
        Path object to the package source directory.
    
    Raises:
        FileNotFoundError: If package directory cannot be found.
    """
    # Method 1: From launch file location (most reliable)
    try:
        # Get the directory of the calling launch file
        frame = sys._getframe(1)  # Get caller's frame
        launch_file_path = frame.f_globals.get('__file__', '')
        if launch_file_path:
            launch_dir = Path(launch_file_path).parent
            
            # If launch file is in standard location: src/<package>/launch/
            if launch_dir.name == 'launch' and launch_dir.parent.name:
                potential_pkg_dir = launch_dir.parent
                if (potential_pkg_dir / 'package.xml').exists():
                    return potential_pkg_dir
    except (AttributeError, IndexError):
        pass
    
    # Method 2: From ROS2 environment variables
    env_methods = [
        # Check if we're running from colcon workspace
        lambda: os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0],
        # Check AMENT prefix path
        lambda: os.environ.get('AMENT_PREFIX_PATH', '').split(':')[0],
        # Check ROS workspace
        lambda: os.environ.get('ROS_WORKSPACE', ''),
    ]
    
    for get_path in env_methods:
        install_path = get_path()
        if install_path and os.path.exists(install_path):
            install_path = Path(install_path)
            # Navigate from install to src
            # install_path is like: /path/to/workspace/install/<package> or /path/to/workspace/install
            if install_path.name == 'install':
                workspace_root = install_path.parent
            else:
                # install_path might be /path/to/workspace/install/<package>
                workspace_root = install_path.parent.parent
            
            src_dir = workspace_root / 'src'
            if src_dir.exists():
                # If package_name is provided, look for that specific package
                if package_name:
                    pkg_dir = src_dir / package_name
                    if pkg_dir.exists():
                        return pkg_dir
                # Otherwise, try to find any package with package.xml
                for item in src_dir.iterdir():
                    if item.is_dir() and (item / 'package.xml').exists():
                        # If this looks like our package (contains controllers/, worlds/)
                        if (item / 'controllers').exists() and (item / 'worlds').exists():
                            return item
    
    # Method 3: Search from current directory upwards
    current_dir = Path.cwd()
    while current_dir != current_dir.root:
        # Look for package.xml
        package_xml = current_dir / 'package.xml'
        if package_xml.exists():
            # Check if this looks like our self-driving car package
            if (current_dir / 'controllers').exists() and (current_dir / 'worlds').exists():
                return current_dir
        
        # If package_name is provided, check directory name
        if package_name and current_dir.name == package_name:
            if (current_dir / 'package.xml').exists():
                return current_dir
        
        current_dir = current_dir.parent
    
    # Method 4: Try to import the package and find its location
    if package_name:
        try:
            import importlib.util
            spec = importlib.util.find_spec(package_name)
            if spec and spec.origin:
                pkg_path = Path(spec.origin).parent
                # Navigate to source directory
                # Usually spec.origin points to __init__.py in src/<package>/<package>/
                # We need to go up one level
                if pkg_path.parent.name == package_name:
                    return pkg_path.parent.parent / package_name
        except ImportError:
            pass
    
    # Method 5: Common workspace locations
    home = Path.home()
    common_workspaces = [
        home / 'ros2_ws' / 'src' / 'self_driving_car',
        home / 'ros_ws' / 'src' / 'self_driving_car',
        home / 'workspace' / 'src' / 'self_driving_car',
        home / 'catkin_ws' / 'src' / 'self_driving_car',
        home / 'dev_ws' / 'src' / 'self_driving_car',
        Path.cwd() / 'src' / 'self_driving_car',
        Path.cwd().parent / 'src' / 'self_driving_car',
    ]
    
    for workspace in common_workspaces:
        if workspace.exists() and (workspace / 'package.xml').exists():
            return workspace
    
    # If all methods fail, raise an error
    raise FileNotFoundError(
        f"Could not find package source directory for '{package_name or 'self_driving_car'}'. "
        "Please ensure you're in a ROS2 workspace and have sourced setup.bash"
    )

def find_world_file(package_dir: Path, world_name: str = 'reference.wbt') -> Path:
    """
    Find a world file within the package.
    
    Args:
        package_dir: Path to package source directory
        world_name: Name of world file (e.g., 'reference.wbt')
    
    Returns:
        Path to world file
    """
    # Try standard locations
    locations = [
        package_dir / 'worlds' / world_name,
        package_dir / 'worlds' / 'worlds' / world_name,  # Some projects nest worlds
        package_dir / 'resources' / 'worlds' / world_name,
        package_dir / 'share' / package_dir.name / 'worlds' / world_name,  # Install location
    ]
    
    for location in locations:
        if location.exists():
            return location
    
    # If not found, list available worlds
    worlds_dir = package_dir / 'worlds'
    if worlds_dir.exists():
        available = [f.name for f in worlds_dir.glob('*.wbt')]
        raise FileNotFoundError(
            f"World file '{world_name}' not found. Available worlds: {available}"
        )
    
    raise FileNotFoundError(f"No worlds directory found in {package_dir}")

def setup_webots_environment(package_dir: Path) -> dict:
    """
    Setup environment variables for Webots.
    
    Args:
        package_dir: Path to package source directory
    
    Returns:
        Dictionary of environment variables
    """
    env = os.environ.copy()
    
    # Set Webots project path
    env['WEBOTS_PROJECT_PATH'] = str(package_dir)
    
    # Try to find Webots installation
    webots_paths = [
        '/usr/local/webots',
        '/opt/webots',
        Path.home() / 'webots',
        Path.home() / 'snap' / 'webots' / 'current',
        '/snap/webots/current',
    ]
    
    for webots_path in webots_paths:
        if Path(webots_path).exists():
            env['WEBOTS_HOME'] = str(webots_path)
            # Add to library path for controllers
            lib_path = Path(webots_path) / 'lib' / 'controller'
            if lib_path.exists():
                env['LD_LIBRARY_PATH'] = f"{lib_path}:{env.get('LD_LIBRARY_PATH', '')}"
            break
    else:
        # Webots not found in standard locations
        # User might have it in PATH already
        pass
    
    # Add package directory to Python path for controllers
    python_path = env.get('PYTHONPATH', '')
    env['PYTHONPATH'] = f"{package_dir}:{python_path}"
    
    return env

def create_webots_symlinks(package_dir: Path) -> None:
    """
    Create Webots-compatible symlink structure.
    
    Webots expects controllers in specific directories like:
    - projects/default/controllers/
    - projects/vehicles/controllers/
    
    Args:
        package_dir: Path to package source directory
    """
    import subprocess
    
    controllers_dir = package_dir / 'controllers'
    if not controllers_dir.exists():
        return
    
    # Create directory structure
    project_dirs = [
        package_dir / 'projects' / 'default' / 'controllers',
        package_dir / 'projects' / 'vehicles' / 'controllers',
    ]
    
    for proj_dir in project_dirs:
        proj_dir.mkdir(parents=True, exist_ok=True)
    
    # Create symlinks for each controller
    for controller in controllers_dir.iterdir():
        if controller.is_dir():
            controller_name = controller.name
            for proj_dir in project_dirs:
                target = proj_dir / controller_name
                if not target.exists():
                    # Use relative symlink
                    rel_path = os.path.relpath(controller, proj_dir)
                    target.symlink_to(rel_path, target_is_directory=True)
