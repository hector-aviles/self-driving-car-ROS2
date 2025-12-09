#!/usr/bin/env python3
"""
Utility functions for launching Webots simulation with ROS2.
Handles path discovery, symlink creation, and environment setup.
"""

import os
import sys
from pathlib import Path
import subprocess
import shutil

def find_package_source_directory(package_name="self_driving_car"):
    """
    Find the SOURCE directory of the package, not the install directory.
    
    Webots needs to access source files (controllers, protos, worlds) directly,
    not the installed copies in 'install/' directory.
    
    Returns:
        Path to package source directory
    """
    # Method 1: Check if we're running from source directory
    current_file = Path(__file__).resolve()
    launch_dir = current_file.parent
    
    # Look for package.xml in parent directories (up to 3 levels)
    for parent in [launch_dir, launch_dir.parent, launch_dir.parent.parent]:
        package_xml = parent / 'package.xml'
        if package_xml.exists():
            # Check if this is the right package
            if package_name in package_xml.read_text():
                print(f"Found source directory: {parent}")
                return parent
    
    # Method 2: Use ROS_PACKAGE_PATH if available
    if 'ROS_PACKAGE_PATH' in os.environ:
        for path in os.environ['ROS_PACKAGE_PATH'].split(':'):
            package_dir = Path(path) / package_name
            if (package_dir / 'package.xml').exists():
                print(f"Found via ROS_PACKAGE_PATH: {package_dir}")
                return package_dir
    
    # Method 3: Try to find in current workspace
    # Look for typical ROS2 workspace structure
    cwd = Path.cwd()
    for parent in [cwd, cwd.parent]:
        # Check for src/ directory
        src_dir = parent / 'src' / package_name
        if (src_dir / 'package.xml').exists():
            print(f"Found in workspace src/: {src_dir}")
            return src_dir
    
    # Method 4: Look in standard locations
    possible_paths = [
        Path.home() / 'ros2_ws' / 'src' / package_name,
        Path.home() / 'ros_ws' / 'src' / package_name,
        Path.home() / 'workspace' / 'src' / package_name,
    ]
    
    for path in possible_paths:
        if (path / 'package.xml').exists():
            print(f"Found in standard location: {path}")
            return path
    
    raise FileNotFoundError(
        f"Could not find source directory for package '{package_name}'.\n"
        f"Current directory: {cwd}\n"
        f"Please ensure you're in the ROS2 workspace root and the package is built."
    )

def find_world_file(package_dir, world_name="reference.wbt"):
    """
    Find the world file in the package source directory.
    
    Args:
        package_dir: Path to package source directory
        world_name: Name of the world file
        
    Returns:
        Path to the world file
    """
    # First check in worlds/ directory
    world_path = package_dir / 'worlds' / world_name
    if world_path.exists():
        return world_path
    
    # Check for world file directly in package
    world_path = package_dir / world_name
    if world_path.exists():
        return world_path
    
    # Check in subdirectories
    for root, dirs, files in os.walk(package_dir):
        for file in files:
            if file == world_name:
                return Path(root) / file
    
    raise FileNotFoundError(
        f"World file '{world_name}' not found in package '{package_dir}'.\n"
        f"Searched in: {package_dir}/worlds/\n"
        f"And recursively in all subdirectories."
    )

def setup_webots_environment(package_dir):
    """
    Setup environment variables for Webots.
    
    Critical variables:
    - WEBOTS_PROJECT_PATH: Where Webots looks for project files
    - WEBOTS_HOME: Location of Webots installation
    - LD_LIBRARY_PATH: For Webots libraries
    - PATH: For Webots executables
    
    Args:
        package_dir: Path to package source directory
        
    Returns:
        Dictionary of environment variables
    """
    env = os.environ.copy()
    
    # CRITICAL: Set WEBOTS_PROJECT_PATH to the SOURCE directory
    env['WEBOTS_PROJECT_PATH'] = str(package_dir)
    print(f"Setting WEBOTS_PROJECT_PATH to: {package_dir}")
    
    # Find Webots home directory if not set
    if 'WEBOTS_HOME' not in env:
        # Try common Webots installation paths
        possible_paths = [
            '/usr/local/webots',
            '/usr/local/webots-latest',
            '/opt/webots',
            '/opt/webots-latest',
            Path.home() / 'webots',
            Path.home() / '.local' / 'webots',
        ]
        
        # Also try to find via 'which webots'
        try:
            webots_path = subprocess.check_output(['which', 'webots'], 
                                                  text=True).strip()
            # Get the installation directory (usually parent of bin/webots)
            webots_bin = Path(webots_path)
            if webots_bin.parent.name == 'bin':
                env['WEBOTS_HOME'] = str(webots_bin.parent.parent)
        except:
            pass
        
        # Check if we found it
        if 'WEBOTS_HOME' not in env:
            for path in possible_paths:
                if Path(path).exists():
                    env['WEBOTS_HOME'] = str(path)
                    break
    
    # Add Webots libraries to LD_LIBRARY_PATH if needed
    if 'WEBOTS_HOME' in env:
        webots_lib = Path(env['WEBOTS_HOME']) / 'lib'
        if webots_lib.exists():
            if 'LD_LIBRARY_PATH' in env:
                env['LD_LIBRARY_PATH'] = f"{webots_lib}:{env['LD_LIBRARY_PATH']}"
            else:
                env['LD_LIBRARY_PATH'] = str(webots_lib)
    
    return env

def create_webots_symlinks(package_dir):
    """
    Create symlinks that Webots expects in the package directory.
    
    Webots expects specific directory structure:
    - worlds/reference/controllers/ → symlink to controllers/
    - worlds/reference/protos/ → symlink to protos/
    - Any other directories needed for your world
    
    Args:
        package_dir: Path to package source directory
    """
    print("\n=== Creating Webots Symlinks ===")
    
    # Define source and target directories
    world_dir = package_dir / 'worlds' / 'reference'
    
    # Ensure world directory exists
    world_dir.mkdir(parents=True, exist_ok=True)
    
    # List of directories to symlink
    symlinks_to_create = [
        ('controllers', world_dir / 'controllers'),
        ('protos', world_dir / 'protos'),
        ('textures', world_dir / 'textures'),
        ('libraries', world_dir / 'libraries'),
    ]
    
    for source_name, target_path in symlinks_to_create:
        source_path = package_dir / source_name
        
        if source_path.exists():
            # Remove existing symlink or directory
            if target_path.exists():
                if target_path.is_symlink():
                    target_path.unlink()
                else:
                    shutil.rmtree(target_path)
            
            # Create symlink
            try:
                target_path.symlink_to(source_path, target_is_directory=True)
                print(f"✓ Created symlink: {target_path} → {source_path}")
            except Exception as e:
                print(f"✗ Failed to create symlink {target_path}: {e}")
        else:
            print(f"ℹ Source directory not found: {source_path}")
    
    # CRITICAL: Also ensure controller binaries are available
    # Webots controllers need to be executable and in the right place
    controllers_dir = package_dir / 'controllers'
    if controllers_dir.exists():
        for item in controllers_dir.iterdir():
            if item.is_dir():
                # Look for executable files
                for file in item.glob('*'):
                    if file.is_file() and os.access(file, os.X_OK):
                        # Ensure it's executable
                        file.chmod(0o755)
    
    print("=== Webots Symlinks Created ===\n")

def update_webots_world_file(package_dir, old_name, new_name):
    """
    Helper function to update references in world file.
    
    Use this when you rename controllers or protos in your Webots world.
    
    Args:
        package_dir: Package source directory
        old_name: Old name to replace
        new_name: New name to use
    """
    world_file = find_world_file(package_dir, "reference.wbt")
    
    if world_file.exists():
        content = world_file.read_text()
        
        # Replace all occurrences
        updated_content = content.replace(old_name, new_name)
        
        if content != updated_content:
            world_file.write_text(updated_content)
            print(f"Updated world file: Replaced '{old_name}' with '{new_name}'")
            return True
        else:
            print(f"No changes needed: '{old_name}' not found in world file")
            return False
    else:
        print(f"World file not found: {world_file}")
        return False
