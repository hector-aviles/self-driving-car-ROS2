#!/bin/bash
# setup.sh - Setup script for self-driving car ROS2 package

set -e

echo "=== Setting up Self-Driving Car ROS2 Package ==="

# Check if in ROS2 workspace
if [ ! -f "install/setup.bash" ] && [ ! -f "devel/setup.bash" ]; then
    echo "Warning: Not in a ROS2 workspace root."
    echo "Please run this script from your ROS2 workspace directory."
    read -p "Are you in the workspace directory? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Please cd to your ROS2 workspace and run again."
        exit 1
    fi
fi

# Check for Webots
if ! command -v webots &> /dev/null; then
    echo "Warning: Webots not found in PATH."
    echo "Please install Webots R2023a or later from:"
    echo "https://github.com/cyberbotics/webots/releases"
    echo ""
    echo "After installation, make sure to:"
    echo "1. Add Webots to your PATH"
    echo "2. Set WEBOTS_HOME environment variable"
    read -p "Continue anyway? (y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "✓ Webots found: $(webots --version 2>/dev/null || echo 'version unknown')"
fi

# Source ROS2
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
    echo "✓ Sourced ROS2 $ROS_DISTRO"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
    echo "✓ Sourced ROS2 humble"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source "/opt/ros/foxy/setup.bash"
    echo "✓ Sourced ROS2 foxy"
else
    echo "Warning: Could not find ROS2 setup.bash"
    echo "Please source ROS2 manually: source /opt/ros/<distro>/setup.bash"
fi

# Build package
echo ""
echo "Building self_driving_car package..."
colcon build --packages-select self_driving_car --symlink-install

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    echo "✓ Sourced workspace"
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "To run the simulation:"
echo "1. Make sure you're in the workspace directory"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Launch: ros2 launch self_driving_car portable_launch.py"
echo ""
echo "Optional arguments:"
echo "  max_speed:=30.0      # Maximum speed"
echo "  gui:=false           # Run without GUI"
echo "  mode:=fast           # Run in fast mode"
echo ""
echo "Example:"
echo "  ros2 launch self_driving_car portable_launch.py max_speed:=30.0 gui:=false"
