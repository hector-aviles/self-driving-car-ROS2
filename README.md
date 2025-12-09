## Quick Start

1. **Clone and build:**
```bash
# Create workspace (if needed)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone repository
git clone https://github.com/yourusername/self-driving-car-ROS2.git
cd self-driving-car-ROS2

# Run setup script
chmod +x setup.sh
./setup.sh

# Or build manually
cd ~/ros2_ws
colcon build --packages-select self_driving_car
source install/setup.bash
