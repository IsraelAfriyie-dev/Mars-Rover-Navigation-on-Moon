# Installation Guide

## Prerequisites

### System Requirements

- **OS**: Ubuntu 22.04 (Jammy Jellyfish)
- **ROS 2**: Humble Hawksbill
- **Python**: 3.8+
- **RAM**: 8GB minimum (16GB recommended)
- **Disk**: 20GB free space

### Required Software

```bash
# Install ROS 2 Humble (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Verify ROS 2 installation
ros2 --version
# Should output: ros2 humbe
```

## Installation Steps

### 1. Create Workspace

```bash
# Create workspace directory
mkdir -p ~/lunar_rover_ws/src
cd ~/lunar_rover_ws
```

### 2. Clone Repository

```bash
# Clone the repository
git clone https://github.com/IsraelAfriyie-dev/Mars-Rover-Navigation-on-Moon.git src/

# Navigate to repository
cd src/
git checkout moon-navigation-cleanup
```

### 3. Install Dependencies

```bash
# Navigate to workspace root
cd ~/lunar_rover_ws

# Install system dependencies
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-numpy \
    python3-scipy \
    python3-matplotlib \
    python3-opencv \
    ros-humble-nav2-* \
    ros-humble-gazebo-* \
    ros-humble-tf2-*

# Install Python dependencies
pip3 install numpy scipy matplotlib jupyter
```

### 4. Build Packages

```bash
# Build the workspace
cd ~/lunar_rover_ws
source /opt/ros/humble/setup.bash
colcon build

# If you encounter issues, try:
colcon build --symlink-install
```

### 5. Source Workspace

Add to your `~/.bashrc` for convenience:

```bash
echo 'source ~/lunar_rover_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## Verification

### 1. Verify Package Installation

```bash
# List available packages
ros2 pkg list | grep moon_rover

# Should show:
# moon_rover_navigation
```

### 2. Verify Launch Files

```bash
# List launch files
ros2 pkg executables moon_rover_navigation

# Should show:
# planner_node
# controller_node
# localization_node
# map_server_node
# visualization_node
```

### 3. Test Launch

```bash
# Test planner launch (without simulation)
ros2 launch moon_rover_navigation planner.launch.py

# Should start without errors
```

## Docker Support (Optional)

For isolated installation, use Docker:

```bash
# Build Docker image
docker build -t lunar-rover-nav:latest .

# Run container
docker run -it --rm \
    --privileged \
    --network=host \
    -e DISPLAY=$DISPLAY \
    lunar-rover-nav:latest

# Inside container
cd ~/lunar_rover_ws
colcon build
source install/setup.bash
ros2 launch moon_rover_navigation planner.launch.py
```

## Troubleshooting

### Issue: `colcon build` fails

**Solution:**
```bash
# Clean and rebuild
rm -rf build install log
colcon build --cmake-clean-cache
```

### Issue: Missing ROS packages

**Solution:**
```bash
# Install missing dependencies
rosdep install --from-paths src -r -y
```

### Issue: Python import errors

**Solution:**
```bash
# Install Python dependencies
pip3 install -r src/requirements.txt
```

### Issue: Gazebo not launching

**Solution:**
```bash
# Install Gazebo dependencies
sudo apt install ros-humble-gazebo-*
source /opt/ros/humble/setup.bash
```

## Next Steps

After successful installation:

1. Read the [Quick Start Guide](../README.md#quick-start)
2. Review [Planner Comparison](../docs/planner_comparison.md)
3. Try [Example Missions](../docs/missions.md)
4. Explore the [Jupyter Notebooks](../notebooks/)