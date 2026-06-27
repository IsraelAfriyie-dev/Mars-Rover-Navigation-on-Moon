# 🛰️ Lunar Rover Autonomous Navigation Framework 🌕

A complete autonomous navigation framework for lunar rovers operating on unstructured lunar terrain, built with ROS 2 Humble.

## Features

- **Global Path Planning**: A*, Hybrid A*, and RRT* algorithms
- **Local Control**: Dynamic Window Approach (DWA) and Pure Pursuit
- **Terrain Analysis**: Multi-factor cost system including elevation, slope, rock hazards, energy, and illumination
- **Ackermann Kinematics**: Respects rover steering constraints and minimum turning radius
- **Gazebo Simulation**: Ready-to-run lunar terrain simulation
- **Jupyter Notebooks**: Interactive algorithm comparison and analysis

## Quick Start

### Launch Full Simulation

```bash
ros2 launch moon_rover_navigation simulation.launch.py
```

### Launch Planner Only

```bash
ros2 launch moon_rover_navigation planner.launch.py planner_type:=hybrid_astar
```

### Launch Controller Only

```bash
ros2 launch moon_rover_navigation controller.launch.py controller_type:=pure_pursuit
```

## Project Structure

```
moon_rover_navigation/
├── src/                    # Source code
│   └── moon_rover_navigation/
│       ├── nodes/          # ROS 2 nodes
│       ├── planners.py     # Global planners
│       ├── local_planners.py  # Local controllers
│       ├── terrain_cost.py # Terrain cost system
│       └── rover_constraints.py  # Rover kinematics
│
├── launch/                 # Launch files
├── config/                 # Configuration
├── docs/                   # Documentation
├── notebooks/              # Jupyter notebooks
├── tests/                  # Unit tests
├── maps/                   # Map storage
├── worlds/                 # Gazebo worlds
└── outputs/                # Planning outputs
```

## Navigation Stack

- **Global Planners**: A* (baseline), Hybrid A* (non-holonomic), RRT* (exploration)
- **Local Controllers**: DWA (optimization), Pure Pursuit (geometric)
- **Terrain Costs**: Elevation, Slope, Rock, Energy, Illumination


## Installation

```bash
# Clone repository
git clone https://github.com/IsraelAfriyie-dev/Mars-Rover-Navigation-on-Moon.git
cd Mars-Rover-Navigation-on-Moon

# Install dependencies
rosdep install --from-paths src -r -y

# Build
colcon build

# Source
source install/setup.bash
```

## Documentation

- [Installation Guide](docs/installation.md)
- [Architecture Documentation](docs/README.md)
- [Planner Comparison](docs/planner_comparison.md)

## Testing

```bash
# Run all tests
pytest tests/

# Run specific test file
pytest tests/test_planners.py -v
```

## Rover Constraints

- **Ackermann Steering**: Minimum turning radius of 0.5m
- **Maximum Slope**: 30° forward/backward, 25° lateral
- **Maximum Velocity**: 0.5 m/s
- **Maximum Acceleration**: 0.3 m/s²

## License

MIT License

## Acknowledgments

Based on the [ros2_rover](https://github.com/mgonzs13/ros2_rover) project.

Built with ROS 2 Humble, Nav2, and Gazebo.



