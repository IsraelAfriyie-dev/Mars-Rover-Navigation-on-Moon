# Lunar Rover Autonomous Navigation Framework

A complete autonomous navigation framework for lunar rovers operating on unstructured lunar terrain, built with ROS 2.

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Directory Structure](#directory-structure)
3. [Installation](#installation)
4. [Quick Start](#quick-start)
5. [Planners](#planners)
6. [Controllers](#controllers)
7. [Terrain Costs](#terrain-costs)
8. [Launch Files](#launch-files)
9. [Configuration](#configuration)
10. [Example Missions](#example-missions)

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                      NAVIGATION STACK                                │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐ │
│  │  Perception │    │  Terrain    │    │      Localization       │ │
│  │   Stack     │───▶│  Analysis   │───▶│        Node             │ │
│  └─────────────┘    └─────────────┘    └─────────────────────────┘ │
│         │                  │                      │                  │
│         ▼                  ▼                      ▼                  │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                      TERRAIN COSTMAP                         │   │
│  │  - Elevation  - Slope  - Rock Hazards  - Energy  - Lighting │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                             │                                       │
│         ┌───────────────────┼───────────────────┐                   │
│         ▼                   ▼                   ▼                   │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐ │
│  │   A*        │    │ Hybrid A*   │    │        RRT*             │ │
│  │  (Baseline) │    │(Non-holono) │    │    (Exploration)        │ │
│  └─────────────┘    └─────────────┘    └─────────────────────────┘ │
│         │                   │                   │                   │
│         └───────────────────┼───────────────────┘                   │
│                             ▼                                       │
│                    ┌─────────────┐                                  │
│                    │ GLOBAL PATH │                                  │
│                    └─────────────┘                                  │
│                             │                                       │
│                             ▼                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    LOCAL PLANNING                            │   │
│  │  ┌─────────────────┐        ┌─────────────────────────────┐ │   │
│  │  │      DWA        │        │       Pure Pursuit          │ │   │
│  │  │  (Optimization) │        │     (Geometric Track)       │ │   │
│  │  └─────────────────┘        └─────────────────────────────┘ │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│                    ┌─────────────┐                                  │
│                    │  cmd_vel    │                                  │
│                    │  Commands   │                                  │
│                    └─────────────┘                                  │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
moon_rover_navigation/
├── src/                    # Source code packages
│   └── moon_rover_navigation/
│       ├── nodes/          # ROS 2 node executables
│       │   ├── planner_node.py
│       │   ├── controller_node.py
│       │   ├── localization_node.py
│       │   ├── map_server_node.py
│       │   └── visualization_node.py
│       ├── planners.py     # Global planning algorithms
│       ├── local_planners.py  # Local control algorithms
│       ├── terrain_cost.py # Terrain cost calculation
│       ├── rover_constraints.py  # Rover kinematics
│       └── msg/            # Custom message definitions
│
├── launch/                 # Launch files
│   ├── simulation.launch.py
│   ├── planner.launch.py
│   └── controller.launch.py
│
├── config/                 # Configuration files
│   ├── planners/           # Planner configurations
│   │   ├── astar.yaml
│   │   ├── hybrid_astar.yaml
│   │   └── rrt_star.yaml
│   ├── controllers/        # Controller configurations
│   │   ├── dwa.yaml
│   │   └── pure_pursuit.yaml
│   └── terrain/            # Terrain cost configurations
│       └── lunar_costs.yaml
│
├── docs/                   # Documentation
│   ├── README.md           # This file
│   ├── architecture.md     # Detailed architecture
│   ├── planner_comparison.md
│   ├── installation.md
│   └── missions.md
│
├── notebooks/              # Jupyter notebooks
│   └── planner_comparison.ipynb
│
├── tests/                  # Unit and integration tests
├── maps/                   # Map storage
├── worlds/                 # Gazebo world files
└── outputs/                # Planning outputs, logs
```

---

## Installation

### Prerequisites

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble
- Python 3.8+

### Steps

```bash
# Create workspace
mkdir -p ~/lunar_rover_ws/src
cd ~/lunar_rover_ws

# Clone repository
git clone https://github.com/IsraelAfriyie-dev/Mars-Rover-Navigation-on-Moon.git src/

# Install dependencies
cd src
rosdep install --from-paths . -r -y

# Build packages
cd ~/lunar_rover_ws
colcon build

# Source workspace
source install/setup.bash
```

---

## Quick Start

### 1. Launch Full Simulation

```bash
ros2 launch moon_rover_navigation simulation.launch.py
```

### 2. Launch Planner Only

```bash
ros2 launch moon_rover_navigation planner.launch.py planner_type:=hybrid_astar
```

### 3. Launch Controller Only

```bash
ros2 launch moon_rover_navigation controller.launch.py controller_type:=pure_pursuit
```

---

## Planners

### A* (Baseline)

- **Type**: Grid-based, holonomic
- **Use case**: Simple environments, baseline comparison
- **Strengths**: Fast, optimal path length
- **Limitations**: Ignores vehicle kinematics

```bash
ros2 launch moon_rover_navigation planner.launch.py planner_type:=astar
```

### Hybrid A* (Recommended)

- **Type**: Grid-based, non-holonomic
- **Use case**: Standard navigation, curved paths
- **Strengths**: Respects Ackermann constraints, smooth paths
- **Limitations**: Higher computation than A*

```bash
ros2 launch moon_rover_navigation planner.launch.py planner_type:=hybrid_astar
```

### RRT*

- **Type**: Sampling-based, optimal
- **Use case**: Complex terrain, exploration
- **Strengths**: Handles complex obstacles, asymptotically optimal
- **Limitations**: Variable path quality, higher latency

```bash
ros2 launch moon_rover_navigation planner.launch.py planner_type:=rrt_star
```

---

## Controllers

### Dynamic Window Approach (DWA)

- **Type**: Trajectory optimization
- **Use case**: Obstacle-rich environments
- **Strengths**: Dynamic obstacle avoidance, smooth velocities
- **Parameters**: Prediction horizon, velocity resolution

```bash
ros2 launch moon_rover_navigation controller.launch.py controller_type:=dwa
```

### Pure Pursuit

- **Type**: Geometric tracking
- **Use case**: Smooth path following
- **Strengths**: Simple, reliable, predictable behavior
- **Parameters**: Lookahead distance, desired velocity

```bash
ros2 launch moon_rover_navigation controller.launch.py controller_type:=pure_pursuit
```

---

## Terrain Costs

The navigation stack uses a multi-factor terrain cost system:

| Cost Factor | Weight | Description |
|-------------|--------|-------------|
| Elevation | 1.0 | Penalizes high altitude areas |
| Slope | 2.0 | Penalizes steep terrain (max 30°) |
| Rock Hazard | 3.0 | Penalizes rocky/obstacle areas |
| Energy | 1.5 | Penalizes energy-expensive routes |
| Illumination | 1.0 | Penalizes low-light/shadowed areas |

Total cost is normalized to [0, 1], where 0 is optimal and 1 is blocked.

---

## Launch Files

### simulation.launch.py

Full simulation with Gazebo and navigation stack:

```bash
ros2 launch moon_rover_navigation simulation.launch.py \
    world:=moon \
    planner:=hybrid_astar \
    controller:=pure_pursuit
```

### planner.launch.py

Standalone planner node:

```bash
ros2 launch moon_rover_navigation planner.launch.py \
    planner_type:=hybrid_astar \
    grid_resolution:=0.2
```

### controller.launch.py

Standalone controller node:

```bash
ros2 launch moon_rover_navigation controller.launch.py \
    controller_type:=pure_pursuit \
    max_velocity:=0.5 \
    lookahead_distance:=0.6
```

---

## Configuration

Configuration files are located in `config/`:

- `planners/*.yaml` - Global planner parameters
- `controllers/*.yaml` - Local controller parameters
- `terrain/*.yaml` - Terrain cost weights

### Example: Tuning Hybrid A*

```yaml
# config/planners/hybrid_astar.yaml
planner_type: hybrid_astar
grid_resolution: 0.2
min_turning_radius: 0.5
cost_travel_multiplier: 2.0
reverse_penalty: 2.1
```

---

## Example Missions

### Mission 1: Point-to-Point Navigation

```bash
# Start simulation
ros2 launch moon_rover_navigation simulation.launch.py

# In another terminal, send goal
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{...}"
```

### Mission 2: Waypoint Following

```bash
# Launch with waypoint follower
ros2 launch moon_rover_navigation simulation.launch.py

# Send waypoint array
ros2 topic pub /waypoints nav_msgs/Path "{...}"
```

### Mission 3: Planner Comparison

```bash
# Launch comparison service
ros2 launch moon_rover_navigation planner.launch.py

# Call comparison service
ros2 service call /compare_planners moon_rover_navigation_msgs/srv/ComparePlanners "{...}"
```

---

## Support

For issues and questions, please open an issue on the GitHub repository.

## License

MIT License - See LICENSE file for details