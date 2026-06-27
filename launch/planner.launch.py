"""
Planner Launch File for Lunar Rover Navigation.

Launches the global planning stack with configurable planners:
- A* (baseline grid-based)
- Hybrid A* (non-holonomic)
- RRT* (exploration)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    planner_type = LaunchConfiguration('planner_type', default='hybrid_astar')
    grid_resolution = LaunchConfiguration('grid_resolution', default='0.2')
    costmap_topic = LaunchConfiguration('costmap_topic', default='/global_costmap/costmap')
    
    # Set global use_sim_time
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )
    
    declare_planner_type = DeclareLaunchArgument(
        'planner_type',
        default_value='hybrid_astar',
        choices=['astar', 'hybrid_astar', 'rrt_star'],
        description='Type of global planner to use'
    )
    
    declare_grid_resolution = DeclareLaunchArgument(
        'grid_resolution',
        default_value='0.2',
        description='Grid resolution in meters'
    )
    
    declare_costmap_topic = DeclareLaunchArgument(
        'costmap_topic',
        default_value='/global_costmap/costmap',
        description='Topic for costmap data'
    )
    
    # Planner node
    planner_node = Node(
        package='moon_rover_navigation',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planner_type': planner_type,
            'grid_resolution': grid_resolution,
            'costmap_topic': costmap_topic,
            'goal_tolerance': 0.5,
            'use_costmap': True,
        }],
    )
    
    # Map server node (for costmap generation)
    map_server_node = Node(
        package='moon_rover_navigation',
        executable='map_server_node',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_resolution': 0.1,
            'map_width': 100,
            'map_height': 100,
        }],
    )
    
    # Visualization node
    viz_node = Node(
        package='moon_rover_navigation',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )
    
    return LaunchDescription([
        set_sim_time,
        declare_use_sim_time,
        declare_planner_type,
        declare_grid_resolution,
        declare_costmap_topic,
        map_server_node,
        planner_node,
        viz_node,
    ])