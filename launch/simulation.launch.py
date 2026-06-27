"""
Simulation Launch File for Lunar Rover Navigation.

Launches:
- Gazebo simulation with lunar terrain
- Rover model spawn
- Navigation stack
- Visualization tools
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package paths
    pkg_dir = get_package_share_directory('moon_rover_navigation')
    rover_gazebo_dir = get_package_share_directory('rover_gazebo')
    rover_nav_dir = get_package_share_directory('rover_navigation')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='moon')
    planner = LaunchConfiguration('planner', default='hybrid_astar')
    controller = LaunchConfiguration('controller', default='pure_pursuit')
    
    # Set use_sim_time parameter globally
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Create actions
    actions = [set_sim_time]
    
    # Gazebo simulation group
    gazebo_actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rover_gazebo_dir, 'launch', f'{world.perform_substitution()}.launch.py')
            ),
            launch_arguments={
                'nav2_planner': planner,
                'nav2_controller': controller,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ]
    
    gazebo_group = GroupAction(gazebo_actions, scoped=True)
    actions.append(gazebo_group)
    
    # Navigation stack nodes
    nav_actions = [
        # Map server node
        Node(
            package='moon_rover_navigation',
            executable='map_server_node',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Planner node
        Node(
            package='moon_rover_navigation',
            executable='planner_node',
            name='planner',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'planner_type': planner,
                'costmap_topic': '/global_costmap/costmap',
            }],
        ),
        
        # Controller node
        Node(
            package='moon_rover_navigation',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'controller_type': controller,
                'odom_topic': '/diff_drive_controller/odom',
                'path_topic': '/planned_path',
            }],
        ),
        
        # Localization node
        Node(
            package='moon_rover_navigation',
            executable='localization_node',
            name='localization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Visualization node
        Node(
            package='moon_rover_navigation',
            executable='visualization_node',
            name='visualization',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]
    
    actions.extend(nav_actions)
    
    # RViz configuration
    rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation.rviz')
    if os.path.exists(rviz_config):
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )
    
    return LaunchDescription(actions)