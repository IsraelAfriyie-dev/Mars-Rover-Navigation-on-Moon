"""
Visualization Node for Lunar Rover Navigation.

Provides RViz visualization for:
- Planned paths
- Rover trajectory
- Costmaps
- Terrain data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import ColorRGBA
import numpy as np


class VisualizationNode(Node):
    """
    Visualization node for lunar rover navigation.
    
    Provides markers for RViz visualization:
    - Path visualization
    - Waypoint markers
    - Rover position marker
    - Costmap overlay
    """
    
    def __init__(self):
        super().__init__('visualization_node')
        
        # Parameters
        self.declare_parameter('path_color', [0.0, 1.0, 0.0, 1.0])  # RGBA green
        self.declare_parameter('waypoint_color', [1.0, 0.5, 0.0, 1.0])  # Orange
        self.declare_parameter('rover_color', [0.0, 0.0, 1.0, 1.0])  # Blue
        self.declare_parameter('marker_lifetime', 5.0)
        
        # Marker lifetimes
        self.marker_lifetime = self.get_parameter('marker_lifetime').value
        
        # Subscriber callbacks storage
        self.current_path = None
        self.current_costmap = None
        self.rover_pose = None
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.path_sub = self.create_subscription(
            Path,
            '/planned_path',
            self._path_callback,
            qos
        )
        
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self._costmap_callback,
            qos
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self._pose_callback,
            qos
        )
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_markers',
            qos
        )
        
        self.path_marker_pub = self.create_publisher(
            Marker,
            '/path_marker',
            qos
        )
        
        self.waypoint_marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            qos
        )
        
        # Timer for periodic visualization
        self.timer = self.create_timer(0.2, self._visualization_loop)
        
        self.get_logger().info('Visualization node initialized')
    
    def _path_callback(self, msg: Path):
        """Handle path updates."""
        self.current_path = msg
    
    def _costmap_callback(self, msg: OccupancyGrid):
        """Handle costmap updates."""
        self.current_costmap = msg
    
    def _pose_callback(self, msg: PoseStamped):
        """Handle rover pose updates."""
        self.rover_pose = msg
    
    def _visualization_loop(self):
        """Main visualization loop."""
        # Publish path marker
        if self.current_path is not None:
            self._publish_path_marker()
            self._publish_waypoint_markers()
        
        # Publish rover marker
        if self.rover_pose is not None:
            self._publish_rover_marker()
    
    def _publish_path_marker(self):
        """Publish path as line strip."""
        marker = Marker()
        marker.header = self.current_path.header
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
        
        # Scale
        marker.scale.x = 0.05  # Line width
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color
        color = self.get_parameter('path_color').value
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        
        # Points
        for pose in self.current_path.poses:
            p = Point()
            p.x = pose.pose.position.x
            p.y = pose.pose.position.y
            p.z = pose.pose.position.z
            marker.points.append(p)
        
        self.path_marker_pub.publish(marker)
    
    def _publish_waypoint_markers(self):
        """Publish waypoint markers."""
        marker_array = MarkerArray()
        
        for i, pose in enumerate(self.current_path.poses):
            marker = Marker()
            marker.header = pose.header
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
            marker.id = i
            
            # Scale
            if i == 0:
                # Start - larger
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
            elif i == len(self.current_path.poses) - 1:
                # Goal - larger
                marker.scale.x = 0.3
                marker.scale.y = 0.3
                marker.scale.z = 0.3
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
            else:
                # Intermediate - smaller
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                color = self.get_parameter('waypoint_color').value
                marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
            
            marker.pose = pose.pose
            
            marker_array.markers.append(marker)
        
        self.waypoint_marker_pub.publish(marker_array)
    
    def _publish_rover_marker(self):
        """Publish rover position marker."""
        marker = Marker()
        marker.header = self.rover_pose.header
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.lifetime = rclpy.duration.Duration(seconds=self.marker_lifetime).to_msg()
        marker.id = 0
        
        # Scale
        marker.scale.x = 0.5  # Length
        marker.scale.y = 0.2  # Width
        marker.scale.z = 0.1  # Height
        
        # Color
        color = self.get_parameter('rover_color').value
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        
        marker.pose = self.rover_pose.pose
        
        # Publish as marker array for consistency
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
    
    def publish_costmap_visualization(self):
        """Publish costmap as marker array (for overlay visualization)."""
        if self.current_costmap is None:
            return
            
        marker_array = MarkerArray()
        
        resolution = self.current_costmap.info.resolution
        width = self.current_costmap.info.width
        height = self.current_costmap.info.height
        origin = self.current_costmap.info.origin.position
        
        for i in range(min(width * height, 1000)):  # Limit for performance
            row = i // width
            col = i % width
            
            cost = self.current_costmap.data[i]
            if cost < 0 or cost > 100:
                continue
                
            marker = Marker()
            marker.header = self.current_costmap.header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = i
            
            # Position
            marker.pose.position.x = origin.x + col * resolution + resolution / 2
            marker.pose.position.y = origin.y + row * resolution + resolution / 2
            marker.pose.position.z = 0.05
            
            # Scale
            marker.scale.x = resolution * 0.9
            marker.scale.y = resolution * 0.9
            marker.scale.z = 0.01
            
            # Color based on cost (green=free, red=occupied)
            normalized_cost = cost / 100.0
            marker.color = ColorRGBA(
                r=normalized_cost,
                g=1.0 - normalized_cost,
                b=0.0,
                a=0.5
            )
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()