"""
Map Server ROS 2 Node.

Provides map services including:
- Terrain costmap generation
- Traversability analysis
- DEM processing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import PoseStamped
from moon_rover_navigation.terrain_cost import TerrainCostCalculator, LunarTerrainAnalyzer
import numpy as np


class MapServerNode(Node):
    """
    Map server node for lunar terrain mapping.
    
    Provides:
    - Costmap generation from terrain data
    - Traversability analysis
    - DEM processing
    """
    
    def __init__(self):
        super().__init__('map_server_node')
        
        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('map_resolution', 0.1)  # meters per cell
        self.declare_parameter('map_width', 100)  # cells
        self.declare_parameter('map_height', 100)  # cells
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)
        self.declare_parameter('latch_duration', 5.0)
        
        self.map_frame = self.get_parameter('map_frame').value
        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        
        # Initialize terrain utilities
        self.cost_calculator = TerrainCostCalculator()
        self.terrain_analyzer = LunarTerrainAnalyzer(resolution=self.map_resolution)
        
        # Costmap storage
        self.elevation_map = np.zeros((self.map_height, self.map_width))
        self.slope_map = np.zeros((self.map_height, self.map_width))
        self.rock_map = np.zeros((self.map_height, self.map_width))
        self.illumination_map = np.ones((self.map_height, self.map_width))
        self.costmap = np.zeros((self.map_height, self.map_width))
        
        # Subscriptions
        self.elevation_sub = self.create_subscription(
            Image,
            '/terrain/elevation',
            self._elevation_callback,
            10
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/terrain/pointcloud',
            self._pointcloud_callback,
            10
        )
        
        # Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/global_costmap/costmap',
            qos
        )
        
        self.costmap_raw_pub = self.create_publisher(
            OccupancyGrid,
            '/costmap_raw',
            qos
        )
        
        self.traversability_pub = self.create_publisher(
            OccupancyGrid,
            '/traversability_map',
            qos
        )
        
        # Service servers
        self.update_service = self.create_service(
            'update_map',
            UpdateMap,
            self._update_map_callback
        )
        
        self.get_cost_at_service = self.create_service(
            GetCostAt,
            '/get_cost_at',
            self._get_cost_at_callback
        )
        
        # Timer for periodic publishing
        self.publish_timer = self.create_timer(1.0, self._publish_maps)
        
        self.get_logger().info('Map server node initialized')
    
    def _elevation_callback(self, msg: Image):
        """Process elevation map from image."""
        try:
            # Convert image to numpy array
            data = np.frombuffer(msg.data, dtype=np.float32)
            self.elevation_map = data.reshape(msg.height, msg.width)
            
            # Compute slope from elevation
            self.slope_map = self.terrain_analyzer.compute_slope_from_elevation(
                self.elevation_map
            )
            
            # Detect rock hazards
            self.rock_map = self.terrain_analyzer.detect_rock_hazards(
                self.elevation_map
            )
            
            # Regenerate costmap
            self._regenerate_costmap()
            
        except Exception as e:
            self.get_logger().error(f'Failed to process elevation: {str(e)}')
    
    def _pointcloud_callback(self, msg: PointCloud2):
        """Process point cloud for terrain analysis."""
        # Extract points and create elevation grid
        # Simplified implementation
        self.get_logger().debug('Point cloud received')
    
    def _regenerate_costmap(self):
        """Regenerate costmap from terrain data."""
        self.costmap = self.cost_calculator.generate_costmap(
            self.elevation_map,
            self.slope_map,
            self.rock_map,
            self.illumination_map
        )
    
    def _publish_maps(self):
        """Publish all map topics."""
        # Publish costmap
        costmap_msg = self._create_occupancy_grid_msg(self.costmap)
        self.costmap_pub.publish(costmap_msg)
        self.costmap_raw_pub.publish(costmap_msg)
        
        # Publish traversability map
        traversability = self.terrain_analyzer.generate_traversability_map(
            self.costmap, threshold=0.7
        )
        traversability_msg = self._create_occupancy_grid_msg(traversability)
        self.traversability_pub.publish(traversability_msg)
    
    def _create_occupancy_grid_msg(self, data: np.ndarray) -> OccupancyGrid:
        """Create OccupancyGrid message from numpy array."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_width
        msg.info.height = self.map_height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        # Origin orientation would be set properly in real code
        
        # Convert to 0-100 scale for occupancy grid
        # 0 = free, 100 = occupied, -1 = unknown
        costmap_scaled = (data * 100).astype(np.int8)
        costmap_scaled = np.clip(costmap_scaled, 0, 100)
        
        msg.data = costmap_scaled.flatten().tolist()
        
        return msg
    
    def _update_map_callback(self, request, response):
        """Handle map update requests."""
        try:
            # Update specific terrain data
            if request.update_elevation:
                # Would update elevation map
                pass
            
            if request.update_slope:
                # Would update slope map
                pass
                
            # Regenerate costmap
            self._regenerate_costmap()
            
            response.success = True
            response.message = 'Map updated successfully'
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            
        return response
    
    def _get_cost_at_callback(self, request, response):
        """Get cost at a specific position."""
        try:
            # Convert world coordinates to grid indices
            gx = int((request.x - self.origin_x) / self.map_resolution)
            gy = int((request.y - self.origin_y) / self.map_resolution)
            
            if 0 <= gx < self.map_width and 0 <= gy < self.map_height:
                response.cost = float(self.costmap[gy, gx])
                response.traversable = self.costmap[gy, gx] < 0.7
                response.success = True
            else:
                response.cost = -1.0
                response.traversable = False
                response.success = False
                
        except Exception as e:
            response.cost = -1.0
            response.traversable = False
            response.success = False
            
        return response
    
    def load_demo_terrain(self):
        """Load a demo terrain for testing."""
        # Create a simple demo terrain with some variation
        x = np.linspace(0, 10, self.map_width)
        y = np.linspace(0, 10, self.map_height)
        X, Y = np.meshgrid(x, y)
        
        # Elevation with some hills
        self.elevation_map = 0.5 * np.sin(X * 0.5) * np.cos(Y * 0.5)
        
        # Compute slopes
        self.slope_map = self.terrain_analyzer.compute_slope_from_elevation(
            self.elevation_map
        )
        
        # Add some rock hazards
        self.rock_map = np.random.rand(self.map_height, self.map_width) * 0.3
        
        # Generate costmap
        self._regenerate_costmap()
        
        self.get_logger().info('Demo terrain loaded')


def main(args=None):
    rclpy.init(args=args)
    node = MapServerNode()
    
    # Load demo terrain for testing
    node.load_demo_terrain()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()