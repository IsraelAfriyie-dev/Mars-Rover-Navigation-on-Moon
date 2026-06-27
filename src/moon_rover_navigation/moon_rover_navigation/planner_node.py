"""
Global Planner ROS 2 Node.

Provides global path planning services using A*, Hybrid A*, and RRT* algorithms.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path, OccupancyGrid
from moon_rover_navigation_msgs.srv import PlanPath, ComparePlanners
from moon_rover_navigation.terrain_cost import TerrainCostCalculator
from moon_rover_navigation.planners import (
    AStarPlanner, HybridAStarPlanner, RRTStarPlanner,
    PlannerType, PlannerFactory
)


class PlannerNode(Node):
    """
    Global planner node for lunar rover navigation.
    
    Provides:
    - Path planning service
    - Multi-planner comparison
    - Terrain-aware planning
    """
    
    def __init__(self):
        super().__init__('planner_node')
        
        # Declare parameters
        self.declare_parameter('planner_type', 'hybrid_astar')
        self.declare_parameter('grid_resolution', 0.2)
        self.declare_parameter('use_costmap', True)
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('goal_tolerance', 0.5)
        
        self.planner_type = self.get_parameter('planner_type').value
        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.use_costmap = self.get_parameter('use_costmap').value
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        
        # Initialize cost calculator
        self.cost_calculator = TerrainCostCalculator()
        
        # Current costmap
        self.costmap = None
        self.costmap_resolution = 0.05
        self.costmap_origin = (0.0, 0.0)
        
        # Initialize planner based on type
        self._initialize_planner()
        
        # QoS profile for reliable communication
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self._costmap_callback,
            qos
        )
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', qos)
        self.poses_pub = self.create_publisher(PoseArray, '/planned_poses', qos)
        
        # Services
        self.plan_service = self.create_service(
            PlanPath,
            '/plan_path',
            self._plan_path_callback
        )
        
        self.compare_service = self.create_service(
            ComparePlanners,
            '/compare_planners',
            self._compare_planners_callback
        )
        
        self.get_logger().info(f'Planner node initialized with {self.planner_type} planner')
    
    def _initialize_planner(self):
        """Initialize the path planner based on configuration."""
        kwargs = {
            'grid_resolution': self.grid_resolution,
            'costmap': self.costmap
        }
        
        if self.planner_type == 'astar':
            self.planner = AStarPlanner(**kwargs)
        elif self.planner_type == 'hybrid_astar':
            kwargs['theta_resolution'] = 36
            kwargs['min_turning_radius'] = 0.5
            self.planner = HybridAStarPlanner(**kwargs)
        elif self.planner_type == 'rrt_star':
            self.planner = RRTStarPlanner(bounds=(0, 50, 0, 50), **kwargs)
        else:
            self.get_logger().warn(f'Unknown planner type {self.planner_type}, using Hybrid A*')
            self.planner = HybridAStarPlanner(**kwargs)
    
    def _costmap_callback(self, msg: OccupancyGrid):
        """Handle costmap updates."""
        self.costmap = np.array(msg.data).reshape(
            msg.info.height, msg.info.width
        )
        # Convert to 0-1 range (costmap is 0-255 typically)
        self.costmap = self.costmap / 255.0
        self.costmap_resolution = msg.info.resolution
        self.costmap_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # Update planner costmap
        self.planner.costmap = self.costmap
    
    def _plan_path_callback(self, request, response):
        """Handle path planning service requests."""
        self.get_logger().info('Received path planning request')
        
        try:
            start = (
                request.start.position.x,
                request.start.position.y,
                self._get_yaw(request.start.orientation)
            )
            goal = (
                request.goal.position.x,
                request.goal.position.y,
                self._get_yaw(request.goal.orientation)
            )
            
            grid_shape = (100, 100)  # Default, should use costmap size
            
            # Plan path
            path = self.planner.plan(start, goal, grid_shape)
            
            if path is None:
                response.success = False
                response.message = 'No path found'
                return response
            
            # Convert to Path message
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            
            for point in path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                if len(point) > 2:
                    pose.pose.position.z = 0.0
                    # Set orientation from theta
                    quat = self._euler_to_quaternion(0, 0, point[2])
                    pose.pose.orientation = quat
                path_msg.poses.append(pose)
            
            response.path = path_msg
            response.success = True
            response.message = 'Path planned successfully'
            
            # Publish for visualization
            self.path_pub.publish(path_msg)
            
        except Exception as e:
            self.get_logger().error(f'Path planning failed: {str(e)}')
            response.success = False
            response.message = str(e)
            
        return response
    
    def _compare_planners_callback(self, request, response):
        """Compare multiple planners on the same planning problem."""
        self.get_logger().info('Comparing planners')
        
        start = (request.start.position.x, request.start.position.y)
        goal = (request.goal.position.x, request.goal.position.y)
        grid_shape = (100, 100)
        
        planners_to_compare = ['astar', 'hybrid_astar', 'rrt_star']
        response.results = []
        
        for planner_name in planners_to_compare:
            result = PlannerResult()
            
            try:
                if planner_name == 'astar':
                    planner = AStarPlanner(grid_resolution=0.1, costmap=self.costmap)
                    path = planner.plan(start, goal, grid_shape)
                elif planner_name == 'hybrid_astar':
                    planner = HybridAStarPlanner(
                        grid_resolution=0.2,
                        theta_resolution=36,
                        min_turning_radius=0.5,
                        costmap=self.costmap
                    )
                    path = planner.plan(
                        (start[0], start[1], 0.0),
                        (goal[0], goal[1], 0.0),
                        grid_shape
                    )
                elif planner_name == 'rrt_star':
                    planner = RRTStarPlanner(
                        bounds=(0, 50, 0, 50),
                        max_iterations=500,
                        costmap=self.costmap
                    )
                    path = planner.plan(start, goal)
                else:
                    continue
                
                if path:
                    result.planner_name = planner_name
                    result.path_length = self._calculate_path_length(path)
                    result.planning_time = 0.0  # Would need timing instrumentation
                    result.success = True
                else:
                    result.planner_name = planner_name
                    result.success = False
                    
            except Exception as e:
                result.planner_name = planner_name
                result.success = False
                self.get_logger().error(f'{planner_name} failed: {str(e)}')
            
            response.results.append(result)
            
        return response
    
    def _calculate_path_length(self, path) -> float:
        """Calculate total path length."""
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            length += np.sqrt(dx**2 + dy**2)
        return length
    
    @staticmethod
    def _get_yaw(quaternion) -> float:
        """Extract yaw angle from quaternion."""
        # Simplified - would use tf2 transformations in real code
        return 0.0
    
    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        """Convert Euler angles to quaternion."""
        # Simplified implementation
        import geometry_msgs.msg
        q = geometry_msgs.msg.Quaternion()
        # Would use proper conversion
        return q


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()