"""
Local Controller ROS 2 Node.

Provides local trajectory tracking using DWA and Pure Pursuit controllers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path, Odometry
from moon_rover_navigation.local_planners import (
    DynamicWindowApproach, PurePursuitController,
    VelocityCommand, LocalPlannerFactory
)


class ControllerNode(Node):
    """
    Local controller node for lunar rover navigation.
    
    Provides:
    - DWA trajectory optimization
    - Pure pursuit path tracking
    - Velocity command publishing
    """
    
    def __init__(self):
        super().__init__('controller_node')
        
        # Declare parameters
        self.declare_parameter('controller_type', 'pure_pursuit')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('use_sim_time', True)
        
        # Controller parameters
        self.declare_parameter('max_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        self.declare_parameter('lookahead_distance', 0.6)
        self.declare_parameter('desired_velocity', 0.3)
        
        self.controller_type = self.get_parameter('controller_type').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_vx = 0.0
        self.current_wz = 0.0
        
        # Current path
        self.current_path = []
        self.path_received = False
        
        # Initialize controller
        self._initialize_controller()
        
        # QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            qos
        )
        
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self._path_callback,
            qos
        )
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, qos)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self._control_loop)
        
        self.get_logger().info(f'Controller node initialized with {self.controller_type}')
    
    def _initialize_controller(self):
        """Initialize the local controller."""
        max_vel = self.get_parameter('max_velocity').value
        max_ang_vel = self.get_parameter('max_angular_velocity').value
        lookahead = self.get_parameter('lookahead_distance').value
        desired_vel = self.get_parameter('desired_velocity').value
        
        if self.controller_type == 'dwa':
            self.controller = DynamicWindowApproach(
                max_velocity=max_vel,
                max_angular_velocity=max_ang_vel,
                velocity_resolution=15,
                angular_resolution=15,
                prediction_horizon=1.0
            )
        elif self.controller_type == 'pure_pursuit':
            self.controller = PurePursuitController(
                lookahead_distance=lookahead,
                max_angular_velocity=max_ang_vel
            )
        else:
            self.get_logger().warn(f'Unknown controller {self.controller_type}, using Pure Pursuit')
            self.controller = PurePursuitController()
    
    def _odom_callback(self, msg: Odometry):
        """Handle odometry updates."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = self._get_yaw(msg.pose.pose.orientation)
        
        # Calculate velocity from odometry
        self.current_vx = msg.twist.twist.linear.x
        self.current_wz = msg.twist.twist.angular.z
    
    def _path_callback(self, msg: Path):
        """Handle path updates."""
        self.current_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.path_received = True
        self.get_logger().debug(f'Received path with {len(self.current_path)} waypoints')
    
    def _control_loop(self):
        """Main control loop."""
        if not self.path_received or len(self.current_path) < 2:
            # No path, stop rover
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return
        
        # Get goal from path
        goal = self.current_path[-1]
        
        # Compute velocity command
        if self.controller_type == 'dwa':
            command = self.controller.compute_velocity(
                self.current_x, self.current_y, self.current_theta,
                self.current_vx, self.current_wz,
                goal[0], goal[1], 0.0  # Goal theta
            )
        else:  # pure_pursuit
            desired_vel = self.get_parameter('desired_velocity').value
            command = self.controller.compute_command(
                self.current_x, self.current_y, self.current_theta,
                self.current_vx,
                self.current_path,
                desired_vel
            )
        
        # Publish command
        twist = Twist()
        twist.linear.x = command.vx
        twist.angular.z = command.wz
        self.cmd_pub.publish(twist)
    
    @staticmethod
    def _get_yaw(quaternion) -> float:
        """Extract yaw angle from quaternion."""
        # Simplified - would use tf2 in real code
        import math
        # Basic extraction from quaternion
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def check_goal_reached(self) -> bool:
        """Check if goal has been reached."""
        if not self.current_path:
            return False
        
        goal = self.current_path[-1]
        dx = goal[0] - self.current_x
        dy = goal[1] - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        return distance < 0.3  # 30cm tolerance


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()