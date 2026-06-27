"""
Localization Node for Lunar Rover Navigation.

Provides rover pose estimation using:
- Odometry fusion
- IMU integration
- (Extensible for GPS, visual odometry, etc.)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math


class LocalizationNode(Node):
    """
    Localization node for lunar rover navigation.
    
    Provides:
    - Pose estimation from sensor fusion
    - TF publishing for visualization
    - Covariance tracking
    """
    
    def __init__(self):
        super().__init__('localization_node')
        
        # Parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_imu', True)
        self.declare_parameter('imu_bias', [0.0, 0.0, 0.0])
        
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_imu = self.get_parameter('use_imu').value
        
        # State estimate
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        
        # Covariance (6x6: x, y, z, roll, pitch, yaw)
        self.covariance = np.eye(6) * 0.01
        
        # IMU data
        self.imu_angular_velocity = [0.0, 0.0, 0.0]
        self.imu_linear_acceleration = [0.0, 0.0, 0.0]
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscriptions
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self._odom_callback,
            qos
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self._imu_callback,
            qos
        )
        
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self._initial_pose_callback,
            qos
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            qos
        )
        
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos
        )
        
        # Timer for TF and pose publishing
        self.timer = self.create_timer(0.05, self._publish_loop)
        
        self.get_logger().info('Localization node initialized')
    
    def _odom_callback(self, msg: Odometry):
        """Handle odometry updates."""
        # Extract pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = self._get_yaw(msg.pose.pose.orientation)
        
        # Extract velocity
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y
        self.wz = msg.twist.twist.angular.z
        
        # Update covariance
        if len(msg.pose.covariance) == 36:
            self.covariance = np.array(msg.pose.covariance).reshape(6, 6)
    
    def _imu_callback(self, msg: Imu):
        """Handle IMU updates."""
        self.imu_angular_velocity = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        self.imu_linear_acceleration = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        
        if self.use_imu:
            # Fuse IMU with odometry for better heading estimation
            # Simplified: use IMU angular velocity for heading
            self.wz = self.imu_angular_velocity[2]
    
    def _initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle initial pose reset."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = self._get_yaw(msg.pose.pose.orientation)
        
        if len(msg.pose.covariance) == 36:
            self.covariance = np.array(msg.pose.covariance).reshape(6, 6)
        
        self.get_logger().info(f'Initial pose set: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}')
    
    def _publish_loop(self):
        """Publish TF, pose, and odometry."""
        if not self.publish_tf:
            return
            
        now = self.get_clock().now()
        
        # Publish TF: map -> odom -> base_link
        self._publish_tf(now)
        
        # Publish pose estimate
        self._publish_pose(now)
        
        # Publish odometry
        self._publish_odom(now)
    
    def _publish_tf(self, now):
        """Publish transform tree."""
        # map -> odom transform (identity for now, would use SLAM/AMCL)
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = now.to_msg()
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        map_to_odom.transform.translation.x = 0.0
        map_to_odom.transform.translation.y = 0.0
        map_to_odom.transform.translation.z = 0.0
        map_to_odom.transform.rotation.x = 0.0
        map_to_odom.transform.rotation.y = 0.0
        map_to_odom.transform.rotation.z = 0.0
        map_to_odom.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(map_to_odom)
        
        # odom -> base_link transform
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = now.to_msg()
        odom_to_base.header.frame_id = self.odom_frame
        odom_to_base.child_frame_id = self.base_frame
        odom_to_base.transform.translation.x = self.x
        odom_to_base.transform.translation.y = self.y
        odom_to_base.transform.translation.z = 0.0
        
        quat = self._euler_to_quaternion(0, 0, self.theta)
        odom_to_base.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(odom_to_base)
    
    def _publish_pose(self, now):
        """Publish pose estimate with covariance."""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.map_frame
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._euler_to_quaternion(0, 0, self.theta)
        
        msg.pose.covariance = self.covariance.flatten().tolist()
        
        self.pose_pub.publish(msg)
    
    def _publish_odom(self, now):
        """Publish odometry message."""
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = self._euler_to_quaternion(0, 0, self.theta)
        msg.pose.covariance = self.covariance.flatten().tolist()
        
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.wz
        
        self.odom_pub.publish(msg)
    
    @staticmethod
    def _get_yaw(quaternion) -> float:
        """Extract yaw angle from quaternion."""
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
        """Convert Euler angles to quaternion."""
        from geometry_msgs.msg import Quaternion
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def get_current_pose(self) -> tuple:
        """Get current estimated pose."""
        return (self.x, self.y, self.theta)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()