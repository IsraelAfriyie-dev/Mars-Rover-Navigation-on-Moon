"""
Local Path Tracking Controllers for Lunar Rover Navigation.

Implements:
- Dynamic Window Approach (DWA) - local trajectory optimization
- Pure Pursuit Controller - geometric path following
"""

import numpy as np
from typing import Tuple, List, Optional, Callable
from dataclasses import dataclass


@dataclass
class VelocityCommand:
    """Velocity command for the rover."""
    vx: float  # Linear velocity in m/s
    wz: float  # Angular velocity in rad/s
    steering: float = 0.0  # Steering angle in degrees


@dataclass
class TrajectoryPoint:
    """Point in a trajectory."""
    x: float
    y: float
    theta: float
    vx: float
    wz: float
    cost: float = 0.0


class DynamicWindowApproach:
    """
    Dynamic Window Approach (DWA) local planner.
    
    Samples velocity space and selects optimal command that:
    - Maximizes progress towards goal
    - Minimizes distance to obstacles
    - Respects rover dynamics
    """
    
    def __init__(
        self,
        max_velocity: float = 0.5,
        max_angular_velocity: float = 1.0,
        max_acceleration: float = 0.3,
        max_angular_acceleration: float = 1.0,
        velocity_resolution: int = 10,
        angular_resolution: int = 10,
        prediction_horizon: float = 1.0,
        dt: float = 0.1,
        goal_tolerance: float = 0.2,
        # Cost weights
        heading_weight: float = 0.8,
        distance_weight: float = 0.1,
        velocity_weight: float = 0.1,
        obstacle_cost_scale: float = 1.0,
    ):
        self.max_velocity = max_velocity
        self.max_angular_velocity = max_angular_velocity
        self.max_acceleration = max_acceleration
        self.max_angular_acceleration = max_angular_acceleration
        self.velocity_resolution = velocity_resolution
        self.angular_resolution = angular_resolution
        self.prediction_horizon = prediction_horizon
        self.dt = dt
        self.goal_tolerance = goal_tolerance
        
        # Cost weights
        self.heading_weight = heading_weight
        self.distance_weight = distance_weight
        self.velocity_weight = velocity_weight
        self.obstacle_cost_scale = obstacle_cost_scale
        
    def compute_dynamic_window(
        self,
        current_vx: float,
        current_wz: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute the dynamic window of allowable velocities.
        
        Args:
            current_vx: Current linear velocity
            current_wz: Current angular velocity
            
        Returns:
            Tuple of (velocity_window, angular_velocity_window)
        """
        # Velocity constraints
        v_min = max(0, current_vx - self.max_acceleration * self.dt)
        v_max = min(self.max_velocity, current_vx + self.max_acceleration * self.dt)
        
        # Angular velocity constraints
        w_min = max(-self.max_angular_velocity, current_wz - self.max_angular_acceleration * self.dt)
        w_max = min(self.max_angular_velocity, current_wz + self.max_angular_acceleration * self.dt)
        
        v_window = np.linspace(v_min, v_max, self.velocity_resolution)
        w_window = np.linspace(w_min, w_max, self.angular_resolution)
        
        return v_window, w_window
    
    def predict_trajectory(
        self,
        x: float,
        y: float,
        theta: float,
        vx: float,
        wz: float,
        costmap: Optional[np.ndarray] = None,
        costmap_resolution: float = 0.1
    ) -> List[TrajectoryPoint]:
        """
        Predict trajectory given velocity commands.
        
        Args:
            x, y, theta: Current pose
            vx: Linear velocity
            wz: Angular velocity
            costmap: Optional obstacle costmap
            costmap_resolution: Resolution of costmap
            
        Returns:
            List of trajectory points
        """
        trajectory = []
        num_steps = int(self.prediction_horizon / self.dt)
        
        current_x, current_y, current_theta = x, y, theta
        
        for _ in range(num_steps):
            trajectory.append(TrajectoryPoint(
                x=current_x,
                y=current_y,
                theta=current_theta,
                vx=vx,
                wz=wz
            ))
            
            # Update pose using bicycle model
            if abs(wz) < 1e-6:
                current_x += vx * np.cos(current_theta) * self.dt
                current_y += vx * np.sin(current_theta) * self.dt
            else:
                turn_radius = vx / wz
                current_theta += wz * self.dt
                current_x += turn_radius * (np.sin(current_theta) - np.sin(current_theta - wz * self.dt))
                current_y -= turn_radius * (np.cos(current_theta) - np.cos(current_theta - wz * self.dt))
                
        return trajectory
    
    def trajectory_cost(
        self,
        trajectory: List[TrajectoryPoint],
        goal_x: float,
        goal_y: float,
        goal_theta: float,
        costmap: Optional[np.ndarray] = None,
        costmap_resolution: float = 0.1,
        costmap_origin: Tuple[float, float] = (0, 0)
    ) -> float:
        """
        Calculate cost for a trajectory.
        
        Args:
            trajectory: Predicted trajectory
            goal_x, goal_y, goal_theta: Goal pose
            costmap: Optional obstacle costmap
            costmap_resolution: Resolution of costmap
            costmap_origin: Origin of costmap
            
        Returns:
            Trajectory cost (lower is better)
        """
        if not trajectory:
            return float('inf')
            
        final_point = trajectory[-1]
        
        # Heading cost (angle to goal)
        dx = goal_x - final_point.x
        dy = goal_y - final_point.y
        goal_angle = np.arctan2(dy, dx)
        heading_error = abs(final_point.theta - goal_angle)
        heading_cost = heading_error / np.pi  # Normalize to [0, 1]
        
        # Distance cost
        distance = np.sqrt(dx**2 + dy**2)
        distance_cost = distance / 10.0  # Normalize assuming max distance of 10m
        
        # Velocity cost (prefer higher speeds)
        avg_velocity = np.mean([p.vx for p in trajectory])
        velocity_cost = 1.0 - (avg_velocity / self.max_velocity)
        
        # Obstacle cost
        obstacle_cost = 0.0
        if costmap is not None:
            for point in trajectory:
                gx = int((point.x - costmap_origin[0]) / costmap_resolution)
                gy = int((point.y - costmap_origin[1]) / costmap_resolution)
                
                if 0 <= gx < costmap.shape[0] and 0 <= gy < costmap.shape[1]:
                    obstacle_cost += costmap[gx, gy]
                    
        obstacle_cost = min(1.0, obstacle_cost / len(trajectory))
        
        # Total weighted cost
        total_cost = (
            self.heading_weight * heading_cost +
            self.distance_weight * distance_cost +
            self.velocity_weight * velocity_cost +
            self.obstacle_cost_scale * obstacle_cost
        )
        
        return total_cost
    
    def compute_velocity(
        self,
        x: float,
        y: float,
        theta: float,
        current_vx: float,
        current_wz: float,
        goal_x: float,
        goal_y: float,
        goal_theta: float,
        costmap: Optional[np.ndarray] = None,
        costmap_resolution: float = 0.1,
        costmap_origin: Tuple[float, float] = (0, 0)
    ) -> VelocityCommand:
        """
        Compute optimal velocity command using DWA.
        
        Args:
            x, y, theta: Current pose
            current_vx, current_wz: Current velocities
            goal_x, goal_y, goal_theta: Goal pose
            costmap: Optional obstacle costmap
            costmap_resolution: Resolution of costmap
            costmap_origin: Origin of costmap
            
        Returns:
            Optimal velocity command
        """
        # Check if goal reached
        distance_to_goal = np.sqrt((goal_x - x)**2 + (goal_y - y)**2)
        if distance_to_goal < self.goal_tolerance:
            return VelocityCommand(vx=0.0, wz=0.0)
        
        # Get dynamic window
        v_window, w_window = self.compute_dynamic_window(current_vx, current_wz)
        
        best_cost = float('inf')
        best_vx, best_wz = current_vx, 0.0
        
        # Sample velocity space
        for vx in v_window:
            for wz in w_window:
                # Predict trajectory
                trajectory = self.predict_trajectory(x, y, theta, vx, wz)
                
                # Calculate cost
                cost = self.trajectory_cost(
                    trajectory, goal_x, goal_y, goal_theta,
                    costmap, costmap_resolution, costmap_origin
                )
                
                if cost < best_cost:
                    best_cost = cost
                    best_vx, best_wz = vx, wz
        
        return VelocityCommand(vx=best_vx, wz=best_wz)


class PurePursuitController:
    """
    Pure Pursuit geometric path tracking controller.
    
    Calculates steering angle to reach a lookahead point on the path.
    """
    
    def __init__(
        self,
        wheelbase: float = 1.2,
        lookahead_distance: float = 0.6,
        min_lookahead_distance: float = 0.3,
        max_lookahead_distance: float = 0.9,
        lookahead_time: float = 1.5,
        max_angular_velocity: float = 1.0,
        max_steering_angle: float = 35.0,
        velocity_scaling: bool = True,
        approach_velocity_scaling_dist: float = 1.0,
        min_approach_velocity: float = 0.05,
    ):
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance
        self.min_lookahead_distance = min_lookahead_distance
        self.max_lookahead_distance = max_lookahead_distance
        self.lookahead_time = lookahead_time
        self.max_angular_velocity = max_angular_velocity
        self.max_steering_angle = max_steering_angle
        self.velocity_scaling = velocity_scaling
        self.approach_velocity_scaling_dist = approach_velocity_scaling_dist
        self.min_approach_velocity = min_approach_velocity
        
    def _calculate_lookahead_distance(
        self,
        current_velocity: float
    ) -> float:
        """
        Calculate lookahead distance based on velocity.
        
        Args:
            current_velocity: Current velocity in m/s
            
        Returns:
            Lookahead distance in meters
        """
        if self.velocity_scaling:
            # Time-based lookahead
            ld = abs(current_velocity) * self.lookahead_time
            
            # Clamp to limits
            return max(
                self.min_lookahead_distance,
                min(self.max_lookahead_distance, ld)
            )
        return self.lookahead_distance
    
    def _find_lookahead_point(
        self,
        path: List[Tuple[float, float]],
        current_x: float,
        current_y: float,
        lookahead_dist: float
    ) -> Optional[Tuple[float, float]]:
        """
        Find the lookahead point on the path.
        
        Args:
            path: List of (x, y) waypoints
            current_x, current_y: Current position
            lookahead_dist: Distance to lookahead
            
        Returns:
            Lookahead point (x, y) or None
        """
        if not path:
            return None
            
        # Find point on path at approximately lookahead distance
        for i, (wx, wy) in enumerate(path):
            dist = np.sqrt((wx - current_x)**2 + (wy - current_y)**2)
            if dist >= lookahead_dist:
                return (wx, wy)
                
        # If no point found, return last point
        return path[-1]
    
    def _calculate_curvature(
        self,
        current_x: float,
        current_y: float,
        current_theta: float,
        lookahead_x: float,
        lookahead_y: float
    ) -> float:
        """
        Calculate curvature to reach lookahead point.
        
        Args:
            current_x, current_y, current_theta: Current pose
            lookahead_x, lookahead_y: Lookahead point
            
        Returns:
            Curvature (1/radius)
        """
        # Transform lookahead point to vehicle frame
        dx = lookahead_x - current_x
        dy = lookahead_y - current_y
        
        # Rotate to vehicle frame
        local_x = dx * np.cos(current_theta) + dy * np.sin(current_theta)
        local_y = -dx * np.sin(current_theta) + dy * np.cos(current_theta)
        
        # Calculate curvature using geometry
        # Alpha is the angle to lookahead point in vehicle frame
        alpha = np.arctan2(local_y, local_x)
        
        # Distance to lookahead point
        ld = np.sqrt(local_x**2 + local_y**2)
        
        if ld < 1e-6:
            return 0.0
            
        # Curvature formula for Ackermann vehicle
        curvature = (2 * local_y) / (ld ** 2)
        
        return curvature
    
    def compute_command(
        self,
        current_x: float,
        current_y: float,
        current_theta: float,
        current_velocity: float,
        path: List[Tuple[float, float]],
        desired_velocity: float = 0.3
    ) -> VelocityCommand:
        """
        Compute velocity command using Pure Pursuit.
        
        Args:
            current_x, current_y, current_theta: Current pose
            current_velocity: Current velocity in m/s
            path: List of (x, y) waypoints
            desired_velocity: Desired velocity in m/s
            
        Returns:
            Velocity command
        """
        if not path or len(path) < 2:
            return VelocityCommand(vx=0.0, wz=0.0)
        
        # Calculate lookahead distance
        lookahead_dist = self._calculate_lookahead_distance(current_velocity)
        
        # Find lookahead point
        lookahead = self._find_lookahead_point(
            path, current_x, current_y, lookahead_dist
        )
        
        if lookahead is None:
            return VelocityCommand(vx=0.0, wz=0.0)
            
        # Calculate curvature
        curvature = self._calculate_curvature(
            current_x, current_y, current_theta,
            lookahead[0], lookahead[1]
        )
        
        # Convert curvature to angular velocity
        # Using bicycle model: wz = v * curvature
        angular_velocity = current_velocity * curvature
        
        # Clamp angular velocity
        angular_velocity = np.clip(
            angular_velocity,
            -self.max_angular_velocity,
            self.max_angular_velocity
        )
        
        # Calculate steering angle for display
        steering_angle = np.degrees(np.arctan2(curvature * self.wheelbase, 1.0))
        steering_angle = np.clip(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )
        
        # Scale velocity based on distance to goal
        final_waypoint = path[-1]
        dist_to_goal = np.sqrt(
            (final_waypoint[0] - current_x)**2 +
            (final_waypoint[1] - current_y)**2
        )
        
        velocity = desired_velocity
        if dist_to_goal < self.approach_velocity_scaling_dist:
            # Reduce velocity as approaching goal
            scale = max(0.1, dist_to_goal / self.approach_velocity_scaling_dist)
            velocity = max(self.min_approach_velocity, desired_velocity * scale)
        
        return VelocityCommand(
            vx=velocity,
            wz=angular_velocity,
            steering=steering_angle
        )


class LocalPlannerFactory:
    """Factory for creating local planners."""
    
    @staticmethod
    def create_local_planner(
        planner_type: str,
        **kwargs
    ):
        """
        Create a local planner of the specified type.
        
        Args:
            planner_type: Type of planner ('dwa' or 'pure_pursuit')
            **kwargs: Additional arguments
            
        Returns:
            Local planner instance
        """
        if planner_type.lower() == 'dwa':
            return DynamicWindowApproach(**kwargs)
        elif planner_type.lower() == 'pure_pursuit':
            return PurePursuitController(**kwargs)
        else:
            raise ValueError(f"Unknown local planner type: {planner_type}")