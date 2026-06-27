"""
Rover Constraints Module for Lunar Navigation.

Defines physical and kinematic constraints for the lunar rover:
- Ackermann steering geometry
- Minimum turning radius
- Maximum slope constraints
- Terrain traversability limits
"""

import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class RoverConstraints:
    """Physical and kinematic constraints for the lunar rover."""
    
    # Ackermann Steering Parameters
    wheelbase: float = 1.2  # meters - distance between front and rear axles
    track_width: float = 0.8  # meters - distance between left and right wheels
    wheel_radius: float = 0.15  # meters - wheel radius
    
    # Kinematic Constraints
    min_turning_radius: float = 0.5  # meters - minimum turning radius
    max_steering_angle: float = 35.0  # degrees - maximum steering angle
    
    # Dynamic Constraints
    max_velocity: float = 0.5  # m/s - maximum forward velocity
    max_angular_velocity: float = 1.0  # rad/s - maximum angular velocity
    max_acceleration: float = 0.3  # m/s^2 - maximum acceleration
    max_deceleration: float = 0.5  # m/s^2 - maximum deceleration
    
    # Terrain Constraints
    max_slope: float = 30.0  # degrees - maximum traversable slope
    max_lateral_slope: float = 25.0  # degrees - maximum lateral slope
    max_step_height: float = 0.2  # meters - maximum obstacle height
    
    # Energy Constraints
    max_power_consumption: float = 100.0  # watts
    min_battery_level: float = 0.2  # 20% minimum battery
    
    def get_steering_angle_from_turn_radius(self, turn_radius: float) -> float:
        """
        Calculate steering angle required for a given turn radius.
        
        Args:
            turn_radius: Desired turn radius in meters
            
        Returns:
            Required steering angle in degrees
        """
        if turn_radius < self.min_turning_radius:
            turn_radius = self.min_turning_radius
            
        # Ackermann steering equation: tan(steering_angle) = wheelbase / turn_radius
        steering_rad = np.arctan(self.wheelbase / turn_radius)
        steering_deg = np.degrees(steering_rad)
        
        return min(steering_deg, self.max_steering_angle)
    
    def get_turn_radius_from_curvature(self, curvature: float) -> float:
        """
        Calculate turn radius from curvature.
        
        Args:
            curvature: Path curvature (1/radius)
            
        Returns:
            Turn radius in meters
        """
        if abs(curvature) < 1e-6:
            return float('inf')
        return 1.0 / abs(curvature)
    
    def is_traversable_slope(self, slope: float, lateral_slope: float = 0.0) -> bool:
        """
        Check if a slope is traversable.
        
        Args:
            slope: Forward/backward slope in degrees
            lateral_slope: Side slope in degrees
            
        Returns:
            True if traversable
        """
        return abs(slope) <= self.max_slope and abs(lateral_slope) <= self.max_lateral_slope
    
    def get_max_velocity_for_slope(self, slope: float) -> float:
        """
        Calculate maximum safe velocity for given slope.
        
        Args:
            slope: Terrain slope in degrees
            
        Returns:
            Maximum safe velocity in m/s
        """
        slope_factor = 1.0 - (abs(slope) / self.max_slope) * 0.5
        return self.max_velocity * max(0.3, slope_factor)
    
    def get_kinematic_limits(self, velocity: float) -> Tuple[float, float]:
        """
        Calculate kinematic limits based on current velocity.
        
        Args:
            velocity: Current velocity in m/s
            
        Returns:
            Tuple of (max_angular_velocity, max_acceleration)
        """
        # Angular velocity decreases at higher speeds
        speed_factor = 1.0 - (velocity / self.max_velocity) * 0.3
        max_angular = self.max_angular_velocity * speed_factor
        
        # Acceleration limits based on velocity
        if velocity > 0:
            max_accel = min(self.max_acceleration, velocity * 2.0)
        else:
            max_accel = self.max_acceleration
            
        return max_angular, max_accel


class AckermannKinematics:
    """Ackermann steering kinematics calculator."""
    
    def __init__(self, constraints: Optional[RoverConstraints] = None):
        self.constraints = constraints or RoverConstraints()
        
    def calculate_turn_radius(
        self,
        velocity: float,
        angular_velocity: float
    ) -> float:
        """
        Calculate turn radius from velocity and angular velocity.
        
        Args:
            velocity: Forward velocity in m/s
            angular_velocity: Angular velocity in rad/s
            
        Returns:
            Turn radius in meters
        """
        if abs(angular_velocity) < 1e-6:
            return float('inf')
        return abs(velocity / angular_velocity)
    
    def calculate_angular_velocity(
        self,
        velocity: float,
        turn_radius: float
    ) -> float:
        """
        Calculate angular velocity from velocity and turn radius.
        
        Args:
            velocity: Forward velocity in m/s
            turn_radius: Turn radius in meters
            
        Returns:
            Angular velocity in rad/s
        """
        if turn_radius < self.constraints.min_turning_radius:
            turn_radius = self.constraints.min_turning_radius
        return velocity / turn_radius
    
    def get_wheel_velocities(
        self,
        center_velocity: float,
        steering_angle: float
    ) -> Tuple[float, float]:
        """
        Calculate individual wheel velocities for Ackermann steering.
        
        Args:
            center_velocity: Velocity at vehicle center
            steering_angle: Front wheel steering angle in degrees
            
        Returns:
            Tuple of (left_wheel_velocity, right_wheel_velocity)
        """
        steering_rad = np.radians(steering_angle)
        half_track = self.constraints.track_width / 2.0
        
        # Inner wheel follows smaller radius, outer wheel follows larger radius
        inner_radius = self.constraints.min_turning_radius
        outer_radius = inner_radius + half_track * 2
        
        if abs(steering_angle) < 1e-6:
            return center_velocity, center_velocity
            
        # Calculate angular velocity about ICC (Instantaneous Center of Curvature)
        angular_vel = center_velocity * np.tan(steering_rad) / self.constraints.wheelbase
        
        left_vel = angular_vel * (inner_radius - half_track)
        right_vel = angular_vel * (outer_radius + half_track)
        
        return left_vel, right_vel
    
    def discretize_dynamics(
        self,
        x: float,
        y: float,
        theta: float,
        velocity: float,
        steering_angle: float,
        dt: float
    ) -> Tuple[float, float, float]:
        """
        Compute discrete-time Ackermann vehicle dynamics.
        
        Args:
            x, y, theta: Current pose (x, y in meters, theta in radians)
            velocity: Forward velocity in m/s
            steering_angle: Steering angle in degrees
            dt: Time step in seconds
            
        Returns:
            Tuple of (new_x, new_y, new_theta)
        """
        steering_rad = np.radians(steering_angle)
        
        # Ensure minimum turn radius
        if abs(steering_rad) < 1e-6:
            # Going straight
            new_x = x + velocity * np.cos(theta) * dt
            new_y = y + velocity * np.sin(theta) * dt
            new_theta = theta
        else:
            # Turning
            wheelbase = self.constraints.wheelbase
            turn_radius = wheelbase / np.tan(steering_rad)
            angular_vel = velocity / turn_radius
            
            new_theta = theta + angular_vel * dt
            new_x = x + turn_radius * (np.sin(new_theta) - np.sin(theta))
            new_y = y - turn_radius * (np.cos(new_theta) - np.cos(theta))
            
        return new_x, new_y, new_theta