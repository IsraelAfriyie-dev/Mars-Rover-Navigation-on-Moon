"""
Unit tests for local controllers.
"""

import pytest
import numpy as np
import sys
sys.path.insert(0, '../src/moon_rover_navigation')

from moon_rover_navigation.local_planners import (
    DynamicWindowApproach,
    PurePursuitController,
    VelocityCommand
)


class TestDynamicWindowApproach:
    """Tests for DWA controller."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.dwa = DynamicWindowApproach(
            max_velocity=0.5,
            max_angular_velocity=1.0
        )
    
    def test_dynamic_window_shape(self):
        """Test dynamic window calculation."""
        v_window, w_window = self.dwa.compute_dynamic_window(0.0, 0.0)
        
        assert len(v_window) == self.dwa.velocity_resolution
        assert len(w_window) == self.dwa.angular_resolution
        assert np.all(v_window >= 0)  # No reverse in DWA by default
        assert np.all(w_window <= self.dwa.max_angular_velocity)
    
    def test_velocity_command_at_goal(self):
        """Test that velocity is zero when at goal."""
        cmd = self.dwa.compute_velocity(
            x=5.0, y=5.0, theta=0.0,
            current_vx=0.1, current_wz=0.0,
            goal_x=5.0, goal_y=5.0, goal_theta=0.0
        )
        
        assert cmd.vx == 0.0
        assert cmd.wz == 0.0
    
    def test_velocity_command_forward(self):
        """Test forward velocity command."""
        cmd = self.dwa.compute_velocity(
            x=0.0, y=0.0, theta=0.0,
            current_vx=0.1, current_wz=0.0,
            goal_x=10.0, goal_y=0.0, goal_theta=0.0
        )
        
        assert cmd.vx > 0  # Should move forward
    
    def test_trajectory_prediction(self):
        """Test trajectory prediction."""
        trajectory = self.dwa.predict_trajectory(
            x=0.0, y=0.0, theta=0.0,
            vx=0.5, wz=0.0
        )
        
        assert len(trajectory) > 0
        # Last point should be further than first
        final_dist = np.sqrt(trajectory[-1].x**2 + trajectory[-1].y**2)
        assert final_dist > 0


class TestPurePursuitController:
    """Tests for Pure Pursuit controller."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.pp = PurePursuitController(
            wheelbase=1.2,
            lookahead_distance=0.6
        )
    
    def test_lookahead_distance_fixed(self):
        """Test fixed lookahead distance."""
        ld = self.pp._calculate_lookahead_distance(0.0)
        assert ld == self.pp.lookahead_distance
    
    def test_lookahead_distance_velocity_scaled(self):
        """Test velocity-scaled lookahead."""
        self.pp.velocity_scaling = True
        ld1 = self.pp._calculate_lookahead_distance(0.1)
        ld2 = self.pp._calculate_lookahead_distance(1.0)
        
        assert ld2 > ld1
    
    def test_lookahead_clamping(self):
        """Test lookahead distance clamping."""
        self.pp.velocity_scaling = True
        # Very high velocity should clamp to max
        ld = self.pp._calculate_lookahead_distance(100.0)
        assert ld == self.pp.max_lookahead_distance
        
        # Very low velocity should clamp to min
        ld = self.pp._calculate_lookahead_distance(0.01)
        assert ld == self.pp.min_lookahead_distance
    
    def test_empty_path(self):
        """Test behavior with empty path."""
        cmd = self.pp.compute_command(
            current_x=0.0, current_y=0.0, current_theta=0.0,
            current_velocity=0.0,
            path=[],
            desired_velocity=0.3
        )
        
        assert cmd.vx == 0.0
        assert cmd.wz == 0.0
    
    def test_straight_path(self):
        """Test following a straight path."""
        path = [(0, 0), (1, 0), (2, 0), (3, 0)]
        
        cmd = self.pp.compute_command(
            current_x=0.0, current_y=0.0, current_theta=0.0,
            current_velocity=0.1,
            path=path,
            desired_velocity=0.3
        )
        
        assert cmd.vx > 0  # Should move forward
        assert abs(cmd.wz) < 0.1  # Minimal turning
    
    def test_curved_path(self):
        """Test following a curved path."""
        # Create a curved path
        theta = np.linspace(0, np.pi/2, 20)
        path = [(np.cos(t)*5, np.sin(t)*5) for t in theta]
        
        cmd = self.pp.compute_command(
            current_x=5.0, current_y=0.0, current_theta=0.0,
            current_velocity=0.1,
            path=path,
            desired_velocity=0.3
        )
        
        assert cmd.vx > 0
        # Should have positive angular velocity to follow curve
        assert cmd.wz > 0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])