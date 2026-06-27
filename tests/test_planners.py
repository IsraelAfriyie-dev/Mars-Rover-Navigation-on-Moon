"""
Unit tests for path planning algorithms.
"""

import pytest
import numpy as np
import sys
sys.path.insert(0, '../src/moon_rover_navigation')

from moon_rover_navigation.planners import (
    AStarPlanner,
    HybridAStarPlanner,
    RRTStarPlanner
)


class TestAStarPlanner:
    """Tests for A* planner."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.grid = np.zeros((20, 20))
        self.planner = AStarPlanner(grid_resolution=1.0)
    
    def test_simple_path(self):
        """Test basic path finding."""
        path = self.planner.plan((0, 0), (5, 5), self.grid.shape)
        assert path is not None
        assert len(path) > 0
        assert path[0] == (0, 0)
        assert path[-1] == (5, 5)
    
    def test_path_with_obstacles(self):
        """Test path with obstacles."""
        # Add obstacle line
        self.grid[5, :] = 1
        self.planner.costmap = self.grid
        
        path = self.planner.plan((0, 0), (10, 10), self.grid.shape)
        # Should either find path around or return None
        assert path is None or len(path) > 0
    
    def test_no_path_exists(self):
        """Test when no path exists."""
        # Block entire grid
        self.grid[:, :] = 1
        self.planner.costmap = self.grid
        
        path = self.planner.plan((0, 0), (10, 10), self.grid.shape)
        assert path is None


class TestHybridAStarPlanner:
    """Tests for Hybrid A* planner."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.grid = np.zeros((20, 20))
        self.planner = HybridAStarPlanner(
            grid_resolution=1.0,
            theta_resolution=36,
            min_turning_radius=1.0
        )
    
    def test_simple_path(self):
        """Test basic path finding."""
        path = self.planner.plan(
            (0, 0, 0),
            (5, 5, 0),
            self.grid.shape
        )
        assert path is not None
        assert len(path) > 0
    
    def test_path_includes_theta(self):
        """Test that path includes heading information."""
        path = self.planner.plan(
            (0, 0, 0),
            (5, 5, np.pi/4),
            self.grid.shape
        )
        assert path is not None
        # Each point should have 3 values (x, y, theta)
        for point in path:
            assert len(point) == 3


class TestRRTStarPlanner:
    """Tests for RRT* planner."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.planner = RRTStarPlanner(
            bounds=(0, 20, 0, 20),
            max_iterations=500,
            step_size=1.0
        )
    
    def test_simple_path(self):
        """Test basic path finding."""
        path = self.planner.plan((2, 2), (18, 18))
        assert path is not None
        assert len(path) > 0
    
    def test_path_respects_bounds(self):
        """Test that path stays within bounds."""
        path = self.planner.plan((2, 2), (18, 18))
        if path:
            for point in path:
                assert 0 <= point[0] <= 20
                assert 0 <= point[1] <= 20


if __name__ == '__main__':
    pytest.main([__file__, '-v'])