"""
Unit tests for terrain cost calculation.
"""

import pytest
import numpy as np
import sys
sys.path.insert(0, '../src/moon_rover_navigation')

from moon_rover_navigation.terrain_cost import (
    TerrainCostCalculator,
    LunarTerrainAnalyzer
)


class TestTerrainCostCalculator:
    """Tests for terrain cost calculator."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.calculator = TerrainCostCalculator()
    
    def test_elevation_cost_normal(self):
        """Test elevation cost calculation."""
        cost = self.calculator.calculate_elevation_cost(5.0, max_elevation=10.0)
        assert 0.0 <= cost <= 1.0
        assert cost == 0.5
    
    def test_elevation_cost_zero(self):
        """Test elevation cost at zero."""
        cost = self.calculator.calculate_elevation_cost(0.0)
        assert cost == 0.0
    
    def test_elevation_cost_max(self):
        """Test elevation cost at max."""
        cost = self.calculator.calculate_elevation_cost(10.0, max_elevation=10.0)
        assert cost == 1.0
    
    def test_slope_cost_zero(self):
        """Test slope cost for flat terrain."""
        cost = self.calculator.calculate_slope_cost(0.0)
        assert cost == 0.0
    
    def test_slope_cost_at_max(self):
        """Test slope cost at maximum slope."""
        cost = self.calculator.calculate_slope_cost(30.0)
        assert cost == 1.0
    
    def test_slope_cost_quadratic(self):
        """Test that slope cost follows quadratic curve."""
        cost_15 = self.calculator.calculate_slope_cost(15.0)
        cost_30 = self.calculator.calculate_slope_cost(30.0)
        assert cost_30 == 1.0
        assert 0.0 < cost_15 < 1.0
    
    def test_rock_cost_below_threshold(self):
        """Test rock cost below threshold."""
        cost = self.calculator.calculate_rock_cost(0.3)
        assert cost == 0.0
    
    def test_rock_cost_above_threshold(self):
        """Test rock cost above threshold."""
        cost = self.calculator.calculate_rock_cost(0.8)
        assert cost > 0.0
    
    def test_total_cost_range(self):
        """Test that total cost is always in [0, 1]."""
        for _ in range(100):
            elevation = np.random.uniform(0, 10)
            slope = np.random.uniform(0, 30)
            rock = np.random.uniform(0, 1)
            illum = np.random.uniform(0, 1)
            
            total = self.calculator.calculate_total_cost(
                elevation, slope, rock, illum
            )
            assert 0.0 <= total <= 1.0
    
    def test_generate_costmap_shape(self):
        """Test costmap generation output shape."""
        elevation = np.random.rand(50, 50) * 5
        slope = np.random.rand(50, 50) * 30
        rock = np.random.rand(50, 50)
        illumination = np.random.rand(50, 50)
        
        costmap = self.calculator.generate_costmap(
            elevation, slope, rock, illumination
        )
        
        assert costmap.shape == elevation.shape
        assert costmap.dtype == np.float32


class TestLunarTerrainAnalyzer:
    """Tests for lunar terrain analyzer."""
    
    def setup_method(self):
        """Set up test fixtures."""
        self.analyzer = LunarTerrainAnalyzer(resolution=1.0)
    
    def test_slope_computation_flat(self):
        """Test slope computation for flat terrain."""
        elevation = np.zeros((10, 10))
        slope = self.analyzer.compute_slope_from_elevation(elevation)
        assert np.allclose(slope, 0.0)
    
    def test_slope_computation_incline(self):
        """Test slope computation for inclined plane."""
        x = np.linspace(0, 10, 10)
        X, Y = np.meshgrid(x, x)
        elevation = X  # Linear increase in X direction
        
        slope = self.analyzer.compute_slope_from_elevation(elevation)
        # Should have non-zero slope in X direction
        assert np.any(slope > 0)
    
    def test_rock_detection_uniform(self):
        """Test rock detection on uniform terrain."""
        elevation = np.random.rand(20, 20) * 0.1  # Small variations
        rocks = self.analyzer.detect_rock_hazards(elevation)
        assert rocks.shape == elevation.shape
    
    def test_traversability_map_binary(self):
        """Test that traversability map is binary."""
        costmap = np.random.rand(20, 20)
        traversable = self.analyzer.generate_traversability_map(costmap)
        
        unique_values = np.unique(traversable)
        assert set(unique_values).issubset({0, 1})


if __name__ == '__main__':
    pytest.main([__file__, '-v'])