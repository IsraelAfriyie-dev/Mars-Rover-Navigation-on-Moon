"""
Terrain Cost System for Lunar Rover Navigation.

This module implements a multi-factor terrain cost system that considers:
- Elevation cost
- Slope cost
- Rock hazard cost
- Energy consumption cost
- Illumination risk cost
"""

import numpy as np
from typing import Tuple, Optional


class TerrainCostCalculator:
    """Calculate terrain traversability costs for lunar navigation."""
    
    def __init__(
        self,
        elevation_weight: float = 1.0,
        slope_weight: float = 2.0,
        rock_weight: float = 3.0,
        energy_weight: float = 1.5,
        illumination_weight: float = 1.0,
        max_slope_degrees: float = 30.0,
        rock_threshold: float = 0.5,
    ):
        self.elevation_weight = elevation_weight
        self.slope_weight = slope_weight
        self.rock_weight = rock_weight
        self.energy_weight = energy_weight
        self.illumination_weight = illumination_weight
        self.max_slope_degrees = max_slope_degrees
        self.rock_threshold = rock_threshold
        
    def calculate_elevation_cost(self, elevation: float, max_elevation: float = 10.0) -> float:
        """
        Calculate cost based on elevation.
        
        Args:
            elevation: Current elevation in meters
            max_elevation: Maximum expected elevation for normalization
            
        Returns:
            Normalized elevation cost [0, 1]
        """
        return min(1.0, elevation / max_elevation)
    
    def calculate_slope_cost(self, slope_degrees: float) -> float:
        """
        Calculate cost based on terrain slope.
        
        Args:
            slope_degrees: Slope angle in degrees
            
        Returns:
            Normalized slope cost [0, 1]
        """
        if slope_degrees <= 0:
            return 0.0
        if slope_degrees >= self.max_slope_degrees:
            return 1.0
        # Quadratic cost function for slope
        return (slope_degrees / self.max_slope_degrees) ** 2
    
    def calculate_rock_cost(self, rock_density: float) -> float:
        """
        Calculate cost based on rock/small obstacle density.
        
        Args:
            rock_density: Normalized rock density [0, 1]
            
        Returns:
            Normalized rock cost [0, 1]
        """
        if rock_density <= self.rock_threshold:
            return 0.0
        # Exponential increase beyond threshold
        return 1.0 - np.exp(-3.0 * (rock_density - self.rock_threshold))
    
    def calculate_energy_cost(self, slope_degrees: float, distance: float = 1.0) -> float:
        """
        Calculate energy consumption cost.
        
        Args:
            slope_degrees: Slope angle affecting energy consumption
            distance: Distance traveled
            
        Returns:
            Normalized energy cost [0, 1]
        """
        # Energy increases with uphill slope
        slope_rad = np.radians(slope_degrees)
        energy_factor = 1.0 + 0.5 * np.sin(slope_rad)
        return min(1.0, (energy_factor * distance) / 2.0)
    
    def calculate_illumination_cost(self, illumination: float, shadow_factor: float = 1.0) -> float:
        """
        Calculate risk cost based on lighting conditions.
        
        Args:
            illumination: Normalized illumination [0, 1]
            shadow_factor: Factor for shadowed regions
            
        Returns:
            Normalized illumination risk cost [0, 1]
        """
        # Low illumination increases navigation risk
        base_cost = 1.0 - illumination
        # Shadows create additional uncertainty
        return min(1.0, base_cost * shadow_factor)
    
    def calculate_total_cost(
        self,
        elevation: float,
        slope_degrees: float,
        rock_density: float,
        illumination: float,
        shadow_factor: float = 1.0,
    ) -> float:
        """
        Calculate total terrain traversability cost.
        
        Args:
            elevation: Current elevation in meters
            slope_degrees: Terrain slope in degrees
            rock_density: Normalized rock density [0, 1]
            illumination: Normalized illumination [0, 1]
            shadow_factor: Additional shadow risk factor
            
        Returns:
            Total weighted cost [0, 1]
        """
        elevation_cost = self.calculate_elevation_cost(elevation)
        slope_cost = self.calculate_slope_cost(slope_degrees)
        rock_cost = self.calculate_rock_cost(rock_density)
        energy_cost = self.calculate_energy_cost(slope_degrees)
        illum_cost = self.calculate_illumination_cost(illumination, shadow_factor)
        
        total = (
            self.elevation_weight * elevation_cost +
            self.slope_weight * slope_cost +
            self.rock_weight * rock_cost +
            self.energy_weight * energy_cost +
            self.illumination_weight * illum_cost
        )
        
        max_possible = (
            self.elevation_weight +
            self.slope_weight +
            self.rock_weight +
            self.energy_weight +
            self.illumination_weight
        )
        
        return min(1.0, total / max_possible)
    
    def generate_costmap(
        self,
        elevation_grid: np.ndarray,
        slope_grid: np.ndarray,
        rock_grid: np.ndarray,
        illumination_grid: np.ndarray,
    ) -> np.ndarray:
        """
        Generate a costmap from terrain data grids.
        
        Args:
            elevation_grid: 2D array of elevation values
            slope_grid: 2D array of slope values in degrees
            rock_grid: 2D array of rock density values
            illumination_grid: 2D array of illumination values
            
        Returns:
            2D costmap with values [0, 1]
        """
        costmap = np.zeros_like(elevation_grid, dtype=np.float32)
        
        for i in range(elevation_grid.shape[0]):
            for j in range(elevation_grid.shape[1]):
                costmap[i, j] = self.calculate_total_cost(
                    elevation_grid[i, j],
                    slope_grid[i, j],
                    rock_grid[i, j],
                    illumination_grid[i, j]
                )
                
        return costmap


class LunarTerrainAnalyzer:
    """Analyze and process lunar terrain data for navigation."""
    
    def __init__(self, resolution: float = 0.1):
        self.resolution = resolution
        
    def compute_slope_from_elevation(self, elevation_grid: np.ndarray) -> np.ndarray:
        """
        Compute slope angles from elevation data using gradient.
        
        Args:
            elevation_grid: 2D elevation map
            
        Returns:
            2D array of slope angles in degrees
        """
        grad_y, grad_x = np.gradient(elevation_grid, self.resolution)
        slope_rad = np.arctan(np.sqrt(grad_x**2 + grad_y**2))
        return np.degrees(slope_rad)
    
    def detect_rock_hazards(
        self,
        elevation_grid: np.ndarray,
        threshold_std: float = 0.5
    ) -> np.ndarray:
        """
        Detect rock hazards from elevation variations.
        
        Args:
            elevation_grid: 2D elevation map
            threshold_std: Standard deviation threshold for rock detection
            
        Returns:
            2D array of rock density values [0, 1]
        """
        # Use local standard deviation to detect rough terrain
        from scipy.ndimage import uniform_filter_std
        
        local_std = uniform_filter_std(elevation_grid, size=5)
        max_std = local_std.max() if local_std.max() > 0 else 1.0
        return local_std / max_std
    
    def generate_traversability_map(
        self,
        costmap: np.ndarray,
        threshold: float = 0.7
    ) -> np.ndarray:
        """
        Generate binary traversability map from costmap.
        
        Args:
            costmap: Input costmap [0, 1]
            threshold: Maximum cost for traversable terrain
            
        Returns:
            Binary traversability map (1 = traversable, 0 = blocked)
        """
        return (costmap < threshold).astype(np.uint8)