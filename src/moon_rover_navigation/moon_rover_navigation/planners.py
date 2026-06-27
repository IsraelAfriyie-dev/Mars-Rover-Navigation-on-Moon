"""
Global Path Planning Algorithms for Lunar Rover Navigation.

Implements:
- A* Algorithm (baseline grid-based planner)
- Hybrid A* (for non-holonomic rover motion)
- RRT* (for exploration and complex terrain)
"""

import numpy as np
from typing import Tuple, List, Optional, Callable, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
import heapq


class PlannerType(Enum):
    """Types of global path planners."""
    ASTAR = "astar"
    HYBRID_ASTAR = "hybrid_astar"
    RRT_STAR = "rrt_star"


@dataclass
class Node:
    """Node for path planning algorithms."""
    x: float
    y: float
    theta: float = 0.0  # For hybrid planners
    g_cost: float = float('inf')
    h_cost: float = 0.0
    parent: Optional['Node'] = None
    cost: float = 0.0  # Total cost (g + h)
    
    def __lt__(self, other: 'Node') -> bool:
        return self.cost < other.cost


@dataclass(order=True)
class RRTNode:
    """Node for RRT* algorithm."""
    cost: float = field(compare=True)
    x: float = field(compare=False)
    y: float = field(compare=False)
    theta: float = field(compare=False)
    parent: Optional['RRTNode'] = field(compare=False, default=None)


class AStarPlanner:
    """A* grid-based path planner (baseline algorithm)."""
    
    def __init__(
        self,
        grid_resolution: float = 0.1,
        costmap: Optional[np.ndarray] = None,
        heuristic: str = "euclidean"
    ):
        self.grid_resolution = grid_resolution
        self.costmap = costmap
        self.heuristic = heuristic
        
    def heuristic_euclidean(
        self,
        current: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> float:
        """Euclidean distance heuristic."""
        dx = goal[0] - current[0]
        dy = goal[1] - current[1]
        return np.sqrt(dx**2 + dy**2) * self.grid_resolution
    
    def heuristic_manhattan(
        self,
        current: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> float:
        """Manhattan distance heuristic."""
        dx = abs(goal[0] - current[0])
        dy = abs(goal[1] - current[1])
        return (dx + dy) * self.grid_resolution
    
    def heuristic_diagonal(
        self,
        current: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> float:
        """Diagonal distance heuristic."""
        dx = abs(goal[0] - current[0])
        dy = abs(goal[1] - current[1])
        return (dx + dy + (np.sqrt(2) - 2) * min(dx, dy)) * self.grid_resolution
    
    def get_heuristic(self, current: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Get heuristic value based on selected method."""
        if self.heuristic == "euclidean":
            return self.heuristic_euclidean(current, goal)
        elif self.heuristic == "manhattan":
            return self.heuristic_manhattan(current, goal)
        elif self.heuristic == "diagonal":
            return self.heuristic_diagonal(current, goal)
        return self.heuristic_euclidean(current, goal)
    
    def get_neighbors(
        self,
        node: Tuple[int, int],
        grid_shape: Tuple[int, int]
    ) -> List[Tuple[Tuple[int, int], float]]:
        """Get valid neighboring cells and their movement costs."""
        neighbors = []
        directions = [
            (-1, 0, 1.0),   # Up
            (1, 0, 1.0),    # Down
            (0, -1, 1.0),   # Left
            (0, 1, 1.0),    # Right
            (-1, -1, np.sqrt(2)),   # Diagonal up-left
            (-1, 1, np.sqrt(2)),    # Diagonal up-right
            (1, -1, np.sqrt(2)),    # Diagonal down-left
            (1, 1, np.sqrt(2)),     # Diagonal down-right
        ]
        
        for dx, dy, move_cost in directions:
            nx, ny = node[0] + dx, node[1] + dy
            if 0 <= nx < grid_shape[0] and 0 <= ny < grid_shape[1]:
                # Check if cell is traversable
                if self.costmap is None or self.costmap[nx, ny] < 0.7:
                    neighbors.append(((nx, ny), move_cost))
                    
        return neighbors
    
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        grid_shape: Tuple[int, int]
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path using A* algorithm.
        
        Args:
            start: Start position (x, y) in meters
            goal: Goal position (x, y) in meters
            grid_shape: Shape of the grid (rows, cols)
            
        Returns:
            List of (x, y) waypoints or None if no path found
        """
        # Convert to grid coordinates
        start_grid = (int(start[0] / self.grid_resolution),
                      int(start[1] / self.grid_resolution))
        goal_grid = (int(goal[0] / self.grid_resolution),
                     int(goal[1] / self.grid_resolution))
        
        # Ensure within bounds
        start_grid = (
            max(0, min(grid_shape[0] - 1, start_grid[0])),
            max(0, min(grid_shape[1] - 1, start_grid[1]))
        )
        goal_grid = (
            max(0, min(grid_shape[0] - 1, goal_grid[0])),
            max(0, min(grid_shape[1] - 1, goal_grid[1]))
        )
        
        open_set = []
        closed_set = set()
        g_scores = {start_grid: 0}
        parent_map = {}
        
        h = self.get_heuristic(start_grid, goal_grid)
        heapq.heappush(open_set, (h, start_grid))
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == goal_grid:
                return self._reconstruct_path(parent_map, start_grid, goal_grid)
            
            if current in closed_set:
                continue
            closed_set.add(current)
            
            for neighbor, move_cost in self.get_neighbors(current, grid_shape):
                if neighbor in closed_set:
                    continue
                    
                # Get terrain cost if available
                terrain_cost = 1.0
                if self.costmap is not None:
                    terrain_cost = 1.0 + self.costmap[neighbor]
                
                tentative_g = g_scores[current] + move_cost * terrain_cost
                
                if neighbor not in g_scores or tentative_g < g_scores[neighbor]:
                    g_scores[neighbor] = tentative_g
                    parent_map[neighbor] = current
                    h = self.get_heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (tentative_g + h, neighbor))
                    
        return None
    
    def _reconstruct_path(
        self,
        parent_map: Dict,
        start: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> List[Tuple[float, float]]:
        """Reconstruct path from parent map."""
        path = []
        current = goal
        while current != start:
            path.append((
                current[0] * self.grid_resolution,
                current[1] * self.grid_resolution
            ))
            current = parent_map[current]
        path.append((
            start[0] * self.grid_resolution,
            start[1] * self.grid_resolution
        ))
        path.reverse()
        return path


class HybridAStarPlanner:
    """
    Hybrid A* planner for non-holonomic lunar rover motion.
    
    Considers vehicle kinematics with Ackermann steering and
    produces smooth, drivable paths.
    """
    
    def __init__(
        self,
        grid_resolution: float = 0.2,
        theta_resolution: int = 36,
        min_turning_radius: float = 0.5,
        costmap: Optional[np.ndarray] = None
    ):
        self.grid_resolution = grid_resolution
        self.theta_resolution = theta_resolution
        self.min_turning_radius = min_turning_radius
        self.costmap = costmap
        self.dtheta = 2 * np.pi / theta_resolution
        
        # Motion primitives for hybrid A*
        self._init_motion_primitives()
        
    def _init_motion_primitives(self):
        """Initialize motion primitives for the vehicle."""
        # steering angles (degrees)
        steering_angles = [-35, -20, 0, 20, 35]
        # step sizes
        step_size = self.grid_resolution
        
        self.motion_primitives = []
        for steer in steering_angles:
            # Forward motion
            self.motion_primitives.append({
                'steering': steer,
                'velocity': 1,
                'step': step_size
            })
            # Reverse motion
            self.motion_primitives.append({
                'steering': steer,
                'velocity': -1,
                'step': step_size * 0.5  # Slower in reverse
            })
    
    def _state_to_grid(
        self,
        x: float,
        y: float,
        theta: float,
        grid_shape: Tuple[int, int]
    ) -> Tuple[int, int, int]:
        """Convert continuous state to grid indices."""
        gx = int(x / self.grid_resolution) % grid_shape[0]
        gy = int(y / self.grid_resolution) % grid_shape[1]
        gtheta = int((theta % (2 * np.pi)) / self.dtheta) % self.theta_resolution
        return gx, gy, gtheta
    
    def _compute_dynamics(
        self,
        x: float,
        y: float,
        theta: float,
        steering: float,
        velocity: float,
        dt: float = 0.1
    ) -> Tuple[float, float, float]:
        """Compute next state using Ackermann kinematics."""
        wheelbase = 1.2  # meters
        
        if abs(np.degrees(steering)) < 0.1:
            # Straight line
            x_new = x + velocity * np.cos(theta) * dt
            y_new = y + velocity * np.sin(theta) * dt
            theta_new = theta
        else:
            # Curved path
            steer_rad = np.radians(steering)
            turn_radius = wheelbase / np.tan(steer_rad)
            omega = velocity / turn_radius
            
            x_new = x + turn_radius * (np.sin(theta + omega * dt) - np.sin(theta))
            y_new = y - turn_radius * (np.cos(theta + omega * dt) - np.cos(theta))
            theta_new = theta + omega * dt
            
        return x_new, y_new, theta_new
    
    def _is_valid_state(
        self,
        x: float,
        y: float,
        grid_shape: Tuple[int, int]
    ) -> bool:
        """Check if state is within bounds and not colliding."""
        if x < 0 or x >= grid_shape[0] * self.grid_resolution:
            return False
        if y < 0 or y >= grid_shape[1] * self.grid_resolution:
            return False
            
        if self.costmap is not None:
            gx = int(x / self.grid_resolution)
            gy = int(y / self.grid_resolution)
            if self.costmap[gx, gy] >= 0.7:
                return False
                
        return True
    
    def plan(
        self,
        start: Tuple[float, float, float],
        goal: Tuple[float, float, float],
        grid_shape: Tuple[int, int],
        max_iterations: int = 10000
    ) -> Optional[List[Tuple[float, float, float]]]:
        """
        Plan a path using Hybrid A*.
        
        Args:
            start: Start pose (x, y, theta) in meters and radians
            goal: Goal pose (x, y, theta) in meters and radians
            grid_shape: Shape of the grid (rows, cols)
            max_iterations: Maximum search iterations
            
        Returns:
            List of (x, y, theta) waypoints or None if no path found
        """
        # Initialize
        start_node = Node(start[0], start[1], start[2])
        start_node.g_cost = 0
        start_node.h_cost = self._heuristic(start, goal)
        start_node.cost = start_node.g_cost + start_node.h_cost
        
        start_grid = self._state_to_grid(
            start[0], start[1], start[2], grid_shape
        )
        
        open_set = []
        heapq.heappush(open_set, start_node)
        closed_set = set()
        parent_map = {}
        
        iteration = 0
        
        while open_set and iteration < max_iterations:
            iteration += 1
            current = heapq.heappop(open_set)
            
            current_grid = self._state_to_grid(
                current.x, current.y, current.theta, grid_shape
            )
            
            if current_grid in closed_set:
                continue
            closed_set.add(current_grid)
            
            # Check if goal reached
            if (abs(current.x - goal[0]) < self.grid_resolution * 2 and
                abs(current.y - goal[1]) < self.grid_resolution * 2):
                return self._reconstruct_path_hybrid(
                    parent_map, start_node, current, goal
                )
            
            # Expand motion primitives
            for prim in self.motion_primitives:
                dt = prim['step'] / max(0.1, abs(prim['velocity']))
                x_new, y_new, theta_new = self._compute_dynamics(
                    current.x, current.y, current.theta,
                    prim['steering'], prim['velocity'], dt
                )
                
                if not self._is_valid_state(x_new, y_new, grid_shape):
                    continue
                
                new_grid = self._state_to_grid(x_new, y_new, theta_new, grid_shape)
                if new_grid in closed_set:
                    continue
                
                # Cost calculation
                terrain_cost = 1.0
                if self.costmap is not None:
                    gx = int(x_new / self.grid_resolution)
                    gy = int(y_new / self.grid_resolution)
                    terrain_cost = 1.0 + self.costmap[gx, gy]
                
                # Steering change penalty
                steer_change = abs(prim['steering'] - 
                    (np.degrees(current.theta) % 360 - 180) if hasattr(current, 'last_steer') else 0)
                steer_penalty = 1.0 + steer_change * 0.01
                
                # Reverse penalty
                reverse_penalty = 2.0 if prim['velocity'] < 0 else 1.0
                
                move_cost = prim['step'] * terrain_cost * steer_penalty * reverse_penalty
                g_new = current.g_cost + move_cost
                
                new_node = Node(x_new, y_new, theta_new)
                new_node.g_cost = g_new
                new_node.h_cost = self._heuristic((x_new, y_new, theta_new), goal)
                new_node.cost = g_new + new_node.h_cost
                new_node.parent = current
                new_node.last_steer = prim['steering']
                
                heapq.heappush(open_set, new_node)
                parent_map[(x_new, y_new, theta_new)] = current
                
        return None
    
    def _heuristic(self, state: Tuple, goal: Tuple) -> float:
        """Compute heuristic for hybrid A*."""
        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        dist = np.sqrt(dx**2 + dy**2)
        
        # Minimum turning radius constraint
        if dist > 0:
            min_turns = dist / self.min_turning_radius
            return dist + min_turns * 0.1
            
        return dist
    
    def _reconstruct_path_hybrid(
        self,
        parent_map: Dict,
        start: Node,
        end: Node,
        goal: Tuple
    ) -> List[Tuple[float, float, float]]:
        """Reconstruct hybrid A* path."""
        path = []
        current = end
        
        while current is not None and current != start:
            path.append((current.x, current.y, current.theta))
            current = parent_map.get((current.x, current.y, current.theta))
            
        path.append((start.x, start.y, start.theta))
        path.reverse()
        
        # Add goal orientation
        if path[-1] != goal:
            path.append(goal)
            
        return path


class RRTStarPlanner:
    """
    RRT* (Rapidly-exploring Random Tree Star) planner.
    
    Optimal sampling-based planner for exploration and complex terrain.
    """
    
    def __init__(
        self,
        bounds: Tuple[float, float, float, float],
        min_turning_radius: float = 0.5,
        max_iterations: int = 1000,
        step_size: float = 0.3,
        goal_sample_rate: float = 0.1,
        rewiring_radius: float = 0.5,
        costmap: Optional[np.ndarray] = None
    ):
        self.bounds = bounds  # (x_min, x_max, y_min, y_max)
        self.min_turning_radius = min_turning_radius
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.rewiring_radius = rewiring_radius
        self.costmap = costmap
        
    def _sample_random_state(self, goal: Tuple[float, float]) -> RRTNode:
        """Sample a random state in the configuration space."""
        if np.random.random() < self.goal_sample_rate:
            x, y = goal[0], goal[1]
        else:
            x = np.random.uniform(self.bounds[0], self.bounds[1])
            y = np.random.uniform(self.bounds[2], self.bounds[3])
            
        theta = np.random.uniform(-np.pi, np.pi)
        
        return RRTNode(cost=0.0, x=x, y=y, theta=theta)
    
    def _distance(self, n1: RRTNode, n2: RRTNode) -> float:
        """Compute distance between two nodes."""
        return np.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)
    
    def _steer(
        self,
        from_node: RRTNode,
        to_sample: RRTNode
    ) -> RRTNode:
        """Steer from a node towards a sample."""
        dx = to_sample.x - from_node.x
        dy = to_sample.y - from_node.y
        dist = np.sqrt(dx**2 + dy**2)
        
        if dist < self.step_size:
            new_node = RRTNode(cost=0.0, x=to_sample.x, y=to_sample.y, theta=to_sample.theta)
        else:
            # Interpolate towards sample
            ratio = self.step_size / dist
            new_x = from_node.x + dx * ratio
            new_y = from_node.y + dy * ratio
            new_theta = np.arctan2(dy, dx)
            new_node = RRTNode(cost=0.0, x=new_x, y=new_y, theta=new_theta)
            
        return new_node
    
    def _is_collision_free(
        self,
        from_node: RRTNode,
        to_node: RRTNode
    ) -> bool:
        """Check if edge between nodes is collision-free."""
        dist = self._distance(from_node, to_node)
        num_checks = int(dist / 0.05) + 1
        
        for i in range(num_checks + 1):
            t = i / num_checks
            x = from_node.x + t * (to_node.x - from_node.x)
            y = from_node.y + t * (to_node.y - from_node.y)
            
            # Check costmap
            if self.costmap is not None:
                # Convert to costmap coordinates
                gx = int((x - self.bounds[0]) / (self.bounds[1] - self.bounds[0]) * self.costmap.shape[0])
                gy = int((y - self.bounds[2]) / (self.bounds[3] - self.bounds[2]) * self.costmap.shape[1])
                
                if 0 <= gx < self.costmap.shape[0] and 0 <= gy < self.costmap.shape[1]:
                    if self.costmap[gx, gy] >= 0.7:
                        return False
                        
        return True
    
    def _nearest_node(
        self,
        tree: List[RRTNode],
        sample: RRTNode
    ) -> RRTNode:
        """Find nearest node in tree to sample."""
        return min(tree, key=lambda n: self._distance(n, sample))
    
    def _near_nodes(
        self,
        tree: List[RRTNode],
        node: RRTNode
    ) -> List[RRTNode]:
        """Find nodes within rewiring radius."""
        return [n for n in tree if self._distance(n, node) <= self.rewiring_radius]
    
    def _cost(self, node: RRTNode) -> float:
        """Get cost to reach node from root."""
        cost = 0.0
        current = node
        while current.parent is not None:
            cost += self._distance(current, current.parent)
            current = current.parent
        return cost
    
    def plan(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        goal_tolerance: float = 0.5
    ) -> Optional[List[Tuple[float, float]]]:
        """
        Plan a path using RRT*.
        
        Args:
            start: Start position (x, y)
            goal: Goal position (x, y)
            goal_tolerance: Distance threshold to consider goal reached
            
        Returns:
            List of (x, y) waypoints or None if no path found
        """
        # Initialize tree with start node
        start_node = RRTNode(cost=0.0, x=start[0], y=start[1], theta=0.0)
        tree = [start_node]
        
        for iteration in range(self.max_iterations):
            # Sample
            sample = self._sample_random_state(goal)
            
            # Find nearest
            nearest = self._nearest_node(tree, sample)
            
            # Steer
            new_node = self._steer(nearest, sample)
            
            # Check collision
            if not self._is_collision_free(nearest, new_node):
                continue
            
            # Find near nodes for rewiring
            near_nodes = self._near_nodes(tree, new_node)
            
            # Find minimum cost parent
            min_cost = float('inf')
            best_parent = nearest
            for near in near_nodes:
                if self._is_collision_free(near, new_node):
                    cost = self._cost(near) + self._distance(near, new_node)
                    if cost < min_cost:
                        min_cost = cost
                        best_parent = near
            
            new_node.parent = best_parent
            new_node.cost = min_cost
            
            # Add to tree
            tree.append(new_node)
            
            # Rewire
            for near in near_nodes:
                if near == best_parent:
                    continue
                new_cost = new_node.cost + self._distance(new_node, near)
                if new_cost < near.cost and self._is_collision_free(new_node, near):
                    near.parent = new_node
            
            # Check goal
            if self._distance(new_node, RRTNode(cost=0.0, x=goal[0], y=goal[1], theta=0.0)) < goal_tolerance:
                return self._reconstruct_path(new_node)
                
        return None
    
    def _reconstruct_path(self, goal_node: RRTNode) -> List[Tuple[float, float]]:
        """Reconstruct path from goal node to start."""
        path = []
        current = goal_node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path


class PlannerFactory:
    """Factory for creating path planners."""
    
    @staticmethod
    def create_planner(
        planner_type: PlannerType,
        **kwargs
    ) -> Any:
        """
        Create a path planner of the specified type.
        
        Args:
            planner_type: Type of planner to create
            **kwargs: Additional arguments for the planner
            
        Returns:
            Planner instance
        """
        if planner_type == PlannerType.ASTAR:
            return AStarPlanner(**kwargs)
        elif planner_type == PlannerType.HYBRID_ASTAR:
            return HybridAStarPlanner(**kwargs)
        elif planner_type == PlannerType.RRT_STAR:
            return RRTStarPlanner(**kwargs)
        else:
            raise ValueError(f"Unknown planner type: {planner_type}")
    
    @staticmethod
    def get_available_planners() -> List[PlannerType]:
        """Get list of available planner types."""
        return list(PlannerType)