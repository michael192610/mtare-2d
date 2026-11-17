"""Grid geometry and abstract reveal model; no sensors or estimation.

Scene sampling is adapted from RobustFieldAutonomyLab/Multi-Robot-EM-Exploration,
revision e17bf2444b48b89a5f776abccb6e71e3837d599b, MarineNavEnv2.reset().
"""
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from .config import Config
from .regions import Regions

UNKNOWN, FREE, BLOCKED = -1, 0, 1


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    radius: float = 1.0


class World:
    def __init__(self, config: Config):
        self.config = config
        self.cols = round(config.width / config.cell_size)
        self.rows = round(config.height / config.cell_size)
        self.size = self.rows * self.cols
        self.circles, self.starts, self.original_headings, self.original_goals = self._scene()
        self.neighbors = tuple(tuple(self._neighbor_ids(i)) for i in range(self.size))
        self.truth = self._rasterize()
        for position in self.starts:
            if self.truth.ravel()[self.cell_at(position)] != FREE:
                raise ValueError("A starting position rasterized as an obstacle.")
        self.centres = np.array([self.xy(i) for i in range(self.size)])
        self.regions = Regions(self.rows, self.cols, config.cell_size, config.region_size)
        from .viewpoints import ViewpointLattice
        self.viewpoints = ViewpointLattice(self)
        # Only the evaluator uses this truth-derived denominator, not the planners.
        evaluator = GridGraph(self, self.truth)
        self.reachable_truth = np.zeros(self.size, dtype=bool)
        for start in self.starts:
            self.reachable_truth |= evaluator.distances(self.cell_at(start)) >= 0

    def _scene(self):
        cfg = self.config
        rng = np.random.RandomState(cfg.seed)
        starts, goals, headings = [], [], []
        centre = np.array([min(20.0, cfg.width / 2), cfg.height / 2])
        spread = min(5.0, cfg.width / 5, cfg.height / 5)
        for _ in range(10000):
            start = rng.uniform(centre - spread, centre + spread)
            goal = rng.uniform(start - np.array([4.0, 4.0]), start)
            if np.linalg.norm(goal - start) < 5:
                continue
            if any(np.linalg.norm(start - old) <= 2 for old in starts):
                continue
            if any(np.linalg.norm(goal - old) <= 2 for old in goals):
                continue
            starts.append(start)
            goals.append(goal)
            headings.append(float(rng.uniform(0, 2 * np.pi)))
            if len(starts) == cfg.robots:
                break
        if len(starts) != cfg.robots:
            raise ValueError("Could not place the requested robots.")
        circles = []
        for _ in range(max(500, cfg.obstacles * 50)):
            if len(circles) == cfg.obstacles:
                break
            point = rng.uniform([0, 0], [cfg.width, cfg.height])
            radius = float(rng.uniform(1, 1))  # Preserve the upstream RNG sequence.
            x, y = point
            if x-radius < 0 or x+radius > cfg.width or y-radius < 0 or y+radius > cfg.height:
                continue
            if any(np.linalg.norm(point - p) < radius + 2 for p in starts + goals):
                continue
            clearance = min(cfg.width, cfg.height) / 10
            if any(math.hypot(x-o.x, y-o.y) <= radius+o.radius+clearance for o in circles):
                continue
            circles.append(Circle(float(x), float(y), radius))
        if len(circles) != cfg.obstacles:
            raise ValueError("Could not place all obstacles; reduce their count or enlarge the world.")
        return circles, [tuple(map(float, p)) for p in starts], headings, goals

    def _rasterize(self):
        truth = np.full((self.rows, self.cols), FREE, dtype=np.int8)
        size = self.config.cell_size
        # A cell is blocked if its square intersects an obstacle circle.
        for circle in self.circles:
            for row in range(max(0, int((circle.y-circle.radius)//size)),
                             min(self.rows-1, int((circle.y+circle.radius)//size))+1):
                for col in range(max(0, int((circle.x-circle.radius)//size)),
                                 min(self.cols-1, int((circle.x+circle.radius)//size))+1):
                    dx = max(col*size-circle.x, 0, circle.x-(col+1)*size)
                    dy = max(row*size-circle.y, 0, circle.y-(row+1)*size)
                    if dx*dx + dy*dy <= circle.radius*circle.radius:
                        truth[row, col] = BLOCKED
        return truth

    def _neighbor_ids(self, cell):
        row, col = divmod(cell, self.cols)
        if row:
            yield cell-self.cols
        if col:
            yield cell-1
        if col+1 < self.cols:
            yield cell+1
        if row+1 < self.rows:
            yield cell+self.cols

    def cell_at(self, position):
        col = min(self.cols-1, max(0, int(position[0] // self.config.cell_size)))
        row = min(self.rows-1, max(0, int(position[1] // self.config.cell_size)))
        return row*self.cols + col

    def region_of(self, node):
        if self.viewpoints.contains(node):
            return int(self.viewpoints.region_ids[int(node)-self.viewpoints.offset])
        return int(self.regions.cell_region[int(node)])

    def xy(self, cell):
        if hasattr(self,"viewpoints") and self.viewpoints.contains(cell):
            return self.viewpoints.xy(cell)
        if int(cell) >= self.size:
            return self.regions.centre(self.regions.region(cell))
        row, col = divmod(int(cell), self.cols)
        return ((col+0.5)*self.config.cell_size, (row+0.5)*self.config.cell_size)

    def reveal(self, known, position):
        delta = self.centres - np.asarray(position)
        visible = np.sum(delta*delta, axis=1) <= self.config.reveal_radius**2
        visible[self.cell_at(position)] = True
        indices = np.flatnonzero(visible & (known.ravel() == UNKNOWN))
        known.ravel()[indices] = self.truth.ravel()[indices]
        return indices.tolist()

    def frontier_ids(self, known):
        unknown = known == UNKNOWN
        adjacent = np.zeros_like(unknown)
        adjacent[:-1] |= unknown[1:]
        adjacent[1:] |= unknown[:-1]
        adjacent[:, :-1] |= unknown[:, 1:]
        adjacent[:, 1:] |= unknown[:, :-1]
        return np.flatnonzero(((known == FREE) & adjacent).ravel()).tolist()

    def targets(self, known, depots):
        graph = GridGraph(self, known)
        reachable = np.zeros(self.size, dtype=bool)
        for depot in depots:
            reachable |= graph.distances(depot) >= 0
        frontiers = [c for c in self.frontier_ids(known) if c not in depots and reachable[c]]
        selected = []
        spacing_squared = self.config.target_spacing**2
        for cell in frontiers:
            x, y = self.xy(cell)
            if all((x-self.xy(other)[0])**2+(y-self.xy(other)[1])**2 >= spacing_squared
                   for other in selected):
                selected.append(cell)
                if self.config.max_targets and len(selected) >= self.config.max_targets:
                    break
        return frontiers, selected

    def scene_dict(self):
        return dict(width=self.config.width, height=self.config.height, cell_size=self.config.cell_size,
                    architecture=self.config.architecture, region_size=self.config.region_size,
                    region_rows=self.regions.rows, region_cols=self.regions.cols,
                    rows=self.rows, cols=self.cols, seed=self.config.seed, starts=self.starts,
                    viewpoint_geometry=self.viewpoints.metadata(),
                    original_headings=self.original_headings,
                    circles=[vars(c) for c in self.circles],
                    note="Same seeded scene sampler as the EM demo; movement uses a four-neighbor raster grid.")


class GridGraph:
    """Shortest paths use only the map supplied here, never World.truth."""
    def __init__(self, world, known):
        self.world = world
        self.free = (known.ravel() == FREE)

    def distances(self, start):
        distances = np.full(self.world.size, -1, dtype=np.int32)
        if not self.free[start]:
            return distances
        distances[start] = 0
        queue = deque([start])
        while queue:
            cell = queue.popleft()
            next_distance = int(distances[cell])+1
            for other in self.world.neighbors[cell]:
                if self.free[other] and distances[other] < 0:
                    distances[other] = next_distance
                    queue.append(other)
        return distances

    def path(self, start, goal):
        if start == goal:
            return []
        if not self.free[start] or not self.free[goal]:
            return None
        previous = {start: -1}
        queue = deque([start])
        while queue:
            cell = queue.popleft()
            for other in self.world.neighbors[cell]:
                if not self.free[other] or other in previous:
                    continue
                previous[other] = cell
                if other == goal:
                    route = [other]
                    while previous[route[-1]] != start:
                        route.append(previous[route[-1]])
                    return list(reversed(route))
                queue.append(other)
        return None
