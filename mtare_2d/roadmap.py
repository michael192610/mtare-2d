"""Region roadmap: observed connections, geometric costs, compact edge messages.

2D adapter for GridWorld::UpdateRoadmap/CellToCellDistance. Fine paths establish
local connections; they are neither transmitted nor used as global edge weights.
"""
from dataclasses import asdict, dataclass
import heapq
import math

from .world import FREE, GridGraph

INF = 99_999_999


@dataclass(frozen=True)
class RoadmapEdge:
    from_cell_id: int
    to_cell_id: int
    weight: float = 0.0  # 10 * from mobility + to mobility; homogeneous NONE.

    def to_dict(self):
        return asdict(self)


class Roadmap:
    def __init__(self, world):
        self.world = world
        self.nodes = set()
        self.adjacency = {}
        self.provenance = {}

    def add_edge(self, left, right, provenance=None):
        if left == right or not (0 <= left < self.world.regions.size and
                                 0 <= right < self.world.regions.size):
            raise ValueError('Invalid roadmap endpoints')
        self.nodes.update((left, right))
        distance = math.dist(self.world.regions.centre(left), self.world.regions.centre(right))
        self.adjacency.setdefault(left, {})[right] = distance
        self.adjacency.setdefault(right, {})[left] = distance
        key = tuple(sorted((left, right)))
        new = key not in self.provenance
        if new:
            self.provenance[key] = provenance or {'kind': 'unspecified'}
        return new

    def connected(self, region):
        return bool(self.adjacency.get(region))

    def component(self, region):
        reached, queue = {region}, [region]
        while queue:
            for other in self.adjacency.get(queue.pop(), {}):
                if other not in reached:
                    reached.add(other)
                    queue.append(other)
        return reached

    def shortest_path(self, start, goal):
        queue, best = [(0.0, start, (start,))], {start: 0.0}
        while queue:
            distance, current, path = heapq.heappop(queue)
            if distance != best[current]:
                continue
            if current == goal:
                return distance, list(path)
            for other, length in self.adjacency.get(current, {}).items():
                trial = distance + length
                if trial < best.get(other, math.inf):
                    best[other] = trial
                    heapq.heappush(queue, (trial, other, (*path, other)))
        return INF, []

    def cost(self, left, right):
        if self.connected(left) and self.connected(right):
            distance, _ = self.shortest_path(left, right)
            return int(distance), 'roadmap' if distance < INF else 'roadmap_no_path'
        return int(math.dist(self.world.regions.centre(left),
                             self.world.regions.centre(right))), 'straight_line_fallback'

    def discover(self, known, start, window):
        """Validate adjacent-region links by a <=2-region-width known-free path.

        This is the explicit 2D substitute for direct keypose connectivity. Only
        the robot's map and local window are consulted, never simulator truth.
        """
        regions, world = self.world.regions, self.world
        reachable = GridGraph(world, known).distances(start) >= 0
        representatives = {region: regions.representative(world, known, region, reachable)
                           for region in window}
        pairs = set()
        for cell in (i for region in window for i in regions.cells[region]
                     if reachable[i] and known.ravel()[i] == FREE):
            left = int(regions.cell_region[cell])
            for other in world.neighbors[cell]:
                right = int(regions.cell_region[other])
                if right in window and left != right and reachable[other]:
                    pairs.add(tuple(sorted((left, right))))
        added = []
        for left, right in sorted(pairs):
            if right in self.adjacency.get(left, {}):
                continue
            a, b = representatives[left], representatives[right]
            if a is None or b is None:
                continue
            local = known.copy()
            mask = (regions.cell_region != left) & (regions.cell_region != right)
            local.ravel()[mask] = -1
            path = GridGraph(world, local).path(a, b)
            if path is not None and len(path)*world.config.cell_size <= 2*world.config.region_size:
                if self.add_edge(left, right, {'kind': 'local', 'path': [a, *path]}):
                    added.append((left, right, [a, *path]))
        return added

    def messages(self):
        return tuple(RoadmapEdge(a, b) for a, b in sorted(self.provenance))

    def snapshot(self):
        return [dict(from_cell_id=a, to_cell_id=b, length=self.adjacency[a][b],
                     provenance=self.provenance[a,b]) for a,b in sorted(self.provenance)]


def roadmap_matrices(world, roadmap, depot_cells, targets, receiver, connected, knowledge):
    """GetDistanceMatricesWithTraversability + GetDistanceMatricesNoComms.

    Homogeneous NONE mobility only. Depot coordinates are mapped into regions
    before applying CellToCellDistance, exactly as task endpoints are.
    """
    import numpy as np
    count = len(depot_cells)
    node_regions = [int(world.regions.cell_region[c]) for c in depot_cells] + [
        world.regions.region(t) for t in targets]
    n = len(node_regions)
    base = np.zeros((n,n), dtype=np.int64)
    branches = [['diagonal']*n for _ in range(n)]
    for row in range(n):
        for col in range(row):
            value, branch = roadmap.cost(node_regions[row], node_regions[col])
            if row >= count and col >= count:
                value += world.config.task_arc_offset
            base[row,col] = base[col,row] = value
            branches[row][col] = branches[col][row] = branch
    matrices = np.repeat(base[None], count, axis=0)
    restricted = []
    for vehicle in range(count):
        if vehicle == receiver or vehicle in connected:
            continue
        for row in range(count,n):
            for col in range(row):
                allowed = knowledge.known_by(node_regions[row], vehicle) and (
                    col < count or knowledge.known_by(node_regions[col], vehicle))
                if not allowed:
                    matrices[vehicle,row,col] = matrices[vehicle,col,row] = INF
                    restricted.append([vehicle,row,col])
    return matrices, dict(model='region_roadmap', node_region_ids=node_regions,
        roadmap_edges=roadmap.snapshot(), branches=branches,
        knowledge_masks=[cell.known_by_robot_ids for cell in knowledge.regions],
        knowledge_restrictions=restricted)
