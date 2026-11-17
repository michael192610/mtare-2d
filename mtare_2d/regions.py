"""Shared 2D subspace geometry, separate from fine occupancy and viewpoints.

Region task keys occupy a separate namespace: fine_grid_size + region_id.
This keeps depot fine-cell IDs and persistent region identities unambiguous.
"""
import math

import numpy as np


class Regions:
    def __init__(self, rows, cols, cell_size, region_size):
        self.fine_rows, self.fine_cols = rows, cols
        self.cell_size, self.region_size = cell_size, region_size
        self.stride = round(region_size / cell_size)
        self.rows, self.cols = math.ceil(rows/self.stride), math.ceil(cols/self.stride)
        self.offset = rows*cols
        self.size = self.rows*self.cols
        self.cell_region = np.array([(r//self.stride)*self.cols+c//self.stride
                                     for r in range(rows) for c in range(cols)], dtype=np.int32)
        self.cells = tuple(np.flatnonzero(self.cell_region == i) for i in range(self.size))

    def task(self, region):
        return self.offset + int(region)

    def region(self, task):
        region = int(task) - self.offset
        if not 0 <= region < self.size:
            raise ValueError(f"Not a region task: {task}")
        return region

    def centre(self, region):
        row, col = divmod(int(region), self.cols)
        x0, y0 = col*self.region_size, row*self.region_size
        return ((x0+min(x0+self.region_size, self.fine_cols*self.cell_size))/2,
                (y0+min(y0+self.region_size, self.fine_rows*self.cell_size))/2)

    def nearby(self, fine_cell, radius):
        row, col = divmod(int(self.cell_region[fine_cell]), self.cols)
        return {r*self.cols+c for r in range(max(0,row-radius),min(self.rows,row+radius+1))
                for c in range(max(0,col-radius),min(self.cols,col+radius+1))}

    def neighbors(self, region):
        row,col=divmod(region,self.cols)
        return [r*self.cols+c for r in range(max(0,row-1),min(self.rows,row+2))
                for c in range(max(0,col-1),min(self.cols,col+2)) if r!=row or c!=col]

    def representative(self, world, known, region, reachable=None):
        cells = self.cells[int(region)]
        eligible = cells[known.ravel()[cells] == 0]
        if reachable is not None:
            eligible = eligible[reachable[eligible]]
        if not len(eligible):
            return None
        centre = np.asarray(self.centre(region))
        distances = np.sum((world.centres[eligible]-centre)**2, axis=1)
        return int(eligible[np.argmin(distances)])


def select_global_regions(world, known, depots, own_cell, knowledge=None, peer_local_regions=(), roadmap=None):
    """M-TARE exploration states select tasks; frontier mode preserves step 1."""
    from .world import GridGraph
    regions = world.regions
    graph = GridGraph(world, known)
    reachable = np.zeros(world.size, dtype=bool)
    for depot in set(depots):
        reachable |= graph.distances(depot) >= 0
    frontiers = [cell for cell in world.frontier_ids(known) if reachable[cell]]
    local_window = regions.nearby(own_cell, world.config.local_region_radius)
    excluded = set(peer_local_regions)
    if knowledge is None:
        active = sorted({int(regions.cell_region[cell]) for cell in frontiers})
        global_ids = [region for region in active if region not in local_window]
    else:
        from .region_state import ACTIVE, Evidence
        active = [i for i,cell in enumerate(knowledge.regions) if cell.status in ACTIVE]
        global_ids = [region for region in active if
            (region not in local_window or
             (knowledge.evidence.get(region,Evidence()).candidates==0 and knowledge.regions[region].visit_count>1))
            and region not in excluded
            and (roadmap.connected(region) if roadmap is not None else
                 regions.representative(world,known,region,reachable) is not None)]
    if world.config.max_targets:
        global_ids = global_ids[:world.config.max_targets]
    navigation = [regions.representative(world, known, region, reachable) for region in global_ids]
    return dict(frontier_cell_ids=frontiers, active_region_ids=active,
                local_region_ids=[region for region in active if region in local_window],
                local_window_region_ids=sorted(local_window), global_region_ids=global_ids,
                targets=[regions.task(region) for region in global_ids], navigation_cells=navigation,
                state_model="mtare" if knowledge is not None else "frontier",
                excluded_peer_local_region_ids=sorted(excluded),
                active_region_states={str(region): int(knowledge.regions[region].status) for region in active}
                                     if knowledge is not None else {})
