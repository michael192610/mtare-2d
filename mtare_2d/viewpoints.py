"""Independent metric viewpoint lattice over private 2 m occupancy.

Spacing is configurable (M-TARE indoor default 0.6 m); visit-neighbor radius is 3 m. IDs are disjoint from
fine cells and region tasks; the underlying occupancy geometry never changes.
"""
from collections import deque
import math

import numpy as np

from .world import FREE


class ViewpointLattice:
    def __init__(self, world):
        self.world = world
        self.spacing = world.config.viewpoint_spacing
        self.cols = math.ceil(world.config.width/self.spacing - .5)
        self.rows = math.ceil(world.config.height/self.spacing - .5)
        self.size = self.rows*self.cols
        self.offset = world.size+world.regions.size
        self.centres = np.array([((c+.5)*self.spacing,(r+.5)*self.spacing)
                                for r in range(self.rows) for c in range(self.cols)])
        fine_cols = np.minimum((self.centres[:,0]/world.config.cell_size).astype(int),world.cols-1)
        fine_rows = np.minimum((self.centres[:,1]/world.config.cell_size).astype(int),world.rows-1)
        self.fine_cells = fine_rows*world.cols+fine_cols
        self.region_ids = world.regions.cell_region[self.fine_cells]
        self.neighbors = []
        for i in range(self.size):
            r,c = divmod(i,self.cols)
            self.neighbors.append(tuple(j for valid,j in (
                (r>0,i-self.cols),(c>0,i-1),(c+1<self.cols,i+1),(r+1<self.rows,i+self.cols)) if valid))
        rings = math.ceil(world.config.visited_viewpoint_radius/self.spacing)
        radius2 = world.config.visited_viewpoint_radius**2
        self.visit_offsets = [(r,c) for r in range(-rings,rings+1) for c in range(-rings,rings+1)
                              if (r*r+c*c)*self.spacing**2 <= radius2+1e-9]

    def contains(self, node):
        return self.offset <= int(node) < self.offset+self.size

    def xy(self, node):
        return tuple(map(float,self.centres[int(node)-self.offset]))

    def nearest_index(self, position):
        c=min(self.cols-1,max(0,int(position[0]/self.spacing)))
        r=min(self.rows-1,max(0,int(position[1]/self.spacing)))
        return r*self.cols+c

    def mask_from_poses(self, poses, window):
        """Original: snap each historical pose, then mark its in-range neighbors."""
        result=set()
        for pose in poses:
            index=self.nearest_index(pose)
            if self.world.region_of(self.world.cell_at(pose)) not in window:
                continue
            r,c=divmod(index,self.cols)
            for dr,dc in self.visit_offsets:
                rr,cc=r+dr,c+dc
                if 0<=rr<self.rows and 0<=cc<self.cols:
                    result.add(self.offset+rr*self.cols+cc)
        return result

    def graph(self, known, window, position):
        free=(known.ravel()[self.fine_cells]==FREE)&np.isin(self.region_ids,list(window))
        index=self.nearest_index(position)
        if not free[index]:
            candidates=np.flatnonzero(free)
            if not len(candidates):raise AssertionError('No known-free viewpoint near robot')
            index=int(candidates[np.argmin(np.sum((self.centres[candidates]-position)**2,axis=1))])
            if math.dist(self.centres[index],position)>self.spacing*1.5:
                raise AssertionError('Robot cannot snap safely to nearby viewpoint lattice')
        return ViewpointGraph(self,free,index)

    def metadata(self):
        return dict(spacing=self.spacing,cols=self.cols,rows=self.rows,id_offset=self.offset,
                    id_namespace='viewpoint',occupancy_cell_size=self.world.config.cell_size)


class ViewpointGraph:
    def __init__(self,lattice,free,start):
        self.lattice=lattice
        self.step_size=lattice.spacing
        self.free=free
        self.start=start+lattice.offset
        self._distances={}
        reached=self._bfs(start)
        self.allowed=set(reached)
        self._distances[self.start]={i+lattice.offset:n for i,n in reached.items()}
        self.candidates=sorted(i+lattice.offset for i in reached)

    def _bfs(self,start):
        result={start:0};queue=deque([start])
        neighbors=self.lattice.neighbors
        while queue:
            current=queue.popleft();distance=result[current]+1
            for other in neighbors[current]:
                if self.free[other] and other not in result:
                    result[other]=distance;queue.append(other)
        return result

    def distances(self,start):
        if start not in self._distances:
            self._distances[start]={i+self.lattice.offset:n for i,n in self._bfs(start-self.lattice.offset).items()}
        return self._distances[start]

    def path(self,start,end):
        if start==end:return []
        previous={start:None};queue=deque([start]);offset=self.lattice.offset
        while queue:
            node=queue.popleft()
            for index in self.lattice.neighbors[node-offset]:
                other=index+offset
                if self.free[index] and other not in previous:
                    previous[other]=node
                    if other==end:
                        path=[other]
                        while previous[path[-1]]!=start:path.append(previous[path[-1]])
                        return list(reversed(path))
                    queue.append(other)
        return None
