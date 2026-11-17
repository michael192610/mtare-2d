"""Independent frontier-only evidence replay; no simulated planner imports."""
from collections import deque
import json
import math

import numpy as np

from verify_dense import node_xy


def audit_frontier_replay(directory,config,scene):
    if config.get('coverage_model')!='simple' or config['region_state_model']!='mtare' or config['architecture']!='hierarchical':return {}
    cols,rows=scene['cols'],scene['rows'];size=cols*rows;scale=config['cell_size']
    positions=scene['starts'].copy();vehicles=config['robots']
    truth=np.load(directory/'maps.npz')['truth'].ravel()
    centres=np.array([((i%cols+.5)*scale,(i//cols+.5)*scale) for i in range(size)])
    private=[]
    for point in positions:
        visible=np.sum((centres-point)**2,axis=1)<=config['reveal_radius']**2
        visible[int(point[1]/scale)*cols+int(point[0]/scale)]=True
        grid=np.full(size,-1);grid[visible]=truth[visible];private.append(grid)
    def neighbors(c):
        r,k=divmod(c,cols)
        return ([c-cols] if r else [])+([c-1] if k else [])+([c+1] if k+1<cols else [])+([c+cols] if r+1<rows else [])
    def region(c):
        x,y=centres[c];return int(y/config['region_size'])*scene['region_cols']+int(x/config['region_size'])
    calls=0;active_checks=0;empty_checks=0
    for line in (directory/'events.jsonl').open():
        e=json.loads(line);kind=e['kind'];robot=e.get('robot_id')
        if kind=='reveal':private[robot][e['cells']]=truth[e['cells']]
        elif kind=='move':positions[robot]=e['position']
        elif kind=='coverage_samples_updated':raise AssertionError('Simple default must not use an ordinary coverage ledger')
        elif kind=='local_plan_selected':
            rec=json.loads((directory/'local_calls'/(e['local_plan_id']+'.json')).read_text())
            assert rec['coverage_model']=='simple_frontier' and rec['coverage_state']=={}
            assert not rec['ordinary_sample_cells'] and not rec['ordinary_queue']
            grid=private[robot];p=positions[robot];start=int(p[1]/scale)*cols+int(p[0]/scale)
            reachable={start};queue=deque([start])
            while queue:
                c=queue.popleft()
                for n in neighbors(c):
                    if grid[n]==0 and n not in reachable:reachable.add(n);queue.append(n)
            window=set(rec['local_window_region_ids'])
            frontiers=sorted(c for c in reachable if region(c) in window and any(grid[n]==-1 for n in neighbors(c)))
            assert rec['reachable_frontier_cells']==frontiers
            sources=[c for c in frontiers if rec['region_states_before'][str(region(c))]!=3]
            raw={n for c in sources for n in neighbors(c) if grid[n]==-1}
            supported=set();visited=set(rec['visited_viewpoint_cells'])
            for node,points in rec['viewpoint_frontier_coverage'].items():
                node=int(node);point=node_xy(node,config,scene)
                expected=set() if node in visited else {c for c in raw if math.dist(centres[c],point)<=config['reveal_radius']+1e-9}
                assert set(points)==expected,(rec['local_plan_id'],node)
                supported|=expected
            assert set(rec['frontier_sample_cells'])==supported
            for reg,ev in rec.get('region_coverage_evidence',{}).items():
                reg=int(reg);local=[c for c in frontiers if region(c)==reg]
                assert ev['frontier_cells']==local
                assert ev['reachable_cells']==sum(region(c)==reg for c in reachable)
                active_checks+=bool(local);empty_checks+=not local
            calls+=1
    for value in json.loads((directory/'coverage_bookkeeping.json').read_text()).values():assert not value
    return dict(verified_frontier_calls=calls,regions_with_frontiers=active_checks,regions_without_frontiers=empty_checks,
                ordinary_ledger_used=False)
