"""Independent geometry/route checks for the .6 m viewpoint ID namespace."""
from collections import deque
import math


def geometry(config,scene):
    spacing=config['viewpoint_spacing'];cols=math.ceil(config['width']/spacing-.5)
    rows=math.ceil(config['height']/spacing-.5)
    offset=scene['rows']*scene['cols']+scene['region_rows']*scene['region_cols']
    return spacing,cols,rows,offset


def node_xy(node,config,scene):
    s,cols,rows,offset=geometry(config,scene)
    if node>=offset:
        r,c=divmod(node-offset,cols);assert 0<=r<rows
        return ((c+.5)*s,(r+.5)*s)
    r,c=divmod(node,scene['cols']);return ((c+.5)*config['cell_size'],(r+.5)*config['cell_size'])


def node_region(node,config,scene):
    x,y=node_xy(node,config,scene)
    return int(y//config['region_size'])*scene['region_cols']+int(x//config['region_size'])


def neighbors(node,config,scene):
    _,cols,rows,offset=geometry(config,scene);i=node-offset;r,c=divmod(i,cols)
    return ([node-cols] if r else [])+([node+cols] if r+1<rows else [])+ \
           ([node-1] if c else [])+([node+1] if c+1<cols else [])


def audit_route(record,config,scene):
    if record['route'].get('anchor_model')=='mtare_dummy_nodes':
        from verify_lookahead import audit_anchored_route
        audit_anchored_route(record,config,scene)
        return
    s,cols,rows,offset=geometry(config,scene)
    assert abs(record['viewpoint_geometry']['spacing']-.6)<1e-9 if config['viewpoint_spacing']==.6 else True
    assert record['viewpoint_geometry']==dict(spacing=s,cols=cols,rows=rows,id_offset=offset,
        id_namespace='viewpoint',occupancy_cell_size=config['cell_size'])
    allowed=set(record['local_candidate_cells']);route=record['route'];cells=route['cells']
    assert route['ordered_cells'][0]==record['start_cell']
    assert route['ordered_cells'][-1]==record['end_cell']
    assert set(record['selected_viewpoints'])<=set(route['ordered_cells'])
    for i,start in enumerate(cells):
        distance={start:0};queue=deque([start])
        while queue:
            c=queue.popleft()
            for other in neighbors(c,config,scene):
                if other in allowed and other not in distance:
                    distance[other]=distance[c]+1;queue.append(other)
        for j,end in enumerate(cells):
            assert route['distance_matrix'][i][j]==round(distance[end]*s*10)
    previous=record['start_cell']
    for c in route['execution_path']:
        assert c in allowed and c in neighbors(previous,config,scene);previous=c
    assert previous==record['end_cell']
    assert math.isclose(route['length'],len(route['execution_path'])*s,abs_tol=1e-8)
    objective=sum(route['distance_matrix'][cells.index(a)][cells.index(b)]
                  for a,b in zip(route['ordered_cells'],route['ordered_cells'][1:]))
    assert objective==round(route['length']*10)
    if route['solver_called']:assert objective==route['objective']
    eligible=(allowed-set(record['visited_viewpoint_cells']) if record.get('coverage_model')=='simple_frontier' else
              {c for c in allowed if record['region_states_before'][str(node_region(c,config,scene))] in (0,1,5)})
    assert set(record['eligible_viewpoint_cells'])==eligible
    assert not set(record['selected_viewpoints'])&set(record['visited_viewpoint_cells'])

def audit_visited_masks(directory,config,scene):
    """Reconstruct candidate connectivity and visited masks from events only."""
    import json
    import numpy as np
    if 'viewpoint_spacing' not in config:return {}
    s,cols,rows,offset=geometry(config,scene);size=cols*rows
    finecols=scene['cols'];fine_size=scene['rows']*finecols
    vxy=np.array([((i%cols+.5)*s,(i//cols+.5)*s) for i in range(size)])
    fine_of=(vxy[:,1]/config['cell_size']).astype(int)*finecols+(vxy[:,0]/config['cell_size']).astype(int)
    regions=(vxy[:,1]/config['region_size']).astype(int)*scene['region_cols']+(vxy[:,0]/config['region_size']).astype(int)
    truth=np.load(directory/'maps.npz')['truth'].ravel()
    fine_xy=np.array([((i%finecols+.5)*config['cell_size'],(i//finecols+.5)*config['cell_size']) for i in range(fine_size)])
    private=[];histories=[];positions=list(scene['starts']);masks=[];states=[]
    for p in positions:
        visible=np.sum((fine_xy-p)**2,axis=1)<=config['reveal_radius']**2
        visible[int(p[1]/config['cell_size'])*finecols+int(p[0]/config['cell_size'])]=True
        grid=np.full(fine_size,-1);grid[visible]=truth[visible];private.append(grid)
        histories.append([p]);masks.append(set());states.append([0]*(scene['region_rows']*scene['region_cols']))
    radius=config['visited_viewpoint_radius'];rings=math.ceil(radius/s)
    displacements=[(r,c) for r in range(-rings,rings+1) for c in range(-rings,rings+1) if (r*r+c*c)*s*s<=radius*radius+1e-9]
    def verify(robot,window,start=None,status_override=None):
        allowed=(private[robot][fine_of]==0)&np.isin(regions,list(window))
        if start is None:
            p=positions[robot];start=min(rows-1,int(p[1]/s))*cols+min(cols-1,int(p[0]/s))
            if not allowed[start]:
                choices=np.flatnonzero(allowed);start=int(choices[np.argmin(np.sum((vxy[choices]-p)**2,axis=1))])
        else:start-=offset
        reached={start};queue=deque([start])
        while queue:
            i=queue.popleft();r,c=divmod(i,cols)
            for valid,other in ((r>0,i-cols),(r+1<rows,i+cols),(c>0,i-1),(c+1<cols,i+1)):
                if valid and allowed[other] and other not in reached:reached.add(other);queue.append(other)
        candidate={i+offset for i in reached};mask=masks[robot]
        for node in candidate:
            reg=int(regions[node-offset])
            status=(status_override[str(reg)] if status_override is not None else states[robot][reg])
            if status==3:mask.add(node)
            elif status==5:mask.discard(node)
        for p in histories[robot]:
            physical_region=int(p[1]/config['region_size'])*scene['region_cols']+int(p[0]/config['region_size'])
            if physical_region not in window:continue
            r=min(rows-1,int(p[1]/s));c=min(cols-1,int(p[0]/s))
            for dr,dc in displacements:
                rr,cc=r+dr,c+dc
                if 0<=rr<rows and 0<=cc<cols:mask.add(offset+rr*cols+cc)
        return candidate,mask&candidate
    count=credits=0
    last_masks={}
    context_checks=0
    for line in (directory/'events.jsonl').open():
        e=json.loads(line);kind=e['kind'];robot=e.get('robot_id')
        if kind=='reveal':private[robot][e['cells']]=truth[e['cells']]
        elif kind=='region_state_changed':states[robot][e['region_id']]=e['status']
        elif kind=='move':
            p=e['position'];positions[robot]=p
            if not any(math.dist(p,old)<1 for old in histories[robot]):histories[robot].append(p)
        elif kind=='coverage_samples_updated' and e.get('coverage_source')=='visited_viewpoints':
            p=positions[robot];rr=int(p[1]/config['region_size']);cc=int(p[0]/config['region_size']);radius=config['local_region_radius']
            window={r*scene['region_cols']+c for r in range(max(0,rr-radius),min(scene['region_rows'],rr+radius+1))
                    for c in range(max(0,cc-radius),min(scene['region_cols'],cc+radius+1))}
            _,visited=verify(robot,window)
            assert set(e['visited_viewpoint_ids'])==visited,('visited coverage source',e['tick'],robot)
            credits+=1
        elif kind=='viewpoint_visit_update':
            candidate,visited=verify(robot,set(e['local_window_region_ids']),e['start_viewpoint'])
            assert candidate==set(e['candidate_viewpoints'])
            assert visited==set(e['visited_viewpoints']),('context visited mask',e['tick'],robot)
            last_masks[robot]=(candidate,visited)
            context_checks+=1
        elif kind=='local_plan_selected':
            record=json.loads((directory/'local_calls'/(e['local_plan_id']+'.json')).read_text())
            if record.get('coverage_model') not in ('mtare_dense_viewpoints','simple_frontier'):continue
            window=set(record['local_window_region_ids'])
            if robot in last_masks:
                candidate,visited=last_masks[robot]
            else:
                candidate,visited=verify(robot,window,record['start_cell'],record['region_states_before'])
            assert candidate==set(record['local_candidate_cells']),('candidate connectivity',record['local_plan_id'])
            assert visited==set(record['visited_viewpoint_cells']),('visited mask',record['local_plan_id'])
            assert np.allclose(histories[robot],record['visited_pose_history'])
            count+=1
    return dict(verified_local_masks=count,verified_visited_coverage_sources=credits,verified_context_masks=context_checks)
