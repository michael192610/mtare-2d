"""Independent saved-record checks for anchored tours and lookahead commands."""
from collections import deque
import math

from verify_dense import node_xy, node_region, neighbors


def audit_anchored_route(record,config,scene):
    route=record['route'];cells=route['cells'];allowed=set(record['local_candidate_cells'])
    anchors=route['anchors'];a,robot,previous,b=anchors
    assert robot==record['start_cell']
    assert set(cells)==set(anchors)|set(record['selected_viewpoints'])
    scale=config['viewpoint_spacing']
    for i,start in enumerate(cells):
        distances={start:0};queue=deque([start])
        while queue:
            current=queue.popleft()
            for other in neighbors(current,config,scene):
                if other in allowed and other not in distances:
                    distances[other]=distances[current]+1;queue.append(other)
        assert route['distance_matrix'][i]==[int(10*(float(distances[c])*scale)) for c in cells]
    matrix=[row.copy() for row in route['distance_matrix']]
    def add_dummy(left,right):
        i=len(matrix)
        for j,row in enumerate(matrix):row.append(0 if j in (left,right) else 99_999_999)
        matrix.append([0 if j in (left,right,i) else 99_999_999 for j in range(i+1)])
    if robot!=previous:add_dummy(cells.index(robot),cells.index(previous))
    if a!=b:add_dummy(cells.index(a),cells.index(b))
    ordered=route['ordered_cells'];path=route['full_path'];flags=route['full_path_is_anchor']
    assert ordered[0]==a and ordered[-1]==b
    assert [n for n,f in zip(path,flags) if f]==ordered
    for left,right in zip(path,path[1:]):
        assert left in allowed and right in neighbors(left,config,scene)
    assert math.isclose(route['length'],(len(path)-1)*scale,abs_tol=1e-8)
    if route['solver_called']:
        assert route['solver_distance_matrix']==matrix
        tour=route['solver_tour'];assert set(tour)==set(range(len(matrix)))
        arcs=[matrix[x][y] for x,y in zip(tour,[*tour[1:],tour[0]])]
        assert max(arcs)<99_999_999 and sum(arcs)==route['objective']
    else:
        assert len(cells)==1 and not route['execution_path']
    if previous!=robot:
        assert any({x,y}=={robot,previous} for x,y in zip(ordered,ordered[1:]))
    command=route['execution_path'];last=robot
    for node in command:
        assert node in allowed and node in neighbors(last,config,scene)
        last=node
    assert last==record['lookahead']['target']
    assert math.isclose(route['execution_length'],len(command)*scale,abs_tol=1e-8)
    eligible={n for n in allowed if record['region_states_before'][str(node_region(n,config,scene))] in (0,1,5)}
    assert set(record['eligible_viewpoint_cells'])==eligible
    assert not set(record['selected_viewpoints'])&set(record['visited_viewpoint_cells'])


def audit_direction(record,config,scene,known,position,deprioritized):
    look=record['lookahead'];route=record['route']
    xy=lambda n:node_xy(n,config,scene)
    region=lambda n:node_region(n,config,scene)
    assert all(math.isclose(x,y,abs_tol=1e-8) for x,y in zip(look['robot_position'],position))
    assert set(look['priority_region_ids'])==set(deprioritized)&set(record['local_window_region_ids'])
    low=set(look['priority_region_ids']);path=route['full_path'];flags=route['full_path_is_anchor']
    a,robot,previous,b=route['anchors']
    if look['previous_valid'] and look['previous_point'] is not None:
        point=look['previous_point']
        reg=int(point[1]//config['region_size'])*scene['region_cols']+int(point[0]//config['region_size'])
        expected=(min(record['local_candidate_cells'],key=lambda n:(sum((xy(n)[i]-point[i])**2 for i in (0,1)),n))
                  if reg in record['local_window_region_ids'] else robot)
        assert previous==expected
    else:
        assert previous==robot
    coverage=set(record['selected_viewpoints'])-{a,robot,b}
    gain=record['score_thresholds']
    if previous!=robot and (len(record['viewpoint_coverage'][str(previous)])>=gain['ordinary_min']
            or len(record['viewpoint_frontier_coverage'][str(previous)])>=gain['frontier_min']):
        coverage.add(previous)
    assert set(look['coverage_viewpoints'])==coverage
    idx=next(i for i,n in enumerate(path) if n==robot and flags[i]);loop=path[0]==path[-1]==robot
    forward=list(range(idx+1,len(path)));backward=list(range(idx-1,-1,-1))
    if loop:forward=list(range(1,len(path)));backward=list(range(len(path)-2,-1,-1))
    counts=[sum(flags[i] and path[i] in coverage for i in branch) for branch in (forward,backward)]
    assert counts==look['viewpoint_counts']
    too_short=all(math.dist(position,xy(n))<=config['lookahead_distance']/5 for n in path)
    assert too_short==look['local_path_too_short']
    size=config['cell_size'];cols=scene['cols']
    def los(point):
        dx,dy=point[0]-position[0],point[1]-position[1];cuts={0.,1.}
        for axis,delta in ((0,dx),(1,dy)):
            if abs(delta)<1e-12:continue
            lo,hi=sorted((position[axis],point[axis]))
            cuts.update((k*size-position[axis])/delta
                for k in range(math.floor(lo/size)+1,math.ceil(hi/size)))
        def free(t):
            x,y=position[0]+dx*t,position[1]+dy*t
            return known[int(y//size)*cols+int(x//size)]==0
        cuts=sorted(cuts)
        return free(0) and free(1) and all(free((x+y)/2) for x,y in zip(cuts,cuts[1:]))
    for name,indices in (('forward',forward),('backward',backward)):
        branch=[path[i] for i in indices];travelled=0.;last=robot;skipped=[];target=None
        for j,node in enumerate(branch):
            old,last=last,node
            if any(counts) and region(node) in low:
                skipped.append(node);continue
            travelled+=math.dist(xy(old),xy(node))
            stop=(travelled>config['lookahead_distance'] or
                  (j+1<len(branch) and not los(xy(branch[j+1]))) or
                  (flags[indices[j]] and (node in coverage or node in (a,b))) or j==len(branch)-1)
            if stop:target=node;break
        assert look[name]['target']==target and look[name]['skipped_nodes']==skipped
        expected=-2.
        if target is not None and math.dist(xy(target),position)>1e-12:
            d=math.dist(xy(target),position)
            expected=sum(look['previous_direction'][i]*(xy(target)[i]-position[i])/d for i in (0,1))
        assert math.isclose(look[name]['score'],expected,abs_tol=1e-8)
    ds=sum(math.dist(xy(x),xy(y)) for x,y in zip(path,path[1:idx+1]))
    de=sum(math.dist(xy(x),xy(y)) for x,y in zip(path[idx:],path[idx+1:]))
    assert all(math.isclose(x,y,abs_tol=1e-8) for x,y in zip((ds,de),look['distances_from_boundaries']))
    if not any(counts):
        follow=False if ds<de and a!=robot else True if de<ds and b!=robot else look['forward']['score']>look['backward']['score']
    elif look['forward']['score']>look['backward']['score']:
        follow=counts[0]>0
    else:
        follow=counts[1]==0
    assert look['follow_forward']==follow
    assert look['follow_path']==[path[i] for i in (forward if follow else backward)]
    chosen=look['forward' if follow else 'backward']['target']
    valid=chosen is not None and chosen!=robot and not too_short
    if too_short:chosen=a if ds<de else b
    if chosen is None:chosen=robot
    assert look['target']==chosen and look['valid']==valid
    assert look['point']==list(xy(chosen))
    norm=math.dist(xy(chosen),position)
    expected=[(xy(chosen)[i]-position[i])/norm for i in (0,1)] if norm>1e-12 and valid else look['previous_direction']
    assert all(math.isclose(x,y,abs_tol=1e-8) for x,y in zip(expected,look['direction']))
