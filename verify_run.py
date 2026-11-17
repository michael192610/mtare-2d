#!/usr/bin/env python3
"""Independently audit saved VRP objectives, receipt provenance and movement."""
import argparse
from collections import deque
import json
import math
from pathlib import Path

import numpy as np


def audit_local_calls(directory, config, scene):
    records = {}
    cols, rows = scene["cols"], scene["rows"]
    scale = config["cell_size"]
    def neighbors(cell):
        row, col = divmod(cell,cols)
        return ([cell-cols] if row else []) + ([cell+cols] if row+1<rows else []) + \
               ([cell-1] if col else []) + ([cell+1] if col+1<cols else [])
    def xy(cell):
        row,col=divmod(cell,cols)
        return ((col+.5)*scale,(row+.5)*scale)
    for path in sorted((directory/'local_calls').glob('*.json')):
        record=json.loads(path.read_text())
        if record.get('coverage_model') in ('mtare_dense_viewpoints','simple_frontier'):
            from verify_dense import audit_route
            audit_route(record,config,scene)
            audit_sample_selection(record,config,scene)
            records[record['local_plan_id']]=record
            continue
        route=record['route']
        allowed=set(record['local_candidate_cells'])
        cells=route['cells']
        assert route['ordered_cells'][0]==record['start_cell'],path
        assert route['ordered_cells'][-1]==record['end_cell'],path
        assert set(record['selected_viewpoints']) <= set(route['ordered_cells']),path
        for i,start in enumerate(cells):
            distance={start:0}
            queue=deque([start])
            while queue:
                current=queue.popleft()
                for other in neighbors(current):
                    if other in allowed and other not in distance:
                        distance[other]=distance[current]+1
                        queue.append(other)
            for j,end in enumerate(cells):
                assert route['distance_matrix'][i][j]==round(distance[end]*scale*10),path
        previous=record['start_cell']
        for cell in route['execution_path']:
            assert cell in allowed and cell in neighbors(previous),path
            previous=cell
        assert previous==record['end_cell'],path
        assert route['length']==len(route['execution_path'])*scale,path
        objective=sum(route['distance_matrix'][cells.index(a)][cells.index(b)]
                      for a,b in zip(route['ordered_cells'],route['ordered_cells'][1:]))
        assert objective==round(route['length']*10),path
        if route['solver_called']:
            assert objective==route['objective'],path
        states=record.get('region_states_before')
        if states:
            stride=round(config['region_size']/config['cell_size'])
            region_cols=math.ceil(cols/stride)
            def region(cell):
                row,col=divmod(cell,cols)
                return (row//stride)*region_cols+col//stride
            eligible={cell for cell in allowed if states[str(region(cell))] in (0,1,5)}
            assert set(record['eligible_viewpoint_cells'])==eligible,path
            assert set(record['selected_viewpoints'])<=eligible,path
        if record.get('coverage_model')=='mtare_samples':
            audit_sample_selection(record,config,scene)
        else:
            samples=set(record['frontier_sample_cells'])
            for viewpoint,covered in record['viewpoint_coverage'].items():
                position=xy(int(viewpoint))
                expected={cell for cell in samples if math.dist(position,xy(cell))<=config['reveal_radius']+1e-9}
                assert set(covered)==expected,path
        records[record['local_plan_id']]=record
    return records


def audit_region_states(directory, config):
    path = directory / "region_states.json"
    if not path.exists():
        return dict(applied=0,ignored=0,transitions=0)
    snapshots=json.loads(path.read_text())
    if not any(snapshots.values()):
        return dict(applied=0,ignored=0,transitions=0)
    statuses={int(robot):[0]*len(cells) for robot,cells in snapshots.items()}
    exploring={robot:[0]*len(cells) for robot,cells in statuses.items()}
    visits={robot:[0]*len(cells) for robot,cells in statuses.items()}
    versions={robot:[0]*len(cells) for robot,cells in statuses.items()}
    covered_flags={robot:[False]*len(cells) for robot,cells in statuses.items()}
    published,received={},{}
    counts=dict(applied=0,ignored=0,transitions=0,completion_republications=0,
                matching_report_syncs=0,rejected_report_syncs=0)
    repaired=config.get('repair_region_reports',False) and config.get('coverage_model')=='mtare'
    for line in (directory/"events.jsonl").open():
        event=json.loads(line)
        kind=event['kind']
        if kind=='publish':
            packet=event['message']
            updates=packet.get('region_updates',[])
            local=packet.get('local_exploring_region_ids',[])
            assert len(local)<=3
            assert all(0<=i<len(statuses[packet['sender']]) for i in local)
            for update in updates:
                assert set(update)=={'id','update_id','known_by_robot_ids','mobility_type','status'}
                assert update['status'] in (1,2)
                assert 0<=update['known_by_robot_ids']<2**config['robots']
                assert update['update_id']>=0
            published[packet['sender'],packet['sequence']]=(updates,local)
        elif kind=='receive':
            packet=published[event['sender'],event['sequence']]
            assert (event.get('region_updates',[]),event.get('local_exploring_region_ids',[]))==packet
            received[event['receiver'],event['sender']]={u['id']:u for u in packet[0]}
        elif kind=='region_window_entered':
            robot,region=event['robot_id'],event['region_id']
            visits[robot][region]+=1
            assert visits[robot][region]==event['visit_count']
        elif kind=='region_covered_to_others':
            robot,region=event['robot_id'],event['region_id']
            assert statuses[robot][region]==0
            assert event['update_id']>versions[robot][region]
            versions[robot][region]=event['update_id']
            covered_flags[robot][region]=True
        elif kind=='region_state_changed':
            robot,region=event['robot_id'],event['region_id']
            previous,new=event['previous_status'],event['status']
            assert previous==statuses[robot][region]
            if new in (1,5) and previous not in (1,5):
                exploring[robot][region]+=1
            assert event['exploring_count']==exploring[robot][region]
            assert event['visit_count']==visits[robot][region]
            assert event['update_id']>=versions[robot][region]
            if event['reason']=='new_coverage_reopens_region':
                assert previous==2 and new==1
                assert exploring[robot][region]-1<=config['region_max_exploring_count']
            if event['reason']=='equal_revision_completion_republished':
                assert repaired and previous==new and new in (0,2) and event['actor']==robot
                assert covered_flags[robot][region]
                assert event['update_id']==versions[robot][region]+1
                counts['completion_republications']+=1
            covered_flags[robot][region]=event['covered_to_others']
            statuses[robot][region],versions[robot][region]=new,event['update_id']
            counts['transitions']+=1
        elif kind=='region_update_applied':
            robot,region=event['robot_id'],event['region_id']
            update=received[robot,event['sender']][region]
            assert update['update_id']==event['update_id']
            assert update['status']==event['incoming_status']
            old=event['previous_status']
            expected=(5 if update['status']==1 else 3) if config.get('coverage_model')=='simple' else \
                     (5 if old in (0,3) and update['status']==1 else \
                     3 if old in (0,1,5) and update['status']==2 else old)
            assert event['status']==expected==statuses[robot][region]
            assert event['update_id']>=versions[robot][region]
            versions[robot][region]=event['update_id']
            counts['applied']+=1
        elif kind=='region_update_ignored':
            robot,region=event['robot_id'],event['region_id']
            if event['reason']=='protected_local_exploration':
                assert statuses[robot][region]==1
            elif event['reason']=='not_newer':
                assert event['incoming_update_id']<=versions[robot][region]
            counts['ignored']+=1
        elif repaired and kind in ('region_sync','region_sync_rejected'):
            robot,region=event['robot_id'],event['region_id']
            update=received[robot,event['sender']][region]
            status=statuses[robot][region]
            report=(2 if status in (2,3) or covered_flags[robot][region] else
                    1 if status in (1,5) else None)
            agrees=(update['update_id']==versions[robot][region] and update['status']==report)
            assert event['incoming_update_id']==update['update_id']
            assert event['update_id']==versions[robot][region]
            if kind=='region_sync':
                assert agrees, ('Acknowledged conflicting report',event)
                counts['matching_report_syncs']+=1
            else:
                assert update['update_id']>=versions[robot][region] and not agrees
                counts['rejected_report_syncs']+=1
    for robot,cells in snapshots.items():
        i=int(robot)
        for cell in cells:
            region=cell['id']
            assert cell['status']==statuses[i][region]
            assert cell['update_id']==versions[i][region]
            assert cell['visit_count']==visits[i][region]
            assert cell['exploring_count']==exploring[i][region]
    return counts


def audit_roadmap_costs(record, config, scene):
    import heapq
    info=record.get('cost_provenance')
    if not info:
        return 0
    cols=scene['cols']; stride=round(config['region_size']/config['cell_size'])
    region_cols=math.ceil(cols/stride)
    def centre(region):
        row,col=divmod(region,region_cols)
        a,b=col*config['region_size'],row*config['region_size']
        return ((a+min(a+config['region_size'],config['width']))/2,
                (b+min(b+config['region_size'],config['height']))/2)
    def region(cell):
        row,col=divmod(cell,cols)
        return (row//stride)*region_cols+col//stride
    n=record['num_vehicles']; nodes=record['node_cell_ids']
    regions=[region(c) for c in nodes[:n]]+[c-scene['rows']*cols for c in nodes[n:]]
    assert regions==info['node_region_ids']
    adjacency={}
    for edge in info['roadmap_edges']:
        a,b=edge['from_cell_id'],edge['to_cell_id']
        length=math.dist(centre(a),centre(b))
        assert abs(length-edge['length'])<1e-8
        adjacency.setdefault(a,{})[b]=length
        adjacency.setdefault(b,{})[a]=length
    def distance(a,b):
        if not adjacency.get(a) or not adjacency.get(b):
            return int(math.dist(centre(a),centre(b))),'straight_line_fallback'
        queue=[(0,a)];best={a:0}
        while queue:
            length,current=heapq.heappop(queue)
            if length!=best[current]:continue
            if current==b:return int(length),'roadmap'
            for other,step in adjacency[current].items():
                trial=length+step
                if trial<best.get(other,math.inf):
                    best[other]=trial;heapq.heappush(queue,(trial,other))
        return 99999999,'roadmap_no_path'
    checked=0
    restrictions=[]
    for vehicle,matrix in enumerate(record['distance_matrices']):
        for row in range(len(nodes)):
            assert matrix[row][row]==0
            for col in range(row):
                length,branch=distance(regions[row],regions[col])
                assert info['branches'][row][col]==branch
                if row>=n and col>=n:length+=config['task_arc_offset']
                if row>=n and vehicle!=record['robot_id'] and vehicle not in record['connected_peers']:
                    masks=info['knowledge_masks']
                    if not (masks[regions[row]] & (1<<vehicle)) or (col>=n and not masks[regions[col]]&(1<<vehicle)):
                        length=99999999;restrictions.append([vehicle,row,col])
                assert matrix[row][col]==matrix[col][row]==length, (record['call_id'],vehicle,row,col)
                checked+=1
    assert sorted(restrictions)==sorted(info['knowledge_restrictions'])
    return checked


def audit_compact_protocol(directory,config,scene):
    if config.get('architecture')!='hierarchical' or config.get('region_state_model')!='mtare':
        return {}
    # Legacy stateful runs have no roadmap snapshot and keep their old audit.
    if not (directory/'roadmaps.json').exists():return {}
    published={};graphs={i:set() for i in range(config['robots'])};checks=0
    arrays=np.load(directory/'maps.npz');truth=arrays['truth'].ravel();cols=scene['cols']
    centres=np.array([((i%cols+.5)*config['cell_size'],(i//cols+.5)*config['cell_size']) for i in range(len(truth))])
    private=[]
    for position in scene['starts']:
        known=np.full(len(truth),-1)
        visible=np.sum((centres-np.asarray(position))**2,axis=1)<=config['reveal_radius']**2
        cell=int(position[1]/config['cell_size'])*cols+int(position[0]/config['cell_size'])
        visible[cell]=True;known[visible]=truth[visible];private.append(known)
    for line in (directory/'events.jsonl').open():
        event=json.loads(line);kind=event['kind']
        if kind=='reveal':
            private[event['robot_id']][event['cells']]=truth[event['cells']]
        elif kind=='publish':
            message=event['message']
            assert message['cells']==[]
            for edge in message['roadmap_edges']:
                assert edge['weight']==0.0
                assert tuple(sorted((edge['from_cell_id'],edge['to_cell_id']))) in graphs[message['sender']]
            assert len(message['robot_positions'])==len(message['robot_update_ticks'])==config['robots']
            published[message['sender'],message['sequence']]=message
            checks+=1
        elif kind=='receive':
            assert event['newly_learned_cells']==[]
            message=published[event['sender'],event['sequence']]
            assert event['roadmap_edges']==message['roadmap_edges']
        elif kind=='roadmap_edge_discovered':
            robot=event['robot_id'];path=event['known_free_path']
            assert all(private[robot][c]==0 for c in path)
            assert (len(path)-1)*config['cell_size']<=2*config['region_size']
            for a,b in zip(path,path[1:]):
                ar,ac=divmod(a,cols);br,bc=divmod(b,cols)
                assert abs(ar-br)+abs(ac-bc)==1
            graphs[robot].add(tuple(sorted((event['from_cell_id'],event['to_cell_id']))))
        elif kind=='roadmap_edge_received':
            message=published[event['sender'],event['sequence']]
            assert any(edge['from_cell_id']==event['from_cell_id'] and edge['to_cell_id']==event['to_cell_id'] for edge in message['roadmap_edges'])
            graphs[event['robot_id']].add(tuple(sorted((event['from_cell_id'],event['to_cell_id']))))
        elif kind=='position_forwarded':
            message=published[event['sender'],event['sequence']]
            assert event['position']==message['robot_positions'][event['teammate']]
            assert event['position_tick']==message['robot_update_ticks'][event['teammate']]
    snapshots=json.loads((directory/'roadmaps.json').read_text())
    for robot in range(config['robots']):
        np.testing.assert_array_equal(private[robot],arrays[f'robot_{robot}'].ravel())
        assert graphs[robot]=={tuple(sorted((e['from_cell_id'],e['to_cell_id']))) for e in snapshots[str(robot)]}
    return dict(compact_publications=checks,private_maps_verified=config['robots'],
                final_edges=[len(graphs[i]) for i in range(config['robots'])])


def audit_sample_selection(record, config, scene):
    normal={int(k):set(v) for k,v in record['viewpoint_coverage'].items()}
    front={int(k):set(v) for k,v in record['viewpoint_frontier_coverage'].items()}
    ordinary=set(record['ordinary_sample_cells']);frontier=set(record['frontier_sample_cells'])
    assert not ordinary & frontier
    visited=set(record['visited_viewpoint_cells'])
    cols=scene['cols'];scale=config['cell_size']
    def xy(cell):
        if record.get('coverage_model') in ('mtare_dense_viewpoints','simple_frontier'):
            from verify_dense import node_xy
            return node_xy(cell,config,scene)
        return ((cell%cols+.5)*scale,(cell//cols+.5)*scale)
    state=record['coverage_state']
    if record.get('coverage_model')=='simple_frontier':
        assert state=={} and not ordinary
        seen,covered=set(),set();radius=0
    else:
        seen=set(state['observed_sample_cells']);covered=set(state['covered_sample_cells'])
        assert covered <= seen
        assert set(state['uncovered_sample_cells'])==seen-covered
        radius=state['coverage_radius']
    stride=round(config['region_size']/scale);region_cols=math.ceil(cols/stride)
    def region(c):
        if record.get('coverage_model') in ('mtare_dense_viewpoints','simple_frontier'):
            from verify_dense import node_region
            return node_region(c,config,scene)
        return ((c//cols)//stride)*region_cols+(c%cols)//stride
    uncovered={c for c in seen-covered if region(c) in record['local_window_region_ids']}
    peer=record.get('peer_coverage')
    if peer:
        assert config.get('respect_peer_coverage',False)
        assert peer['model']=='peer_region_completion_v1'
        peer_regions=set(peer['region_ids'])
        for reg,status in record['region_states_before'].items():
            assert (int(reg) in peer_regions)==(status==3)
        ignored={c for c in uncovered if region(c) in peer_regions}
        assert set(peer['suppressed_ordinary_sample_cells'])==ignored
        uncovered-=ignored
        assert not any(region(c) in peer_regions for c in ordinary|frontier)
    supported=set();supported_front=set()
    for viewpoint,points in normal.items():
        expected=set() if viewpoint in visited else {c for c in uncovered if math.dist(xy(c),xy(viewpoint))<=radius+1e-9}
        assert points==expected,(record['local_plan_id'],viewpoint,'ordinary')
        expected_front=set() if viewpoint in visited else {c for c in frontier if math.dist(xy(c),xy(viewpoint))<=config['reveal_radius']+1e-9}
        assert front[viewpoint]==expected_front,(record['local_plan_id'],viewpoint,'frontier')
        supported|=points;supported_front|=front[viewpoint]
    assert ordinary==supported and frontier==supported_front
    weight=record['score_thresholds'].get('sample_weight',1.0)
    small=record['score_thresholds']['ordinary_min']
    big=record['score_thresholds'].get('ordinary_big',config['region_big_gain'])
    frontier_threshold=record['score_thresholds']['frontier_min']
    for region_id, evidence in record.get('region_coverage_evidence',{}).items():
        points=[c for c in normal if region(c)==int(region_id)]
        unvisited=[c for c in points if c not in visited]
        assert evidence['candidates']==len(points)
        assert evidence['selected']==len(set(points)&set(record['selected_viewpoints']))
        assert evidence['above_small']==sum(len(normal[c])*weight>=small for c in unvisited)
        assert evidence['above_big']==sum(len(normal[c])*weight>=big for c in unvisited)
        expected_frontier=sum(len(front[c])>=record['score_thresholds']['frontier_evidence_min']
            if 'frontier_evidence_min' in record['score_thresholds'] else
            len(front[c])*weight>frontier_threshold for c in unvisited)
        assert evidence['above_frontier']==expected_frontier
    thresholds=record['score_thresholds'];nmin,fmin=thresholds['ordinary_min'],thresholds['frontier_min']
    eligible=set(record['eligible_viewpoint_cells']);reused=record['reused_viewpoints']
    assert all(c in eligible and c not in visited and (len(normal[c])*weight>=nmin or len(front[c])*weight>=fmin) for c in reused)
    if 'previous_viewpoints' in record:
        expected=[c for c in record['previous_viewpoints'] if c in eligible and c not in visited and c not in (record['start_cell'],record['end_cell']) and (len(normal[c])*weight>=nmin or len(front[c])*weight>=fmin)]
        assert reused==expected
    pre=set(reused)|set(record['navigation_viewpoints'])
    pn=set().union(*(normal.get(c,set()) for c in pre));pf=set().union(*(front.get(c,set()) for c in pre))
    nq={c for c in eligible-pre-visited if len(normal[c]-pn)*weight>=nmin}
    fq={c for c in eligible-pre-visited-nq if len(front[c]-pf)*weight>=fmin}
    assert nq==set(record['ordinary_queue']) and fq==set(record['frontier_queue'])
    gate=bool(fq and (max(len(front[c]-pf) for c in fq)>=thresholds['frontier_gate_min']
                     if 'frontier_gate_min' in thresholds else
                     max(len(front[c]-pf)*weight for c in fq)>fmin))
    assert gate==record['frontier_gate']
    assert len(record['trials'])==(config['local_optimization_iterations'] if nq else 1)
    def steps_check(queue,lists,mask,steps,threshold,continuation=None):
        queue=set(queue);mask=set(mask)
        for index,step in enumerate(steps):
            minimum=max(threshold,continuation or threshold) if index else threshold
            ranked=sorted(((len(lists[c]-mask)*weight,c) for c in queue),key=lambda x:(-x[0],x[1]))
            choices=[item for item in ranked if item[0]>=minimum][:config['local_greedy_sample_range']]
            assert (step['gain'],step['viewpoint']) in choices
            c=step['viewpoint'];assert step['gain']==len(lists[c]-mask)*weight
            queue.remove(c);mask|=lists[c]
        minimum=max(threshold,continuation or threshold) if steps else threshold
        assert not any(len(lists[c]-mask)*weight>=minimum for c in queue)
        return mask
    for trial in record['trials']:
        mask=steps_check(nq,normal,pn,trial['ordinary_selection_steps'],nmin)
        ns=[step['viewpoint'] for step in trial['ordinary_selection_steps']]
        assert ns==trial['ordinary_selected']
        frontier_mask=pf|set().union(*(front[c] for c in ns))
        if gate:frontier_mask=steps_check(fq,front,frontier_mask,trial['frontier_selection_steps'],fmin,nmin)
        else:assert not trial['frontier_selection_steps']
        fs=[step['viewpoint'] for step in trial['frontier_selection_steps']]
        assert fs==trial['frontier_selected']
        assert trial['selected_viewpoints']==list(dict.fromkeys([*ns,*fs,*reused]))
        assert trial['predicted_ordinary_samples']==len(mask)
        assert trial['predicted_frontier_samples']==len(frontier_mask)
    chosen=record['trials'][record['selected_trial']]
    assert chosen['route_length']==min(t['route_length'] for t in record['trials'])==record['route']['length']
    assert set(record['selected_viewpoints'])==set(chosen['selected_viewpoints'])
    assert record['local_coverage_complete']==(not record['selected_viewpoints'])


def audit_coverage_replay(directory,config,scene,local_records):
    path=directory/'coverage_bookkeeping.json'
    if not path.exists() or not any(json.loads(path.read_text()).values()):return {}
    cols=scene['cols'];rows=scene['rows'];size=cols*rows;scale=config['cell_size'];vehicles=config['robots']
    centres=np.array([((i%cols+.5)*scale,(i//cols+.5)*scale) for i in range(size)])
    truth=np.load(directory/'maps.npz')['truth'].ravel()
    radius=config.get('coverage_radius')
    if radius is None:radius=2*config['reveal_radius']/3
    known=[];physical=[];observed=[set() for _ in range(vehicles)];covered=[set() for _ in range(vehicles)]
    for position in scene['starts']:
        visible=np.sum((centres-np.asarray(position))**2,axis=1)<=config['reveal_radius']**2
        visible[int(position[1]/scale)*cols+int(position[0]/scale)]=True
        grid=np.full(size,-1);grid[visible]=truth[visible];known.append(grid)
        physical.append(np.sum((centres-np.asarray(position))**2,axis=1)<=radius**2)
    def neighbors(c):
        r,k=divmod(c,cols)
        return ([c-cols] if r else [])+([c+cols] if r+1<rows else [])+([c-1] if k else [])+([c+1] if k+1<cols else [])
    stride=round(config['region_size']/scale);region_cols=math.ceil(cols/stride)
    def region(c):return ((c//cols)//stride)*region_cols+(c%cols)//stride
    peer_region_checks=0
    states=[[0]*(scene['region_rows']*scene['region_cols']) for _ in range(vehicles)]
    plan_peer_evidence={}
    positions=[tuple(p) for p in scene['starts']]
    priority=[set() for _ in range(vehicles)]
    peer_work_cache={}
    direction_checks=0
    global_context={}
    if config.get('use_lookahead',False):
        for p in (directory/'calls').glob('*.json'):
            g=json.loads(p.read_text())
            global_context[g['robot_id'],g['tick']]=g['connected_peers']
    updates=checked=0
    for line in (directory/'events.jsonl').open():
        event=json.loads(line);kind=event['kind']
        if kind=='reveal':
            known[event['robot_id']][event['cells']]=truth[event['cells']]
        elif kind=='move':
            physical[event['robot_id']]|=np.sum((centres-np.asarray(event['position']))**2,axis=1)<=radius**2
            positions[event['robot_id']]=tuple(event['position'])
        elif kind=='receive':
            peer_work_cache[event['receiver'],event['sender']]=event.get('local_exploring_region_ids',[])
        elif kind=='local_priority_updated':
            robot=event['robot_id']
            assert sorted(map(int,event['peer_work']))==sorted(global_context[robot,event['tick']])
            for sender,ids in event['peer_work'].items():
                assert ids==peer_work_cache[robot,int(sender)]
            expected={r for r in event['window'] if any(int(sender)<robot and r in ids
                      for sender,ids in event['peer_work'].items())}
            assert set(event['deprioritized'])==expected
            priority[robot]=expected
        elif kind=='region_state_changed':
            states[event['robot_id']][event['region_id']]=event['status']
        elif kind=='peer_coverage_plan_evidence':
            robot=event['robot_id'];peer=event['peer_coverage']
            completed={i for i,status in enumerate(states[robot]) if status==3}
            assert set(peer['region_ids'])==completed
            plan_peer_evidence[robot]=peer
        elif kind=='coverage_samples_updated':
            robot=event['robot_id'];now=set(np.flatnonzero(known[robot]==0).tolist())
            assert set(event['observed_sample_cells'])==now-observed[robot]
            observed[robot]|=now
            if event.get('coverage_source')=='visited_viewpoints':
                from verify_dense import node_xy
                visible=set()
                for node in event['visited_viewpoint_ids']:
                    point=node_xy(node,config,scene)
                    assert known[robot][int(point[1]/scale)*cols+int(point[0]/scale)]==0
                    visible.update(np.flatnonzero(np.sum((centres-point)**2,axis=1)<=radius**2).tolist())
                actual=covered[robot]|(visible&observed[robot])
            else:
                actual=covered[robot]|{c for c in observed[robot] if physical[robot][c]}
            assert set(event['covered_sample_cells'])==actual-covered[robot]
            covered[robot]|=actual;updates+=1
        elif kind=='local_plan_selected':
            record=local_records[event['local_plan_id']]
            if record.get('coverage_model') not in ('mtare_samples','mtare_dense_viewpoints'):continue
            robot=event['robot_id'];state=record['coverage_state']
            if record.get('lookahead'):
                from verify_lookahead import audit_direction
                audit_direction(record,config,scene,known[robot],positions[robot],priority[robot])
                direction_checks+=1
            assert set(state['observed_sample_cells'])==observed[robot]
            assert set(state['covered_sample_cells'])==covered[robot]
            grid=known[robot];window=set(record['local_window_region_ids'])
            raw={other for c in np.flatnonzero(grid==0).tolist() if region(c) in window
                 and any(grid[n]==-1 for n in neighbors(c)) for other in neighbors(c) if grid[other]==-1}
            peer=record.get('peer_coverage')
            if peer:
                assert peer==plan_peer_evidence[robot]
                completed=set(peer['region_ids'])
                ignored={c for c in raw if region(c) in completed}
                assert set(peer['suppressed_frontier_sample_cells'])==ignored
                raw-=ignored
                peer_region_checks+=1
            from verify_dense import node_xy
            visited=set(record['visited_viewpoint_cells']);support=set()
            for candidate in record['local_candidate_cells']:
                if candidate not in visited:
                    support|={c for c in raw if math.dist(centres[c], (node_xy(candidate,config,scene) if record.get('coverage_model')=='mtare_dense_viewpoints' else centres[candidate]))<=config['reveal_radius']+1e-9}
            assert set(record['frontier_sample_cells'])==support
            checked+=1
    snapshots=json.loads(path.read_text())
    for robot in range(vehicles):
        assert set(snapshots[str(robot)]['observed_sample_cells'])==observed[robot]
        assert set(snapshots[str(robot)]['covered_sample_cells'])==covered[robot]
        metrics=json.loads((directory/'summary.json').read_text())['final']['coverage_samples_per_robot'][robot]
        assert metrics==dict(observed=len(observed[robot]),covered=len(covered[robot]),uncovered=len(observed[robot]-covered[robot]))
    return dict(coverage_update_events=updates,verified_sample_calls=checked,robots=vehicles,
                peer_region_checks=peer_region_checks,lookahead_checks=direction_checks)


def audit(directory):
    directory = Path(directory)
    config = json.loads((directory / "config.json").read_text())
    summary = json.loads((directory / "summary.json").read_text())
    scene = json.loads((directory / "scene.json").read_text())
    vehicles = config["robots"]
    count = 0
    roadmap_cost_entries = 0
    for filename in sorted((directory / "calls").glob("*.json")):
        record = json.loads(filename.read_text())
        roadmap_cost_entries += audit_roadmap_costs(record,config,scene)
        matrices = record["distance_matrices"]
        routes = record["result"]["routes"]
        costs = []
        assigned = []
        for vehicle, row in enumerate(routes):
            closed = [vehicle, *row, vehicle]
            costs.append(sum(matrices[vehicle][a][b] for a, b in zip(closed, closed[1:])))
            assigned.extend(row)
        assert len(assigned) == len(set(assigned)), filename
        dropped = set(range(vehicles, len(record["node_cell_ids"]))) - set(assigned)
        assert dropped == set(record["result"]["dropped_node_indices"]), filename
        settings = record["settings"]
        objective = sum(costs)+settings["span_coefficient"]*max(costs)+settings["drop_penalty"]*len(dropped)
        assert objective == record["result"]["objective"], filename
        assert costs == record["result"]["route_costs"], filename
        hierarchy=record.get("hierarchy")
        if hierarchy and hierarchy.get("state_model")=="mtare":
            assert not set(hierarchy['global_region_ids']) & set(hierarchy['excluded_peer_local_region_ids'])
            assert all(hierarchy['active_region_states'][str(i)] in (1,5) for i in hierarchy['global_region_ids'])
        candidates = record["candidate_comparisons"]
        if candidates:
            winner = min(candidates, key=lambda c: (c["dropped_count"], c["longest_route_cost"], c["source"]))
            assert winner["source"] == record["selected_seed_source"], filename
            for candidate in candidates:
                candidate_costs = []
                candidate_nodes = set()
                for vehicle, row in enumerate(candidate["initial_routes"]):
                    route = [vehicle, *row, vehicle]
                    candidate_costs.append(sum(matrices[vehicle][a][b] for a, b in zip(route, route[1:])))
                    candidate_nodes.update(row)
                assert max(candidate_costs) == candidate["longest_route_cost"], filename
                assert len(record["target_cell_ids"])-len(candidate_nodes) == candidate["dropped_count"], filename
        count += 1
    local_records = audit_local_calls(directory, config, scene)
    active_local_paths = {}
    received = {}
    positions = [tuple(p) for p in scene["starts"]]
    travelled = [0.0]*vehicles
    movements, verified_peer_seeds = 0, 0
    completion_event = None
    maps = np.load(directory / "maps.npz")
    truth = maps["truth"].ravel()
    cols = scene["cols"]
    for line in (directory / "events.jsonl").open():
        event = json.loads(line)
        if event["kind"] == "planner_completion_confirmed":
            completion_event = event
        if event["kind"] == "local_plan_selected":
            record = local_records[event["local_plan_id"]]
            assert event["execution_path"] == record["route"]["execution_path"]
            active_local_paths[event["robot_id"]] = (event["local_plan_id"], deque(event["execution_path"]))
        elif event["kind"] == "receive":
            received[event["receiver"], event["sender"]] = event["proposal_id"]
        elif event["kind"] == "plan_selected" and event["peer_seed_used"]:
            record = json.loads((directory / "calls" / (event["call_id"]+".json")).read_text())
            assert received[event["robot_id"], event["selected_seed_source"]] == record["selected_seed_proposal_id"]
            verified_peer_seeds += 1
        elif event["kind"] == "move":
            left, right = event["from_cell"], event["to_cell"]
            if event.get("planning_level") == "local":
                plan_id, remaining = active_local_paths[event["robot_id"]]
                assert event["local_plan_id"] == plan_id
                assert remaining.popleft() == event.get('viewpoint_id',right)
            lr, lc = divmod(left, cols)
            rr, rc = divmod(right, cols)
            if 'viewpoint_id' in event:
                from verify_dense import node_xy
                assert np.allclose(event['position'],node_xy(event['viewpoint_id'],config,scene))
                assert abs(lr-rr)+abs(lc-rc)<=2
            else:
                assert abs(lr-rr)+abs(lc-rc) == 1
            assert truth[right] == 0
            robot = event["robot_id"]
            distance = math.dist(positions[robot], event["position"])
            assert abs(distance-event["distance"]) < 1e-8
            travelled[robot] += distance
            positions[robot] = tuple(event["position"])
            movements += 1
    assert np.allclose(travelled, summary["final"]["per_robot_distance"])
    assert verified_peer_seeds == summary["final"]["peer_seed_uses"]
    assert count == summary["final"]["vrp_calls"]
    assert len(local_records) == summary["final"].get("local_planning_calls", 0)
    state_checks=audit_region_states(directory,config)
    protocol_checks=audit_compact_protocol(directory,config,scene)
    coverage_checks=audit_coverage_replay(directory,config,scene,local_records)
    planner_completion_verified = False
    if summary["reason"] == "no_remaining_exploration_work":
        assert summary['status'] == 'finished' and completion_event is not None
        assert summary['final']['cycle'] == completion_event['tick'] + 1
        if completion_event.get('rule') == 'mtare_no_global_tasks_and_local_complete':
            # Independently reconstruct source-style checks from saved input lists
            # and local solver outputs, not region statuses or the truth metric.
            assert completion_event['tick'] > config['completion_startup_cycles']
            final_plans = json.loads((directory/'final_planning_state.json').read_text())
            global_records = [json.loads(p.read_text()) for p in (directory/'calls').glob('*.json')]
            for robot_id in range(vehicles):
                global_call = max((r for r in global_records if r['robot_id']==robot_id),
                                  key=lambda r:r['tick'])
                local_call = max((r for r in local_records.values() if r['robot_id']==robot_id),
                                 key=lambda r:r['tick'])
                assert global_call['tick'] == local_call['tick'] == completion_event['tick']
                assert not global_call['hierarchy']['global_region_ids']
                assert local_call['local_coverage_complete']
                plan = final_plans[str(robot_id)]
                assert not plan['global_exploring_regions']
                assert plan['local_coverage_complete'] and plan['exploration_finished']
            assert all(summary['final']['robot_exploration_finished'])
        elif completion_event.get('rule') == 'coverage_threshold_and_no_executable_work':
            assert completion_event['tick'] > config['completion_startup_cycles']
            minimum = config['completion_min_coverage']
            assert 0 < minimum <= 1
            assert summary['final']['explored_ratio'] >= minimum
            assert math.isclose(completion_event['observed_coverage'],
                                summary['final']['explored_ratio'])
            # Recompute the evaluator coverage from saved observations and the
            # seeded world, independently of the event's claimed percentage.
            from mtare_2d.config import Config
            from mtare_2d.world import World
            from dataclasses import fields
            config_keys = {field.name for field in fields(Config)}
            world = World(Config(**{key:value for key,value in config.items() if key in config_keys}))
            observed = np.logical_or.reduce([maps[f'robot_{i}'] == 0 for i in range(vehicles)])
            reachable = world.reachable_truth.ravel()
            denominator = np.count_nonzero(reachable)
            measured = np.count_nonzero(observed.ravel() & reachable) / denominator if denominator else 1.0
            assert math.isclose(measured, completion_event['observed_coverage'])
            final_plans = json.loads((directory/'final_planning_state.json').read_text())
            global_records = [json.loads(p.read_text()) for p in (directory/'calls').glob('*.json')]
            for robot_id in range(vehicles):
                global_call = max((r for r in global_records if r['robot_id']==robot_id), key=lambda r:r['tick'])
                local_call = max((r for r in local_records.values() if r['robot_id']==robot_id), key=lambda r:r['tick'])
                assert global_call['tick'] == local_call['tick'] == completion_event['tick']
                assert not global_call['selected_own_tasks']
                assert not global_call['result']['routes'][robot_id]
                assert local_call['local_coverage_complete'] and not local_call['selected_viewpoints']
                # Dense local paths use viewpoint-node coordinates; ordinary
                # grid paths use cell centres. Ignore only zero-distance nodes.
                assert all(math.dist(positions[robot_id], world.xy(node)) <= 1e-9
                           for node in local_call['route']['execution_path'])
                plan = final_plans[str(robot_id)]
                assert not plan['selected_tasks'] and not plan['local_viewpoints']
                assert plan['local_coverage_complete'] and plan['exploration_finished']
            assert all(summary['final']['robot_exploration_finished'])
        else:  # Preserve audits of historical stable-idle runs.
            assert completion_event['tick'] - completion_event['idle_since'] >= max(
                config['plan_every'], config['message_delay'] + config['publish_every'])
            states = json.loads((directory/'region_states.json').read_text())
            final_plans = json.loads((directory/'final_planning_state.json').read_text())
            for robot_id in range(vehicles):
                assert all(cell['status'] not in (1,5) for cell in states[str(robot_id)])
                final_plan = final_plans[str(robot_id)]
                assert not any(final_plan[k] for k in ('selected_tasks','local_viewpoints','local_path'))
                records = [r for r in local_records.values() if r['robot_id'] == robot_id]
                latest = max(records, key=lambda r:r['tick'])
                assert latest['tick'] == completion_event['tick'] and latest['local_coverage_complete']
            assert all(n == 0 for n in summary['final']['last_dropped_tasks_per_robot'])
        planner_completion_verified = True
    from verify_frontier import audit_frontier_replay
    frontier_checks=audit_frontier_replay(directory,config,scene)
    from verify_dense import audit_visited_masks
    dense_checks=audit_visited_masks(directory,config,scene)
    result = dict(status="passed",frontier_checks=frontier_checks,dense_viewpoint_checks=dense_checks, region_state_checks=state_checks, coverage_checks=coverage_checks,
                  roadmap_cost_entries=roadmap_cost_entries,protocol_checks=protocol_checks, verified_local_coverage_calls=len(local_records), planning_calls=count, verified_peer_seed_receipts=verified_peer_seeds,
                  verified_movements=movements, planner_completion_verified=planner_completion_verified,
                  objective_failures=0)
    (directory / "validation.json").write_text(json.dumps(result, indent=2)+"\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.run), indent=2))
