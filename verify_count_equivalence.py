#!/usr/bin/env python3
"""Audit historical weighted local decisions after exact conversion to counts.

Historical weights appear only in this migration check, never in the planner.
No input run files are changed.
"""
import argparse
import json
import math
from pathlib import Path
from verify_run import audit_sample_selection


def audit(run):
    run=Path(run);config=json.loads((run/'config.json').read_text());scene=json.loads((run/'scene.json').read_text())
    checked=ordinary=frontier=0
    for path in sorted((run/'local_calls').glob('*.json')):
        record=json.loads(path.read_text())
        if record.get('coverage_model')!='mtare_dense_viewpoints':continue
        thresholds=record['score_thresholds'];weight=thresholds.get('sample_weight')
        assert weight is not None and math.isclose(weight,4/.09)
        assert (thresholds['ordinary_min'],thresholds['ordinary_big'],thresholds['frontier_min'])==(40,60,20)
        record['score_thresholds']=dict(ordinary_min=1,ordinary_big=2,frontier_min=1,
            frontier_evidence_min=1,frontier_gate_min=1,score_units='sample_count')
        for trial in record['trials']:
            for key in ('ordinary_selection_steps','frontier_selection_steps'):
                for step in trial[key]:
                    raw=step['gain']/weight
                    assert math.isclose(raw,round(raw),abs_tol=1e-8)
                    step['gain']=round(raw)
        audit_sample_selection(record,config,scene)
        checked+=1;ordinary+=bool(record['ordinary_queue']);frontier+=bool(record['frontier_gate'])
    assert checked
    return dict(status='passed',source_run=str(run),verified_local_calls=checked,
                ordinary_queue_calls=ordinary,frontier_gate_calls=frontier,
                note='Recorded weighted choices, candidate queues, marginal steps and region evidence remain valid under count comparisons. No source records or routes changed.')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    print(json.dumps(audit(parser.parse_args().run),indent=2))
