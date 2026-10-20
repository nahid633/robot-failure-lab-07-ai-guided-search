"""Coordination amendment: reconstruct round-robin cursor without rerunning outcomes."""
import argparse,hashlib,json,sys,time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root.resolve()
sys.path.insert(0,str(root/'source'))
import run as frozen
from search import METHODS,choose,confirmed,training
from validate import validate_trial

protocol=json.loads((root/'PROTOCOL.json').read_text());assert protocol['phase']=='main'
for name,digest in protocol['source_sha256'].items():assert frozen.sha(root/'source'/name)==digest
for name,digest in protocol['input_sha256'].items():assert frozen.sha(root/name)==digest
records=json.loads((root/'results.json').read_text())
for r in records:assert r['evidence_validated'];validate_trial(root,r)
warm=[r for r in records if r['role']=='training'];assert len(warm)==9 and [r['scenario'] for r in warm]==training()
usable=[r for r in warm if r['label'] is not None];assert len(usable)>=6 and {r['label'] for r in usable}=={0,1}
adapter=root/'continue-round-robin.py'
if not adapter.exists():adapter.write_bytes(Path(__file__).read_bytes())
assert adapter.read_bytes()==Path(__file__).read_bytes()
amendment=root/'COORDINATION-AMENDMENT.json'
if not amendment.exists():
    frozen.write(amendment,dict(date='2026-10-20',preserved_outcomes=[r['trial'] for r in records],
        reason='Original --resume loop restarts at first eligible policy instead of interrupted round-robin slot. Reconstruct declared order by replaying prior outcomes, then execute the pending saved request.',
        unchanged='Original frozen worker/model/policy/configuration/source files; no completed outcome retried. Same declared policy order, budgets, scenarios, confirmation and visibility rules.',
        adapter_sha256=frozen.sha(adapter),new_protocol_scope='Coordination-only amendment to implement the existing declared sequence; not a change of treatments or selected scenarios.'))
assert json.loads(amendment.read_text())['adapter_sha256']==frozen.sha(adapter)
frozen.write(root/f'continuation-adapter-{time.time_ns()}.json',dict(command=sys.argv,preserved=[r['trial'] for r in records]))
remaining=[r for r in records if r['role']=='search'];replayed=[];cursor=0
for episode in (1,2,3):
    order=METHODS[episode-1:]+METHODS[:episode-1]
    while True:
        eligible=False
        for method in order:
            history=[r for r in replayed if r.get('episode')==episode and r.get('method')==method]
            if confirmed(history) or len(history)>=(6 if method=='model' else 15):continue
            eligible=True;s,confirmation,decision=choose(method,episode,warm,history)
            request=dict(role='search',episode=episode,method=method,scenario=s,seed=(300 if confirmation else 200)+episode,confirmation=confirmation,decision=decision)
            if cursor<len(remaining):
                r=remaining[cursor];cursor+=1
                for key in ('role','episode','method','scenario','seed','confirmation'):assert r[key]==request[key],('Prior chronology violates protocol',key,r['trial'])
            else:r=frozen.execute(root,records,request,True)
            replayed.append(r)
        if not eligible:break
assert cursor==len(remaining)
frozen.write(root/'COMPLETE.json',dict(outcomes=len(records),phase='main',coordination_amendment='COORDINATION-AMENDMENT.json'))
print('COMPLETE {} outcomes; declared round-robin chronology preserved.'.format(len(records)),flush=True)
