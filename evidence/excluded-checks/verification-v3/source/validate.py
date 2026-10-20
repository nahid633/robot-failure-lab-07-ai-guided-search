"""Recompute labels, packet origins and independent observation coverage."""
import base64,csv,json,math
from collections import Counter
from pathlib import Path
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
import core
from observer import payload_hash
from search import HORIZON


def rows(path):
    with Path(path).open() as f:
        return [{k:(v if k in ('branch','payload_hash','frame','child','polygon') else float(v)) for k,v in r.items()} for r in csv.DictReader(f)]


def validate_trial(root,r):
    p=root/r['trial'];s=r['scenario'];core.configure(s);start=r['window_start'];end=r['observation_end'];cut=r['cutoff'];appearance=r['appearance'];duration=s['duration']
    assert cut==start+HORIZON and end>=r['observed_cutoff']+1.8-.02 and cut<=r['observed_cutoff']<cut+.25
    assert r['goal_accepted'] and r['obstacle_service_success']
    assert start+.20<=r['obstacle_request_time']<=appearance+.05 and start+.20<=appearance<start+1.
    assert core.routing_valid(r['graph']) and core.routing_valid(r['graph_end'])
    before=json.loads((p/'effective-before.json').read_text());assert before==json.loads((p/'effective-after.json').read_text())
    assert before['/controller_server']['general_goal_checker.xy_goal_tolerance']==.25 and before['/collision_monitor']['source_timeout']==1.
    scans=rows(p/'scans.csv');pub=rows(p/'published.csv');rx=rows(p/'receipts.csv');truth=rows(p/'world.csv');obstacle=rows(p/'obstacle.csv');odom=rows(p/'odom.csv')
    fault=json.loads((p/'FAULT_START.json').read_text());assert fault['scenario']==s
    by_seq={int(z['seq']):z for z in scans};cached=int(fault['cached_origin'])
    assert by_seq[cached]['stamp']==fault['cached_stamp'] and by_seq[cached]['payload_hash']==fault['cached_hash']
    raw={int(z['seq']):deserialize_message(base64.b64decode(z['cdr']),LaserScan) for z in map(json.loads,(p/'raw-scans.jsonl').read_text().splitlines())}
    out={int(z['seq']):deserialize_message(base64.b64decode(z['cdr']),LaserScan) for z in map(json.loads,(p/'published-scans.jsonl').read_text().splitlines())}
    assert set(raw)==set(by_seq) and len(pub)==len(out)
    assert Counter(int(z['origin']) for z in pub if z['branch']=='localization')==Counter(by_seq.keys())
    for seq,msg in raw.items():
        assert payload_hash(msg)==by_seq[seq]['payload_hash']
        assert abs(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9-by_seq[seq]['stamp'])<1e-8
    for z in pub:
        source=by_seq[int(z['origin'])];msg=out[int(z['seq'])]
        assert payload_hash(msg)==z['payload_hash']==source['payload_hash'] and z['origin_stamp']==source['stamp']
        assert abs(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9-z['stamp'])<1e-8
        assert -.001<=z['t']-z['policy_t']<.05
        if z['branch']=='obstacles' and start<=z['policy_t']<start+duration:
            assert int(z['origin'])==cached and z['stamp']!=z['origin_stamp']
            assert any(q['stamp']==z['stamp'] and abs(q['t']-z['t'])<.05 for q in scans)
        else:assert z['stamp']==z['origin_stamp'] and abs(source['t']-z['t'])<.05
    counts=Counter(int(z['seq']) for z in rx)
    assert all(counts[int(z['seq'])]==1 for z in pub if z['t']<end-.25)
    inside=[z for z in pub if z['branch']=='obstacles' and start<=z['policy_t']<start+duration]
    assert len(inside)==sum(start<=z['t']<start+duration for z in scans)
    if duration>=.4:assert len(inside)>=1
    restored=[z for z in pub if z['branch']=='obstacles' and z['t']>=start+duration]
    assert restored and restored[0]['t']<start+duration+.35
    coverage={}
    for name,data,gap in [('world',truth,.15),('odom',odom,.15),('obstacle',obstacle,.15),('scans',scans,.35)]:
        group=[z for z in data if start-.3<=z['stamp']<=end]
        assert group and group[0]['stamp']<=start and end-group[-1]['stamp']<gap
        gaps=[b['stamp']-a['stamp'] for a,b in zip(group,group[1:])]
        assert gaps and min(gaps)>=0 and max(gaps)<gap,(name,max(gaps))
        assert all(-.02<=z['t']-z['stamp']<gap for z in group)
        coverage[name]=dict(samples=len(group),max_gap_s=max(gaps))
    assert all(math.hypot(z['x']+8,z['y']+8)<.005 for z in obstacle if z['stamp']<r['obstacle_request_time'])
    assert all(math.hypot(z['x']-core.OBSTACLE[0],z['y']-core.OBSTACLE[1])<.005 for z in obstacle if z['stamp']>=appearance)
    assert core.clearance(min(truth,key=lambda z:abs(z['stamp']-appearance)))>.1
    hits=[z for z in scans if appearance<=z['stamp']<=min(cut,appearance+1.) and z['obstacle_hits']>0]
    assert hits,'Original upstream sensor did not see introduced obstacle'
    events=list(map(json.loads,(p/'contacts.jsonl').read_text().splitlines()))
    contact=[z for z in events if z['robot_obstacle_pairs'] and appearance<=z['stamp']<=cut]
    assert int(bool(contact))==r['label'] and len(contact)==r['contact_messages_in_label_window']
    assert r['first_contact_after_appearance_s']==(contact[0]['stamp']-appearance if contact else None)
    assert r['min_sampled_envelope_clearance_m']==min(core.clearance(z) for z in truth if appearance<=z['stamp']<=cut)
    stopped=core.sustained_stop(odom,appearance,cut)
    assert r['stop_after_appearance_s']==(None if stopped is None else stopped-appearance)
    return dict(passed=True,coverage=coverage,upstream_obstacle_hit_scans=len(hits),fault_messages=len(inside),
        positive_contact_label=r['label'],cancellation_ack_missing=r['cancel_requested'] and r['terminal_action_status'] is None,
        label_scope='Observed obstacle-specific contact by onset+6s; event absence is not a physical no-contact certificate.')
