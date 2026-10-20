"""Retained CDR provenance, routing, geometry and outcome checks."""
import base64,csv,json,math
from collections import Counter
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan
from core import *
from support import endpoint_metrics
from trial import payload_hash


def rows(path):
    with path.open() as f:return [{k:(v if k in ('branch','payload_hash','frame','child','polygon') else float(v)) for k,v in r.items()} for r in csv.DictReader(f)]


def validate_trial(root,r):
    p=root/r['trial'];start=r['window_start'];end=r['observation_end'];appearance=r['appearance']
    assert r['goal_accepted'] and r['terminal_action_status'] in (4,5,6)
    assert r['obstacle_service_success'] and start+.20<=r['obstacle_request_time']<=appearance+.05
    assert start+.20<=appearance<start+1.,('appearance timing',appearance-start)
    assert end>=start+6 and end>=r['end_sim_s']+1.8-.02
    assert routing_valid(r['graph']) and routing_valid(r['graph_end'])
    before=json.loads((p/'effective-before.json').read_text());assert before==json.loads((p/'effective-after.json').read_text())
    assert before['/controller_server']['general_goal_checker.xy_goal_tolerance']==.25
    assert before['/collision_monitor']['source_timeout']==1.
    scans=rows(p/'scans.csv');pub=rows(p/'published.csv');rx=rows(p/'receipts.csv');truth=rows(p/'world.csv');obstacle=rows(p/'obstacle.csv');odom=rows(p/'odom.csv')
    fault=json.loads((p/'FAULT_START.json').read_text());cached=int(fault['cached_origin'])
    by_seq={int(s['seq']):s for s in scans}
    assert by_seq[cached]['stamp']==fault['cached_stamp'] and by_seq[cached]['payload_hash']==fault['cached_hash']
    raw={int(s['seq']):deserialize_message(base64.b64decode(s['cdr']),LaserScan) for s in map(json.loads,(p/'raw-scans.jsonl').read_text().splitlines())}
    out={int(s['seq']):deserialize_message(base64.b64decode(s['cdr']),LaserScan) for s in map(json.loads,(p/'published-scans.jsonl').read_text().splitlines())}
    assert set(raw)==set(by_seq) and len(pub)==len(out)
    localization=[s for s in pub if s['branch']=='localization']
    assert Counter(int(s['origin']) for s in localization)==Counter(by_seq.keys())
    for seq,msg in raw.items():
        assert payload_hash(msg)==by_seq[seq]['payload_hash']
        assert abs(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9-by_seq[seq]['stamp'])<1e-8
    for row in pub:
        seq=int(row['seq']);origin=int(row['origin']);message=out[seq]
        assert payload_hash(message)==row['payload_hash']==by_seq[origin]['payload_hash']
        assert row['origin_stamp']==by_seq[origin]['stamp']
        assert abs(message.header.stamp.sec+message.header.stamp.nanosec/1e9-row['stamp'])<1e-8
        if row['branch']=='localization':assert row['stamp']==row['origin_stamp']
        elif start<=row['t']<start+WINDOW:
            mode=r['condition'];assert mode!='missing'
            if mode in ('frozen','restamped'):assert origin==cached
            else:assert row['stamp']==row['origin_stamp']
            if mode=='frozen':assert row['stamp']==fault['cached_stamp']
            if mode=='restamped':
                assert row['stamp']!=row['origin_stamp']
                assert any(s['stamp']==row['stamp'] and abs(s['t']-row['t'])<.05 for s in scans)
        else:
            assert row['stamp']==row['origin_stamp']
            assert abs(by_seq[origin]['t']-row['t'])<.05
    selected=[s for s in scans if start<=s['t']<start+WINDOW]
    assert len(selected)>=8 and max(b['t']-a['t'] for a,b in zip(selected,selected[1:]))<.35
    inside=[s for s in pub if s['branch']=='obstacles' and start<=s['t']<start+WINDOW]
    assert len(inside)==(0 if r['condition']=='missing' else len(selected))
    counts=Counter(int(s['seq']) for s in rx)
    for s in pub:
        if s['t']<end-.25:assert counts[int(s['seq'])]==1,'Missing independent receipt'
    for branch in TOPICS:
        restored=[s for s in pub if s['branch']==branch and s['t']>=start+WINDOW]
        assert restored and restored[0]['t']<start+WINDOW+.35
    summaries={}
    for name,data,gap in [('world',truth,.15),('odom',odom,.15),('obstacle',obstacle,.15),('scans',scans,.35)]:
        group=[s for s in data if start-.3<=s['stamp']<=end]
        assert group and group[0]['stamp']<=start and end-group[-1]['stamp']<gap
        gaps=[b['stamp']-a['stamp'] for a,b in zip(group,group[1:])]
        assert min(gaps)>=0 and max(gaps)<gap,(name,max(gaps))
        assert all(-.02<=s['t']-s['stamp']<gap for s in group)
        summaries[name]=dict(samples=len(group),max_gap_s=max(gaps))
    parked=[s for s in obstacle if s['stamp']<r['obstacle_request_time']]
    assert parked and all(math.hypot(s['x']+8,s['y']+8)<.005 for s in parked)
    placed=[s for s in obstacle if s['stamp']>=appearance]
    assert placed and all(math.hypot(s['x']-OBSTACLE[0],s['y']-OBSTACLE[1])<.005 for s in placed)
    pre=min(truth,key=lambda s:abs(s['stamp']-appearance))
    assert clearance(pre)>.10,'Initial placement overlap/too close'
    hits=[s for s in scans if appearance<=s['stamp']<start+WINDOW and s['obstacle_hits']>0]
    assert hits,'Upstream laser did not see the introduced obstacle during fault'
    assert summarize(truth,odom,start,appearance,r['end_sim_s'],end)==r['metrics']
    assert endpoint_metrics(truth,rows(p/'localized.csv'),r['start_sim_s'],r['end_sim_s'],r['outcome'])==r['endpoint']
    contacts=list(map(json.loads,(p/'contacts.jsonl').read_text().splitlines()))
    assert len(contacts)==r['contact_messages'] and sum(bool(c['robot_obstacle_pairs']) for c in contacts)==r['robot_contact_messages']
    return dict(passed=True,stream_coverage=summaries,upstream_obstacle_hit_scans=len(hits),
        obstacle_window_messages=len(inside),cached_origin=cached,
        contact_messages=len(contacts),contact_coverage='Positive calibration separately required; observed events only, absence alone is not proof of zero contact.',
        scope='Conservative oriented-envelope clearance at sampled world poses; zero envelope clearance is not itself a physical contact.')
