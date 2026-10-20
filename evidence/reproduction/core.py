"""Frozen predeclared fault policies and sampled conservative-envelope metrics."""
import math

CONDITIONS=('fresh','missing','frozen','restamped')
LABELS={'fresh':'Fresh scans','missing':'Missing scans','frozen':'Frozen / original stamp','restamped':'Frozen / new stamp'}
TOPICS={'localization':'/rfl05/localization_scan','obstacles':'/rfl05/obstacle_scan'}
START=(-2.,-.5);GOAL=(1.,.5);POSITION_LIMIT=.30
TRIGGER_X=-1.30
OBSTACLE=(-.60,.55);OBSTACLE_SIZE=(.30,.45,.60)
WINDOW=2.;APPEAR_DELAY=.20
TASK_LIMIT=60.;TASK_WALL_LIMIT=180.
# Conservative planar envelope encloses the inspected SDF collision shapes.
# base box x=-.064 +/- .1325; camera x=.069 +/- .004;
# wheel cylinder y=+/- .144 +/- about .009. Rounded outward with margin.
ENVELOPE=(-.20,.08,-.16,.16)


def configure(scenario):
    """Called before observer imports; one fresh process per declared scenario."""
    global WINDOW,OBSTACLE
    WINDOW=float(scenario['duration']);OBSTACLE=(float(scenario['x']),.55+float(scenario['offset']))
    assert 0<=WINDOW<=1.6 and -.60<=OBSTACLE[0]<=-.30
    assert -.351<=float(scenario['offset'])<=.351


def policy(condition,t,start):
    if condition not in CONDITIONS:raise ValueError(condition)
    if start is None or not start<=t<start+WINDOW:return 'fresh'
    return condition


def routed_config(base):
    def walk(v):
        if isinstance(v,dict):return {k:(TOPICS['obstacles'] if k in ('topic','scan_topic') and x in ('scan','/scan') else walk(x)) for k,x in v.items()}
        if isinstance(v,list):return [walk(x) for x in v]
        return v
    result=walk(base);result['amcl']['ros__parameters']['scan_topic']=TOPICS['localization'];return result


def routing_valid(graph):
    names=lambda topic:set(graph[topic]['subscribers'])
    return (names('/scan')=={'failure_lab_05_observer'} and
            names(TOPICS['localization'])=={'amcl','failure_lab_05_observer'} and
            names(TOPICS['obstacles'])=={'local_costmap','global_costmap','collision_monitor','failure_lab_05_observer'} and
            names('/rfl05/truth')=={'failure_lab_05_observer'} and
            names('/rfl05/obstacle_pose')=={'failure_lab_05_observer'})


def rectangle(x,y,yaw,bounds):
    lo,hi,bottom,top=bounds;c=math.cos(yaw);s=math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in [(lo,bottom),(hi,bottom),(hi,top),(lo,top)]]


def point_segment(p,a,b):
    dx=b[0]-a[0];dy=b[1]-a[1];den=dx*dx+dy*dy
    f=0 if not den else max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/den))
    return math.hypot(p[0]-a[0]-f*dx,p[1]-a[1]-f*dy)


def polygon_clearance(a,b):
    separated=False
    for poly in (a,b):
        for p,q in zip(poly,poly[1:]+poly[:1]):
            axis=(-(q[1]-p[1]),q[0]-p[0]);pa=[x*axis[0]+y*axis[1] for x,y in a];pb=[x*axis[0]+y*axis[1] for x,y in b]
            if max(pa)<min(pb)-1e-12 or max(pb)<min(pa)-1e-12:separated=True
    if not separated:return 0.
    return min(point_segment(p,u,v) for points,edges in [(a,b),(b,a)] for p in points for u,v in zip(edges,edges[1:]+edges[:1]))


def clearance(row,obstacle=None):
    if obstacle is None:obstacle=OBSTACLE
    sx,sy,_=OBSTACLE_SIZE
    return polygon_clearance(rectangle(row['x'],row['y'],row['yaw'],ENVELOPE),rectangle(*obstacle,0,(-sx/2,sx/2,-sy/2,sy/2)))


def sustained_stop(odom,start,end):
    since=prior=None
    for row in odom:
        t=row['stamp']
        if not start<=t<=end:continue
        if row['speed']<=.01 and abs(row['w'])<=.03:
            if since is None or prior is None or t-prior>.15:since=t
            if t-since>=.30-1e-8:return since
        else:since=None
        prior=t
    return None


def summarize(truth,odom,start,appearance,end,observation_end):
    relevant=[r for r in truth if appearance<=r['stamp']<=observation_end]
    if not relevant:raise ValueError('No post-appearance world pose')
    stop=sustained_stop(odom,appearance,observation_end)
    motion=[r for r in relevant if stop is not None and r['stamp']<=stop]
    return dict(min_sampled_envelope_clearance_m=min(clearance(r) for r in relevant),
        envelope_overlap_samples=sum(clearance(r)<=1e-8 for r in relevant),
        stop_after_appearance_s=None if stop is None else stop-appearance,
        path_to_first_stop_m=None if stop is None else sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(motion,motion[1:])),
        world_samples_after_appearance=len(relevant),
        window_min_clearance_m=min((clearance(r) for r in relevant if r['stamp']<=start+WINDOW),default=None))
