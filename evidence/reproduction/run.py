"""Frozen sequential simulation oracle and policy-isolated search coordinator."""
import argparse,hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from models import POSE_PLUGIN,instrument_world
from core import routed_config
from support import stop
from validate import validate_trial
from search import *


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,indent=2))


def prepare(root,phase,verification_root,calibration_root):
    root.mkdir(parents=True,exist_ok=False);snapshot=root/'source';snapshot.mkdir();(root/'requests').mkdir()
    parent=Path(__file__).resolve().parent
    for f in [*parent.glob('*.py'),parent/'README.md']:shutil.copy2(f,snapshot/f.name)
    bringup=Path(get_package_share_directory('nav2_bringup'));sim=Path(get_package_share_directory('nav2_minimal_tb3_sim'))
    launch=(bringup/'launch/tb3_simulation_launch.py').read_text();needle="cmd=['gz', 'sim', '-r', '-s', world_sdf]";assert launch.count(needle)==1
    (snapshot/'upstream_launch.txt').write_text(launch)
    seeded=launch.replace(needle,"cmd=['gz', 'sim', '--seed', os.environ['RFL_GZ_SEED'], '-r', '-s', world_sdf]")
    seeded=seeded.replace('from launch.event_handlers import OnShutdown','from launch.event_handlers import OnProcessExit, OnShutdown')
    assert seeded.count('ld.add_action(gazebo_server)')==1
    seeded=seeded.replace('ld.add_action(gazebo_server)','ld.add_action(RegisterEventHandler(OnProcessExit(target_action=world_sdf_xacro, on_exit=[gazebo_server])))')
    (snapshot/'seeded_launch.py').write_text(seeded)
    robot=(sim/'urdf/gz_waffle.sdf.xacro').read_text();assert robot.count('</model>')==1
    (snapshot/'upstream_robot.sdf.xacro').write_text(robot);(snapshot/'measured_robot.sdf.xacro').write_text(robot.replace('</model>',POSE_PLUGIN+'\n</model>'))
    world=(sim/'worlds/tb3_sandbox.sdf.xacro').read_text();(snapshot/'upstream_world.sdf.xacro').write_text(world);(snapshot/'world.sdf.xacro').write_text(instrument_world(world))
    config=yaml.safe_load((bringup/'params/nav2_params.yaml').read_text());(root/'upstream-params.yaml').write_text(yaml.safe_dump(config,sort_keys=False));(root/'params.yaml').write_text(yaml.safe_dump(routed_config(config),sort_keys=False))
    for name in ('tb3_sandbox.pgm','tb3_sandbox.yaml'):shutil.copy2(bringup/'maps'/name,root/name)
    packages=subprocess.run(['dpkg-query','-W','ros-jazzy-nav2-*','ros-jazzy-ros-gz-*','ros-jazzy-gz-*','gz-*'],capture_output=True,text=True)
    (root/'packages.txt').write_text(packages.stdout+packages.stderr)
    assert json.loads((calibration_root/'block-0-fresh/CALIBRATION.json').read_text())['passed']
    for name in ('measured_robot.sdf.xacro','world.sdf.xacro'):assert sha(snapshot/name)==sha(calibration_root/'source'/name),'Contact instrumentation changed'
    if phase=='main':
        assert verification_root is not None
        vp=json.loads((verification_root/'PROTOCOL.json').read_text());checks=json.loads((verification_root/'results.json').read_text())
        assert len(checks)==6 and (verification_root/'COMPLETE.json').exists()
        assert [r['label'] for r in checks]==[0,1,0,1,0,1],'Feasibility labels not sufficiently reproducible'
        for r in checks:validate_trial(verification_root,r)
        for name,digest in vp['source_sha256'].items():assert sha(snapshot/name)==digest,name
        for name in ('params.yaml','tb3_sandbox.pgm','tb3_sandbox.yaml','packages.txt'):assert sha(root/name)==sha(verification_root/name),name
    protocol=dict(experiment='Robot Failure Lab 07: AI-guided failure search',phase=phase,
        train=training(),candidate_grid=candidates(),fixed_order=fixed_order(),verification=verification(),
        training_seed_rule='120+training_index',search_seeds='discovery 200+episode; confirmation 300+episode',
        episodes=[1,2,3],policy_order=[list(METHODS[i:]+METHODS[:i]) for i in range(3)],
        episode_total_simulator_budget=15,model_training_charge=9,model_heldout_query_budget=6,baseline_heldout_query_budget=15,
        training_shared_physically=True,training_accounting='All 9 warm calls charged to model in every standalone episode; also report unique-physical/amortized totals. Baselines pay no training charge.',
        model=dict(kind='Gaussian process regression surrogate of binary observed-contact label; not calibrated probability',prior=PRIOR,noise_std=NOISE,length_scales=LENGTH.tolist(),acquisition='mean + 0.5 std',tuning='none'),
        label='At least one obstacle-specific contact event between independently observed appearance and fault onset+6 simulated seconds. Zero means no received event, not certified no contact.',
        confirmation='Positive discovery followed immediately by same scenario with new seed; both positive. Confirmation consumes query budget. Stop policy after first confirmed failure, otherwise right-censored at budget.',
        holdout='All warm cases obstacle x=-0.60; all search cases x=-0.30. No heldout outcome is read before its request. Updates see only own policy/episode labels. Verification excluded from model inputs.',
        task='Fixed six-second post-trigger observation; intentionally cancel still-active action, then passive 1.8 s tail. Active at horizon is not goal failure or mission timeout.',
        domain_policy='Sequential low-domain leases: 10 + startup index modulo 90; reuse only after full prior process-group shutdown. Unique Gazebo partition each attempt.',
        limits=dict(startup_wall_s=75,worker_wall_s=260,pretrigger_sim_s=45,task_wall_s=150,tail_wall_s=30,max_startups_per_request=3),
        calibration_root=str(calibration_root),verification_root=str(verification_root) if verification_root else None,
        source_sha256={f.name:sha(f) for f in snapshot.iterdir()},input_sha256={f.name:sha(f) for f in root.iterdir() if f.is_file()},command=sys.argv)
    write(root/'PROTOCOL.json',protocol);write(root/'results.json',[])


def execute(root,records,request,resume):
    index=len(records)+1;reqpath=root/'requests'/f'{index:04d}.json'
    if reqpath.exists():
        stored=json.loads(reqpath.read_text())
        for key in ('role','scenario','seed','episode','method','confirmation'):assert stored.get(key)==request.get(key)
        request=stored
    else:write(reqpath,request)
    trial=root/f'trial-{index:04d}';attempt=1
    while trial.exists():
        if (trial/'result.json').exists():raise RuntimeError('Uncommitted completed outcome exists; inspect/reconcile without rerunning')
        assert resume and (trial/'INFRASTRUCTURE_ERROR.txt').exists()
        assert not any((trial/n).exists() for n in ('READY.json','GOAL_SUBMITTED.json','OBSERVATION_CUTOFF.json')),'Refuse outcome/protocol retry'
        attempt+=1;assert attempt<=3;trial=root/f'trial-{index:04d}-attempt-{attempt}'
    trial.mkdir();startup=len(list(root.glob('trial-*')))-1;domain=10+startup%90
    write(trial/'scenario.json',request['scenario'])
    env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',GZ_PARTITION=f'rfl07-{os.getpid()}-{trial.name}',LIBGL_ALWAYS_SOFTWARE='1',RFL_GZ_SEED=str(request['seed']));env.pop('ROS_LOCALHOST_ONLY',None)
    snap=root/'source'
    command=['ros2','launch',str(snap/'seeded_launch.py'),'headless:=True','use_rviz:=False','use_composition:=False',f'params_file:={root/"params.yaml"}',f'map:={root/"tb3_sandbox.yaml"}',f'robot_sdf:={snap/"measured_robot.sdf.xacro"}',f'world:={snap/"world.sdf.xacro"}']
    bridge=['ros2','run','ros_gz_bridge','parameter_bridge','/model/turtlebot3_waffle/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','/model/rfl05_obstacle/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','/rfl05/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose','--ros-args','-r','__node:=failure_lab_07_bridge','-r','/model/turtlebot3_waffle/pose:=/rfl05/truth','-r','/model/rfl05_obstacle/pose:=/rfl05/obstacle_pose']
    write(trial/'launch-command.json',dict(command=command,bridge=bridge,domain=domain,request=request,host_load=os.getloadavg()))
    print('Starting {} {} {} domain {} {}'.format(trial.name,request['role'],request.get('method',''),domain,request['scenario']),flush=True)
    processes=[];began=time.monotonic()
    try:
        with (trial/'simulation.log').open('w') as sl,(trial/'bridge.log').open('w') as bl,(trial/'trial.log').open('w') as tl:
            processes.append(subprocess.Popen(command,env=env,stdout=sl,stderr=subprocess.STDOUT,start_new_session=True));processes.append(subprocess.Popen(bridge,env=env,stdout=bl,stderr=subprocess.STDOUT,start_new_session=True));time.sleep(8)
            worker=subprocess.Popen([sys.executable,str(snap/'worker.py'),'--scenario',str(trial/'scenario.json'),'--output',str(trial)],env=env,stdout=tl,stderr=subprocess.STDOUT,start_new_session=True);processes.append(worker);started=time.monotonic()
            while worker.poll() is None:
                dt=time.monotonic()-started
                if dt>75 and not (trial/'READY.json').exists():raise TimeoutError('Readiness exceeded 75 wall seconds')
                if dt>260:raise TimeoutError('Worker exceeded wall bound')
                time.sleep(.2)
            assert worker.returncode==0,'Worker failed; inspect all retained logs'
    except BaseException as error:(trial/'INFRASTRUCTURE_ERROR.txt').write_text(repr(error));raise
    finally:
        for process in reversed(processes):stop(process)
        write(trial/'process-exits.json',[dict(pid=p.pid,returncode=p.returncode) for p in processes])
        write(trial/'ATTEMPT-TIMING.json',dict(elapsed_wall_s=time.monotonic()-began))
    attempt_folders=[z for z in root.glob(f'trial-{index:04d}*') if z.is_dir()]
    wall_charge=sum(json.loads((z/'ATTEMPT-TIMING.json').read_text())['elapsed_wall_s'] for z in attempt_folders)
    r=json.loads((trial/'result.json').read_text());r.update(request,trial=trial.name,simulator_wall_s=wall_charge,startup_attempts=len(attempt_folders),evidence_validated=False)
    records.append(r);write(root/'results.json',records)
    try:write(trial/'VALIDATION.json',validate_trial(root,r));r['evidence_validated']=True;write(root/'results.json',records)
    except BaseException as error:(trial/'PROTOCOL_ERROR.txt').write_text(repr(error));raise
    print('{}: contact label {}; clearance {}'.format(trial.name,r['label'],r['min_sampled_envelope_clearance_m']),flush=True)
    return r


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);p.add_argument('--phase',choices=('verify','main'),required=True);p.add_argument('--verification-root',type=Path);p.add_argument('--calibration-root',type=Path,required=True);p.add_argument('--resume-infrastructure',action='store_true');a=p.parse_args();root=a.output.resolve()
    if not a.resume_infrastructure:prepare(root,a.phase,a.verification_root,a.calibration_root)
    protocol=json.loads((root/'PROTOCOL.json').read_text());assert protocol['phase']==a.phase
    for n,d in protocol['source_sha256'].items():assert sha(root/'source'/n)==d,n
    for n,d in protocol['input_sha256'].items():assert sha(root/n)==d,n
    records=json.loads((root/'results.json').read_text())
    for r in records:assert r['evidence_validated'];validate_trial(root,r)
    if a.resume_infrastructure:write(root/f'continuation-{time.time_ns()}.json',dict(command=sys.argv,preserved=[r['trial'] for r in records]))
    if a.phase=='verify':
        for index,entry in enumerate(verification()):
            if index<len(records):continue
            execute(root,records,dict(role='verification',scenario=entry['scenario'],seed=entry['seed'],confirmation=False),a.resume_infrastructure)
    else:
        for index,s in enumerate(training()):
            if index<sum(r['role']=='training' for r in records):continue
            execute(root,records,dict(role='training',scenario=s,seed=120+index,confirmation=False),a.resume_infrastructure)
        warm=[r for r in records if r['role']=='training'];usable=[r for r in warm if r['label'] is not None]
        assert len(warm)==9 and len(usable)>=6 and {r['label'] for r in usable}=={0,1},'Training requires at least six usable nonconstant labels among nine charged calls'
        for episode in (1,2,3):
            order=METHODS[episode-1:]+METHODS[:episode-1]
            while True:
                ran=False
                for method in order:
                    history=[r for r in records if r.get('episode')==episode and r.get('method')==method]
                    budget=6 if method=='model' else 15
                    if confirmed(history) or len(history)>=budget:continue
                    selected,confirmation,decision=choose(method,episode,warm,history)
                    request=dict(role='search',episode=episode,method=method,scenario=selected,seed=(300 if confirmation else 200)+episode,confirmation=confirmation,decision=decision)
                    execute(root,records,request,a.resume_infrastructure);ran=True
                if not ran:break
    write(root/'COMPLETE.json',dict(outcomes=len(records),phase=a.phase));print('COMPLETE {} outcomes'.format(len(records)),flush=True)


if __name__=='__main__':main()
