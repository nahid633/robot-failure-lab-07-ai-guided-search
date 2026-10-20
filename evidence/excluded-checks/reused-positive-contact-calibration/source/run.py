#!/usr/bin/env python3
"""Bounded trial schedule, fresh ROS domain each startup, frozen evidence."""
import argparse,hashlib,json,os,shutil,subprocess,sys,time
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from core import *
from models import POSE_PLUGIN,instrument_world
from support import stop
from validate import validate_trial

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path)
    p.add_argument('--verification',action='store_true');p.add_argument('--calibration',action='store_true');p.add_argument('--condition',choices=CONDITIONS,action='append')
    p.add_argument('--verification-root',type=Path);p.add_argument('--calibration-root',type=Path);p.add_argument('--resume-infrastructure',action='store_true');p.add_argument('--domain',type=int,default=150)
    a=p.parse_args();root=a.output.resolve();snapshot=root/'source';parent=Path(__file__).resolve().parent
    assert not (a.verification and a.calibration)
    assert a.verification or a.calibration or a.condition is None
    if a.resume_infrastructure:
        protocol=json.loads((root/'PROTOCOL.json').read_text());assert protocol['domain_base']==a.domain
        for n,d in protocol['source_sha256'].items():assert sha(snapshot/n)==d
        for n,d in protocol['input_sha256'].items():assert sha(root/n)==d
        records=json.loads((root/'results.json').read_text())
        (root/f'continuation-{time.time_ns()}.json').write_text(json.dumps(dict(command=sys.argv,preserved_trials=[r['trial'] for r in records],reason='Continue pre-goal startup only; never retry completed outcomes.'),indent=2))
    else:
        root.mkdir(parents=True,exist_ok=False);snapshot.mkdir()
        for f in [*parent.glob('*.py'),parent/'README.md']:shutil.copy2(f,snapshot/f.name)
        bringup=Path(get_package_share_directory('nav2_bringup'));sim=Path(get_package_share_directory('nav2_minimal_tb3_sim'))
        launch=(bringup/'launch/tb3_simulation_launch.py').read_text();needle="cmd=['gz', 'sim', '-r', '-s', world_sdf]";assert launch.count(needle)==1
        (snapshot/'upstream_launch.txt').write_text(launch)
        seeded=launch.replace(needle,"cmd=['gz', 'sim', '--seed', os.environ['RFL_GZ_SEED'], '-r', '-s', world_sdf]")
        assert seeded.count('ld.add_action(gazebo_server)')==1
        seeded=seeded.replace('from launch.event_handlers import OnShutdown','from launch.event_handlers import OnProcessExit, OnShutdown')
        seeded=seeded.replace('ld.add_action(gazebo_server)','ld.add_action(RegisterEventHandler(OnProcessExit(target_action=world_sdf_xacro, on_exit=[gazebo_server])))')
        (snapshot/'seeded_launch.py').write_text(seeded)
        robot=(sim/'urdf/gz_waffle.sdf.xacro').read_text();assert robot.count('</model>')==1
        (snapshot/'upstream_robot.sdf.xacro').write_text(robot);(snapshot/'measured_robot.sdf.xacro').write_text(robot.replace('</model>',POSE_PLUGIN+'\n</model>'))
        world=(sim/'worlds/tb3_sandbox.sdf.xacro').read_text();(snapshot/'upstream_world.sdf.xacro').write_text(world);(snapshot/'world.sdf.xacro').write_text(instrument_world(world))
        config=yaml.safe_load((bringup/'params/nav2_params.yaml').read_text());assert config['controller_server']['ros__parameters']['general_goal_checker']['xy_goal_tolerance']==.25
        (root/'upstream-params.yaml').write_text(yaml.safe_dump(config,sort_keys=False));(root/'params.yaml').write_text(yaml.safe_dump(routed_config(config),sort_keys=False))
        for n in ('tb3_sandbox.pgm','tb3_sandbox.yaml'):shutil.copy2(bringup/'maps'/n,root/n)
        packages=subprocess.run(['dpkg-query','-W','ros-jazzy-nav2-*','ros-jazzy-ros-gz-*','ros-jazzy-gz-*','gz-*'],capture_output=True,text=True)
        (root/'packages.txt').write_text(packages.stdout+packages.stderr)
        if not a.verification and not a.calibration:
            assert a.verification_root is not None and a.calibration_root is not None
            vr=a.verification_root.resolve();vp=json.loads((vr/'PROTOCOL.json').read_text());checks=json.loads((vr/'results.json').read_text())
            assert len(checks)==4 and {r['condition'] for r in checks}==set(CONDITIONS)
            for r in checks:validate_trial(vr,r)
            fresh=next(r for r in checks if r['condition']=='fresh')
            assert fresh['metrics']['min_sampled_envelope_clearance_m']>.01 and fresh['metrics']['stop_after_appearance_s'] is not None,'Fresh control did not demonstrate obstacle response'
            for f in snapshot.iterdir():assert sha(f)==vp['source_sha256'][f.name],f.name
            for n in ('params.yaml','tb3_sandbox.pgm','tb3_sandbox.yaml','packages.txt'):assert sha(root/n)==sha(vr/n),n
            cr=a.calibration_root.resolve();assert json.loads((cr/'block-0-fresh/CALIBRATION.json').read_text())['passed']
            for n in ('measured_robot.sdf.xacro','world.sdf.xacro'):assert sha(snapshot/n)==sha(cr/'source'/n)
        order=a.condition or list(CONDITIONS)
        schedule=[dict(block=0,condition=c,seed=100) for c in (['fresh'] if a.calibration else order)] if a.verification or a.calibration else [dict(block=b+1,condition=c,seed=101+b) for b in range(3) for c in CONDITIONS[b:]+CONDITIONS[:b]]
        assert 1<=a.domain and a.domain+len(schedule)*3<=220
        protocol=dict(experiment='Robot Failure Lab 05',verification=a.verification,calibration=a.calibration,
            schedule=schedule,domain_base=a.domain,domain_policy='base + schedule_index*3 + attempt-1; distinct domain per startup',
            trigger=dict(min_goal_elapsed_s=3.,world_x_crossing=TRIGGER_X,world_y_band=[.25,.85],translation_min_mps=.05,sustained_s=.3),
            obstacle=dict(center=OBSTACLE,size=OBSTACLE_SIZE,parked=[-8,-8,.35],request_delay_s=APPEAR_DELAY,appearance_from='Observed world pose, not request receipt'),
            fault_duration_s=WINDOW,post_fault_min_observation_s=4.,task_limit_sim_s=TASK_LIMIT,task_limit_wall_s=TASK_WALL_LIMIT,startup_limit_wall_s=75,
            contact_calibration='Separate excluded process deliberately places obstacle into robot; never used for main task outcomes.',
            controls='Fresh AMCL branch in every condition; both costmaps and collision monitor share experimental branch. Same parameters, original goal, no external proximity guard or replacement goal.',
            primary='Source age, actual payload origin, source invalidation, motion stop, sampled oriented conservative-envelope clearance, obstacle-specific contacts separately.',
            footprint=dict(bounds=ENVELOPE,meaning='Conservative model-aligned planar envelope around inspected SDF collision shapes; zero envelope clearance is not contact proof.'),
            secondary='Original goal outcome; fixed 0.30m world-position requirement in result+1.0..1.5s.',
            interpretation='Three repeats each on one route; random AMCL/MPPI and scheduling uncontrolled; frozen/restamped branch is a synthetic diagnostic fault. No general physical safety, Nav2 bug or Vektor claim.',
            command=sys.argv,source_sha256={f.name:sha(f) for f in snapshot.iterdir()},
            input_sha256={f.name:sha(f) for f in root.iterdir() if f.is_file()},
            verification_root=str(a.verification_root) if a.verification_root else None,calibration_root=str(a.calibration_root) if a.calibration_root else None)
        (root/'PROTOCOL.json').write_text(json.dumps(protocol,indent=2));records=[];(root/'results.json').write_text('[]')
    for i,entry in enumerate(protocol['schedule']):
        b,c,seed=entry['block'],entry['condition'],entry['seed'];prior=[r for r in records if r['block']==b and r['condition']==c]
        if prior:assert len(prior)==1 and prior[0]['evidence_validated'];validate_trial(root,prior[0]);continue
        base=f'block-{b}-{c}';trial=root/base;attempt=1
        while trial.exists():
            assert a.resume_infrastructure and (trial/'INFRASTRUCTURE_ERROR.txt').exists()
            assert not any((trial/n).exists() for n in ('READY.json','GOAL_SUBMITTED.json','TASK_TERMINAL.json','result.json','CALIBRATION.json')),'Refusing outcome overwrite/retry'
            attempt+=1;assert attempt<=3;trial=root/f'{base}-attempt-{attempt}'
        trial.mkdir();domain=a.domain+i*3+attempt-1
        env=dict(os.environ,ROS_DOMAIN_ID=str(domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',GZ_PARTITION=f'rfl05-{os.getpid()}-{trial.name}',LIBGL_ALWAYS_SOFTWARE='1',RFL_GZ_SEED=str(seed));env.pop('ROS_LOCALHOST_ONLY',None)
        command=['ros2','launch',str(snapshot/'seeded_launch.py'),'headless:=True','use_rviz:=False','use_composition:=False',f'params_file:={root/"params.yaml"}',f'map:={root/"tb3_sandbox.yaml"}',f'robot_sdf:={snapshot/"measured_robot.sdf.xacro"}',f'world:={snapshot/"world.sdf.xacro"}']
        bridge=['ros2','run','ros_gz_bridge','parameter_bridge',
            '/model/turtlebot3_waffle/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V','/model/rfl05_obstacle/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/rfl05/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts','/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '--ros-args','-r','__node:=failure_lab_05_instrumentation_bridge','-r','/model/turtlebot3_waffle/pose:=/rfl05/truth','-r','/model/rfl05_obstacle/pose:=/rfl05/obstacle_pose']
        (trial/'launch-command.json').write_text(json.dumps(dict(command=command,bridge=bridge,domain=domain,seed=seed,host_load=os.getloadavg()),indent=2))
        print(f'Starting {trial.name} domain {domain}',flush=True);processes=[]
        try:
            with (trial/'simulation.log').open('w') as sl,(trial/'bridge.log').open('w') as bl,(trial/'trial.log').open('w') as tl:
                processes.append(subprocess.Popen(command,env=env,stdout=sl,stderr=subprocess.STDOUT,start_new_session=True));processes.append(subprocess.Popen(bridge,env=env,stdout=bl,stderr=subprocess.STDOUT,start_new_session=True));time.sleep(8)
                worker_command=[sys.executable,str(snapshot/'trial.py'),'--condition',c,'--output',str(trial)]+(['--calibration'] if a.calibration else [])
                worker=subprocess.Popen(worker_command,env=env,stdout=tl,stderr=subprocess.STDOUT,start_new_session=True);processes.append(worker);started=time.monotonic()
                while worker.poll() is None:
                    dt=time.monotonic()-started
                    if dt>75 and not (trial/'READY.json').exists():raise TimeoutError('Readiness exceeded 75 wall seconds')
                    if dt>330:raise TimeoutError('Whole worker exceeded deadline')
                    time.sleep(.2)
                assert worker.returncode==0,'Worker failed; inspect retained logs'
        except BaseException as error:(trial/'INFRASTRUCTURE_ERROR.txt').write_text(repr(error));raise
        finally:
            for process in reversed(processes):stop(process)
            (trial/'process-exits.json').write_text(json.dumps([dict(pid=p.pid,returncode=p.returncode) for p in processes],indent=2))
        if a.calibration:
            assert json.loads((trial/'CALIBRATION.json').read_text())['passed'];print('COMPLETE: excluded contact calibration',flush=True);continue
        r=json.loads((trial/'result.json').read_text());r.update(trial=trial.name,block=b,seed=seed,evidence_validated=False);records.append(r);(root/'results.json').write_text(json.dumps(records,indent=2))
        try:
            v=validate_trial(root,r);(trial/'VALIDATION.json').write_text(json.dumps(v,indent=2));r['evidence_validated']=True;(root/'results.json').write_text(json.dumps(records,indent=2))
        except BaseException as error:(trial/'PROTOCOL_ERROR.txt').write_text(repr(error));raise
        print('{}: {}; min envelope clearance {:.3f}m; contact messages {}; stop {}'.format(trial.name,r['outcome'],r['metrics']['min_sampled_envelope_clearance_m'],r['robot_contact_messages'],r['metrics']['stop_after_appearance_s']),flush=True)
    print('COMPLETE: {} task outcomes'.format(len(records)),flush=True)


if __name__=='__main__':main()
