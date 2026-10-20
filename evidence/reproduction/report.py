"""Independently audit live decisions, labels, budget charges and render evidence."""
import argparse,hashlib,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon
from matplotlib.animation import FuncAnimation,PillowWriter
from search import *
from validate import validate_trial,rows
import core

COLORS={'model':'#9b5fa5','random':'#2675a6','fixed':'#bd761b'}


def main():
    p=argparse.ArgumentParser();p.add_argument('root',type=Path);a=p.parse_args();root=a.root.resolve()
    protocol=json.loads((root/'PROTOCOL.json').read_text());assert protocol['phase']=='main' and (root/'COMPLETE.json').exists()
    records=json.loads((root/'results.json').read_text());assert len(records)==json.loads((root/'COMPLETE.json').read_text())['outcomes']
    for name,digest in protocol['source_sha256'].items():assert hashlib.sha256((root/'source'/name).read_bytes()).hexdigest()==digest,name
    for name,digest in protocol['input_sha256'].items():assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
    audits={r['trial']:validate_trial(root,r) for r in records}
    settings=[json.loads((root/r['trial']/'effective-before.json').read_text()) for r in records];assert all(v==settings[0] for v in settings)
    warm=[r for r in records if r['role']=='training'];usable=[r for r in warm if r['label'] is not None]
    assert [r['scenario'] for r in warm]==training() and len(warm)==9 and len(usable)>=6 and {r['label'] for r in usable}=={0,1}
    warm_wall=sum(r['simulator_wall_s'] for r in warm);ledger=[]
    for ep in (1,2,3):
        for method in METHODS:
            history=[r for r in records if r.get('episode')==ep and r.get('method')==method]
            assert history and len(history)<=(6 if method=='model' else 15)
            previous=[]
            for r in history:
                assert not confirmed(previous)
                s,confirmation,decision=choose(method,ep,warm,previous)
                assert r['scenario']==s and r['confirmation']==confirmation and r['seed']==(300 if confirmation else 200)+ep
                if method=='model' and not confirmation:
                    assert r['decision']['training_trial_ids']==decision['training_trial_ids']
                    assert len(r['decision']['ranking'])==len(decision['ranking'])
                    for old,new in zip(r['decision']['ranking'],decision['ranking']):
                        assert old['scenario']==new['scenario']
                        assert all(abs(old[k]-new[k])<1e-10 for k in ('mean','std','score'))
                previous.append(r)
            success=confirmed(history);assert success or len(history)==(6 if method=='model' else 15)
            charge=9 if method=='model' else 0
            ledger.append(dict(episode=ep,method=method,confirmed=success,heldout_calls=len(history),training_charge=charge,
                charged_calls=charge+len(history),charged_wall_s=(warm_wall if charge else 0)+sum(r['simulator_wall_s']+r['decision']['selection_wall_s'] for r in history),
                confirmed_scenario=history[-1]['scenario'] if success else None,terminal_trials=[r['trial'] for r in history[-2:]] if success else []))
    observed_search=[r for r in records if r['role']=='search'];assert len(observed_search)==sum(v['heldout_calls'] for v in ledger)
    audit=dict(passed=True,trials=audits,runtime_parameters_identical=True,source_input_hashes_match=True,
        model_visibility_and_decision_replay_passed=True,policy_budgets_passed=True,physical_main_calls=len(records),ledger=ledger)
    (root/'EVIDENCE-VALIDATION.json').write_text(json.dumps(audit,indent=2));(root/'SEARCH-LEDGER.json').write_text(json.dumps(ledger,indent=2))
    table=['| Episode | Policy | Confirmed contact scenario | Held-out calls | Charged training calls | Total standalone calls | Charged wall seconds |',
           '| --- | --- | --- | --- | --- | --- | --- |']
    for v in ledger:
        table.append('| {} | {} | {} | {} | {} | {}{} | {:.1f} |'.format(v['episode'],v['method'],'yes' if v['confirmed'] else 'not within budget',v['heldout_calls'],v['training_charge'],v['charged_calls'],' (censored)' if not v['confirmed'] else '',v['charged_wall_s']))
    allrows=['| Trial | Role / policy | Episode | Duration (s) | Offset (m) | Obstacle x (m) | Contact label | Confirmation attempt |', '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in records:
        s=r['scenario'];allrows.append('| {} | {} | {} | {:.1f} | {:.3f} | {:.2f} | {} | {} |'.format(r['trial'],r.get('method',r['role']),r.get('episode','—'),s['duration'],s['offset'],s['x'],r['label'],r['confirmation']))
    main_wall=sum(r['simulator_wall_s'] for r in records)+sum(r.get('decision',{}).get('selection_wall_s',0) for r in records)
    amortized={m:sum(r['simulator_wall_s']+r['decision']['selection_wall_s'] for r in observed_search if r['method']==m)+(warm_wall if m=='model' else 0) for m in METHODS}
    text='\n'.join(['# Robot Failure Lab 07: AI-guided failure search','',
        'Live Gazebo/Nav2 simulator queries. Model, random and fixed sweep, three search episodes in one held-out obstacle-placement context.','',*table,'',
        '## Cost and interpretation','', 'Charged wall values sum recorded simulator attempts and saved selection timings. They exclude manual inspection pauses, coordinator replay checks and report generation; simulator-call budget is the primary comparison. BATCH-ACCOUNTING.json separately records observed batch elapsed time.','',
        '{} unique main simulator evaluations: nine warm runs plus {} live search/confirmation calls; {:.1f} wall seconds including starts/shutdown and charged failed-start overhead. The nine warm runs were generated once, shared physically, and charged in full to the model in each standalone comparison. Baselines do not pay a model-training charge. Three episodes share that initial dataset and are not independent training experiments.'.format(len(records),len(observed_search),main_wall),'',
        'Amortized observed wall totals across all three episodes (one physical training charge): '+', '.join('{} {:.1f}s'.format(m,amortized[m]) for m in METHODS)+'. These totals describe this chosen workload, not general compute savings.','',
        'A smaller held-out query count alone is not evidence of lower end-to-end testing cost. Compare the charged total, wall time, censoring and confirmation requirements. The fixed baseline deliberately tries the longest centered fault first, rather than an artificially weak sweep.','',
        '## What the model does','',f'Initial corpus: {len(warm)} charged evaluations; {len(usable)} usable labels; {len(warm)-len(usable)} missing labels excluded from fitting. None in the outcome table means the scenario was not reached, never a zero-event label.','',
        'A fixed-hyperparameter Gaussian process regression surrogate of binary contact-event observations uses mean+0.5 posterior standard deviation to select tests. Scores are not calibrated failure probabilities. Training used obstacle x=-0.60m; every live search used previously withheld x=-0.30m. Later updates see only that policy/episode\'s observed labels plus the initial nine rows. Saved rankings/input IDs were replayed and checked without other policies\' labels. Verification outcomes never train the model.','',
        'The task, obstacle dimensions and navigation settings are fixed. Search varies cached/restamped scan duration and obstacle lateral offset. A failure is an obstacle-specific contact event between independently measured obstacle appearance and onset+6s; a confirmed failure repeats the same case in a fresh process with a new Gazebo seed. Zero labels mean no received event, not certified physical clearance. Positive contact calibration is retained with matching model/world hashes.','',
        'Still-active navigation actions are intentionally canceled after the fixed observation horizon. That cancellation is not a mission timeout or goal failure. Contacts during the passive tail are excluded from model labels. Conservative-envelope clearance is independent geometry; zero overlap is not itself contact proof. AMCL receives fresh scans, so this is a synthetic downstream-branch fault, not a complete physical scanner failure.','',
        'This is one simulated route, one held-out context, three episodes, few labels and uncontrolled AMCL/MPPI/scheduling. Repeated seeds do not guarantee identical trajectories. No general search-efficiency, diversity-of-mechanisms, physical-safety, world-model, algorithmic-novelty, Vektor integration or customer-use claim follows.','',
        '## Every main evaluation','',*allrows,'',
        'Full packet/pose/contact evidence, exact policies and acquisition rankings, startup logs, parameter audits, frozen source/model/map/configuration/package hashes and excluded verification are retained. Fresh-machine reproduction is unverified. Nothing has been posted or sent.',''])
    (root/'REPORT.md').write_text(text)
    fig,axes=plt.subplots(1,2,figsize=(12,4.8),layout='constrained')
    for ax,key,title in [(axes[0],'heldout_calls','Live calls in held-out context'),(axes[1],'charged_calls','Standalone calls including model training')]:
        for j,m in enumerate(METHODS):
            vs=[v for v in ledger if v['method']==m];x=np.arange(3)+(j-1)*.24
            bars=ax.bar(x,[v[key] for v in vs],width=.22,color=COLORS[m],label=m)
            for bar,v in zip(bars,vs):
                if not v['confirmed']:bar.set_hatch('///')
                ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+.2,str(v[key])+('+' if not v['confirmed'] else ''),ha='center',fontsize=9)
        ax.set_xticks(range(3),['1','2','3']);ax.set_xlabel('Search episode');ax.set_ylabel('Simulator evaluations');ax.set_title(title);ax.set_ylim(0,17);ax.legend(frameon=False);ax.grid(axis='y',alpha=.2)
    fig.suptitle('AI-guided search versus random and a strong fixed sweep\nStop at first two-run confirmed contact; + / hatching means budget exhausted',fontsize=12)
    fig.savefig(root/'search-cost.png',dpi=150);plt.close(fig)
    pool=candidates();mu,std=posterior([(r['scenario'],r['label']) for r in usable],pool)
    fig,axes=plt.subplots(1,2,figsize=(11,4.5),layout='constrained')
    for ax,z,title in [(axes[0],mu,'Initial GP mean (not a probability)'),(axes[1],mu+.5*std,'Initial acquisition score: mean + 0.5 std')]:
        sc=ax.scatter([s['duration'] for s in pool],[s['offset'] for s in pool],c=z,s=350,cmap='viridis');fig.colorbar(sc,ax=ax)
        ax.set_xlabel('Restamped cached-scan duration (s)');ax.set_ylabel('Obstacle lateral offset (m)');ax.set_title(title,fontsize=11);ax.grid(alpha=.15)
    fig.suptitle(f'Before any held-out outcome · {len(warm)} initial calls, {len(usable)} usable training labels',fontsize=12)
    fig.savefig(root/'initial-model.png',dpi=150);plt.close(fig)
    chosen={m:next((r for r in records if r.get('episode')==1 and r.get('method')==m and r['oracle_valid']),None) for m in METHODS}
    fig,axes=plt.subplots(1,3,figsize=(12,4.7),layout='constrained');artists={};trajectory={}
    for ax,m in zip(axes,METHODS):
        r=chosen[m]
        if r is None:ax.text(.5,.5,'No scenario reached in episode 1',ha='center',transform=ax.transAxes);continue
        s=r['scenario'];trajectory[m]=rows(root/r['trial']/'world.csv')
        trail,=ax.plot([],[],color=COLORS[m]);dot,=ax.plot([],[],'o',color=COLORS[m],ms=6)
        box=Rectangle((s['x']-.15,.55+s['offset']-.225),.3,.45,facecolor='#333',alpha=0);ax.add_patch(box)
        envelope=Polygon(np.zeros((4,2)),fill=False,edgecolor=COLORS[m]);ax.add_patch(envelope)
        caption=ax.text(.02,.02,'',transform=ax.transAxes,fontsize=8,bbox=dict(facecolor='white',alpha=.9,edgecolor='none'))
        artists[m]=(trail,dot,box,envelope,caption);ax.set_xlim(-1.7,.4);ax.set_ylim(-.15,1.35);ax.set_aspect('equal');ax.set_title(m+' · first reached case');ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)');ax.grid(alpha=.15)
    def animate(t):
        for m,r in chosen.items():
            if r is None:continue
            s=r['scenario'];trail,dot,box,envelope,caption=artists[m];data=[v for v in trajectory[m] if r['window_start']-.6<=v['stamp']<=r['window_start']+t]
            if not data:continue
            v=data[-1];trail.set_data([z['x'] for z in data],[z['y'] for z in data]);dot.set_data([v['x']],[v['y']]);envelope.set_xy(core.rectangle(v['x'],v['y'],v['yaw'],core.ENVELOPE))
            visible=t>=r['appearance']-r['window_start'];box.set_alpha(.75 if visible else 0)
            fault=0<=t<s['duration'];contact=r['first_contact_after_appearance_s']
            observed=contact is not None and t>=r['appearance']-r['window_start']+contact
            caption.set_text('Onset {:+.1f}s · {}\nDuration {:.1f}s · offset {:+.3f}m\nContact by this time: {}'.format(t,'cached + new stamp' if fault else 'original scans',s['duration'],s['offset'],'observed' if observed else 'none recorded'))
    fig.suptitle('Recorded simulated world poses · episode 1 / first reached scenario per policy\nOutline: conservative envelope · dark box: obstacle · not camera footage',fontsize=11)
    anim=FuncAnimation(fig,animate,frames=np.linspace(-.6,6.,90),interval=100)
    anim.save(root/'first-search-recorded.gif',writer=PillowWriter(fps=10),dpi=90);plt.close(fig)
    print('Validated {} main calls, all nine policy histories and cost ledgers; rendered report/figures/GIF.'.format(len(records)))


if __name__=='__main__':main()
