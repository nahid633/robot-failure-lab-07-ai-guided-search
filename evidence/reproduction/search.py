"""Frozen model, candidate space and independent policy histories."""
import hashlib,json,time
import numpy as np

METHODS=('model','random','fixed')
DURATIONS=(0.,.4,.8,1.2,1.6)
OFFSETS=(-.35,-.175,0.,.175,.35)
TRAIN_X=-.60;TEST_X=-.30
TOTAL_BUDGET=15;TRAIN_COUNT=9;MODEL_QUERY_BUDGET=6
PRIOR=.5;NOISE=.2;LENGTH=np.array([.6,.65,1.])
HORIZON=6.


def scenario(duration,offset,x):
    return dict(duration=float(duration),offset=float(offset),x=float(x))


def case_id(s):return hashlib.sha256(json.dumps(s,sort_keys=True).encode()).hexdigest()[:12]


def training():return [scenario(d,o,TRAIN_X) for d in (0.,.8,1.6) for o in (-.35,0.,.35)]


def candidates():return [scenario(d,o,TEST_X) for d in DURATIONS for o in OFFSETS]


def fixed_order():
    # Deliberately strong simple baseline: longest fault first, center before sides.
    return sorted(candidates(),key=lambda s:(-s['duration'],abs(s['offset']),s['offset']))


def features(s):return np.array([s['duration']/1.6,s['offset']/.35,(s['x']-TRAIN_X)/.3])


def posterior(observed,pool):
    """GP regression of 0/1 contact observations, not calibrated probabilities."""
    q=np.array([features(s) for s in pool])
    if not observed:return np.full(len(pool),PRIOR),np.ones(len(pool))
    x=np.array([features(s) for s,y in observed]);y=np.array([y for s,y in observed],float)
    def kernel(a,b):return np.exp(-.5*np.sum(((a[:,None,:]-b[None,:,:])/LENGTH)**2,axis=2))
    k=kernel(x,x)+np.eye(len(x))*(NOISE**2+1e-9)
    chol=np.linalg.cholesky(k);cross=kernel(x,q)
    alpha=np.linalg.solve(chol.T,np.linalg.solve(chol,y-PRIOR))
    mu=PRIOR+cross.T@alpha;v=np.linalg.solve(chol,cross)
    std=np.sqrt(np.maximum(1e-12,1.-np.sum(v*v,axis=0)))
    return mu,std


def choose(method,episode,warm,history):
    """Only warm data and this policy/episode's completed labels are visible."""
    began=time.perf_counter()
    if history and history[-1]['label'] and not history[-1]['confirmation']:
        return history[-1]['scenario'],True,dict(reason='Confirm last positive using a new simulator seed',selection_wall_s=time.perf_counter()-began)
    seen={case_id(r['scenario']) for r in history}
    pool=[s for s in candidates() if case_id(s) not in seen]
    assert pool
    if method=='model':
        usable=[r for r in warm+history if r['label'] is not None]
        observed=[(r['scenario'],r['label']) for r in usable]
        mu,std=posterior(observed,pool);score=mu+.5*std
        rank=sorted(range(len(pool)),key=lambda j:(-float(score[j]),case_id(pool[j])))
        decision=dict(reason='Maximum GP mean + 0.5 posterior standard deviation',
            training_trial_ids=[r['trial'] for r in usable],
            ranking=[dict(scenario=pool[j],mean=float(mu[j]),std=float(std[j]),score=float(score[j])) for j in rank])
        selected=pool[rank[0]]
    else:
        order=fixed_order() if method=='fixed' else [candidates()[int(i)] for i in np.random.default_rng(700+episode).permutation(len(candidates()))]
        selected=next(s for s in order if case_id(s) not in seen)
        decision=dict(reason='Frozen risk-ordered sweep' if method=='fixed' else 'Frozen seeded random permutation')
    decision['selection_wall_s']=time.perf_counter()-began
    return selected,False,decision


def confirmed(history):
    return bool(history and history[-1]['confirmation'] and history[-1]['label'] and len(history)>=2 and history[-2]['label'] and case_id(history[-1]['scenario'])==case_id(history[-2]['scenario']))


def verification():
    return [dict(scenario=scenario(d,0,x),seed=seed) for x,d,seed in
        [(TRAIN_X,0.,100),(TRAIN_X,1.6,100),(TEST_X,0.,100),(TEST_X,1.6,100),(TEST_X,0.,111),(TEST_X,1.6,111)]]
