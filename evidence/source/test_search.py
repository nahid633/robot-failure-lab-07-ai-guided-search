import unittest
import numpy as np
import core
from search import *


class SearchTests(unittest.TestCase):
    def test_single_observation_matches_analytic_posterior(self):
        s=scenario(.8,0.,TRAIN_X);mu,std=posterior([(s,1)], [s])
        self.assertAlmostEqual(mu[0],.5+.5/(1+NOISE**2+1e-9))
        self.assertAlmostEqual(std[0]**2,1.-1/(1+NOISE**2+1e-9))
    def test_learning_changes_ranking(self):
        low=scenario(0,0,TRAIN_X);high=scenario(1.6,0,TRAIN_X)
        mu,_=posterior([(low,0),(high,1)],[low,high]);self.assertGreater(mu[1],mu[0])
    def test_uncertainty_grows_away_from_observation(self):
        s=scenario(.8,0,TRAIN_X);_,std=posterior([(s,1)],[s,scenario(.8,.35,TEST_X)])
        self.assertGreater(std[1],std[0])
    def test_conflicting_repeats_numerically_stable(self):
        s=scenario(.8,0,TRAIN_X);mu,std=posterior([(s,0),(s,1)],[s]);self.assertAlmostEqual(mu[0],.5);self.assertTrue(np.isfinite(std[0]))
    def test_model_visibility_audit(self):
        warm=[dict(scenario=s,label=i%2,trial='warm'+str(i)) for i,s in enumerate(training())]
        _,_,d=choose('model',1,warm,[]);self.assertEqual(d['training_trial_ids'],[r['trial'] for r in warm])
    def test_unreached_trial_does_not_become_training_label(self):
        warm=[dict(scenario=training()[0],label=0,trial='usable'),dict(scenario=training()[1],label=None,trial='unreached')]
        _,_,d=choose('model',1,warm,[]);self.assertEqual(d['training_trial_ids'],['usable'])
    def test_confirmation_reuses_case_within_budget(self):
        r=dict(scenario=candidates()[2],label=1,confirmation=False,trial='x')
        s,c,d=choose('random',1,[],[r]);self.assertTrue(c);self.assertEqual(s,r['scenario']);self.assertFalse(confirmed([r]))
        repeat=dict(r,confirmation=True);self.assertTrue(confirmed([r,repeat]));self.assertFalse(confirmed([r,dict(repeat,label=0)]))
    def test_failed_confirmation_does_not_repeat_forever(self):
        s=candidates()[2];h=[dict(scenario=s,label=1,confirmation=False),dict(scenario=s,label=0,confirmation=True)]
        chosen,c,_=choose('random',1,[],h);self.assertFalse(c);self.assertNotEqual(chosen,s)
    def test_random_is_reproducible_without_replacement(self):
        first=choose('random',2,[],[])[0];self.assertEqual(first,choose('random',2,[],[])[0])
        next_case=choose('random',2,[],[dict(scenario=first,label=0,confirmation=False)])[0];self.assertNotEqual(first,next_case)
    def test_fixed_baseline_is_strong(self):
        self.assertEqual(fixed_order()[0],scenario(1.6,0.,TEST_X))
    def test_group_holdout(self):
        self.assertEqual(len(training()),9);self.assertEqual(len(candidates()),25)
        self.assertTrue(all(s['x']==TRAIN_X for s in training()));self.assertTrue(all(s['x']==TEST_X for s in candidates()))
        self.assertFalse(set(map(case_id,training()))&set(map(case_id,candidates())))
    def test_configured_gate_boundaries(self):
        core.configure(scenario(.4,0.,TRAIN_X))
        self.assertEqual(core.policy('restamped',3,3),'restamped');self.assertEqual(core.policy('restamped',3.4,3),'fresh')
        core.configure(scenario(0.,0.,TRAIN_X));self.assertEqual(core.policy('restamped',3,3),'fresh')
    def test_independent_geometry_and_late_stop(self):
        core.configure(scenario(1.6,0.,TRAIN_X))
        self.assertEqual(core.clearance(dict(x=-.6,y=.55,yaw=0)),0.)
        odom=[dict(stamp=i*.1,speed=0.,w=.2) for i in range(20)];self.assertIsNone(core.sustained_stop(odom,0,2))
    def test_configured_obstacle_changes_geometry(self):
        core.configure(scenario(.8,.35,TEST_X));self.assertEqual(core.clearance(dict(x=TEST_X,y=.90,yaw=0)),0.)
        self.assertGreater(core.clearance(dict(x=-.9,y=.2,yaw=0)),.1)
    def test_gate_decision_can_precede_publication_across_boundary(self):
        core.configure(scenario(.4,0,TRAIN_X))
        self.assertEqual(core.policy('restamped',3.399,3.),'restamped')
        self.assertEqual(core.policy('restamped',3.401,3.),'fresh')


if __name__=='__main__':unittest.main()
