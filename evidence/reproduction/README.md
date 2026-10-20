# Robot Failure Lab 07: AI-guided failure search

Local ROS2 Jazzy / Gazebo 8 / Nav2 experimental test selection. A small Gaussian
process regression surrogate learns a binary **observed obstacle-contact event**
label. It selects test scenarios, not robot actions or synthetic sensor feedback.
Its mean/UCB scores are not calibrated failure probabilities. No world-model or
algorithmic-novelty claim is made.

Freeze all source and this protocol before verification/main. Six excluded
navigation verification outcomes cover fresh and 1.6s restamped faults in train
and held-out obstacle contexts, with held-out repeats. Main requires labels
0,1,0,1,0,1 in that order and unchanged source/configuration/packages. Do not tune
the scene/model to make the learned policy win. The existing positive contact
calibration is reused only after exact robot/world hash checks.

Training: nine new runs at obstacle x=-0.60 m, Cartesian product fault durations
0,0.8,1.6s and lateral offsets -0.35,0,0.35m around y=0.55. At least one observed
contact and one zero event label among at least six usable rows are required. Earlier Lab05 outcomes and excluded
verification labels are never model training inputs.

Search: all candidate scenarios have held-out obstacle x=-0.30 m. Candidate grid
durations 0,0.4,0.8,1.2,1.6s; offsets -0.35,-0.175,0,0.175,0.35m. Three episodes,
discovery seeds 201/202/203, confirmation seeds 301/302/303. Matched Gazebo seeds
do not control AMCL/MPPI/scheduling or guarantee identical trajectories.

Policies: GP mean+0.5std, seeded random permutation, and a deliberately strong
fixed grid starting with longest fault/centered obstacle. Model hyperparameters
are fixed in search.py. It may update only from its own episode's labels plus
the common nine warm runs. No other policy/episode's outcomes are visible.
Model queries and all candidate scores/input trial IDs are saved before execution.

Each policy has total standalone budget 15 simulator evaluations per episode:
the model pays for all nine warm runs, leaving six held-out calls; baselines
have 15 held-out calls and no training cost. A positive candidate is immediately
retested with a fresh confirmation seed, consuming another call. Two positives
confirm a failure and stop that policy's episode. Otherwise the result is
right-censored at its budget; no extra samples. Training is physically generated
once and shared across the three episodes; also report that amortized total and
the actual unique simulation workload. Episodes therefore share training and
are not three independent retraining experiments. Maximum main workload 117
evaluations (nine training plus 108 search); early successful searches use fewer.
Verification and infrastructure overhead are reported separately. Include model
fit/selection time and failed-start overhead in wall-time accounting.

The robot, map/start/goal, navigation settings, moving trigger, observer geometry
and original sensor stream match Lab05. AMCL always receives originals; costmaps
and collision monitor share the experimental branch. Scan contents are frozen
and restamped only during the requested window. Obstacle appearance is independently
confirmed after onset+0.20s. Preserve full serialized scans and canonical hashes.

Every run observes through onset+6s. The label is a contact event between obstacle
appearance and that exact cutoff. Then intentionally cancel a still-active action
and retain a passive 1.8s tail. This cancellation is neither a task failure nor a
goal timeout; original goal completion is not the search target. Do not train on
post-cutoff events. Independent sampled conservative-envelope clearance is a
separate metric; zero envelope clearance is not itself physical contact. The
contact sensor is event-driven: absent events are not a continuous no-contact
certificate. World truth never enters navigation TF/control.

A natural navigation terminal or 45-sim-second deadline before reaching the trigger
is a charged "scenario not reached" evaluation with missing target label, never
zero/contact-free. It is not retried and does not train the GP. It still consumes
policy/warm budget and is shown in the outcome table. Retain its full evidence.
This handling was added before main after one retained verification navigation
attempt made little progress and never reached the trigger; prior source survives.

Bounds: startup 75 wall seconds, pre-trigger 45 simulated seconds, scenario worker
150 wall seconds after sending goal, tail 30 wall seconds, overall worker 260 wall
seconds. Infrastructure errors stop the batch. At most three starts per request;
guarded continuation only before READY/goal/cutoff/result. Retain all attempts;
never retry a completed outcome. Sequential low ROS domains 10..99, rotating only
after full prior process-group shutdown, unique Gazebo partition each attempt.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest -v test_search
python3 run.py ./results-verification --phase verify --calibration-root ../excluded-checks/reused-positive-contact-calibration
python3 run.py ./results-main --phase main --verification-root ./results-verification --calibration-root ../excluded-checks/reused-positive-contact-calibration
python3 report.py ./results-main
```

Use new directories; inspect startup logs before adding --resume-infrastructure.
Use persistent working directories: this host cleared /tmp after the initial
verification exited. Two observed outcomes and the ensuing boundary-audit failure
survive only in tool-history reconstruction, explicitly excluded and documented
as an evidence gap. The revised recorder timestamps the trigger after pending ROS
callbacks under the routing lock and records gate decision time separately from
actual publication time. Original source copy and the gap record are retained.
Exact ROS/Gazebo packages, original files, map, model and source hashes are retained.
Requires Python3, NumPy, matplotlib/Pillow, PyYAML and the installed ROS simulation
packages. No scikit-learn, external model download or paid training service needed.

Only one simulated route, one held-out placement context and three search episodes.
This is an exploratory local feasibility comparison; no general search-efficiency,
physical-safety, Vektor integration, customer-adoption or independent reproduction
claim follows. Report ties, censoring and negative results. Publication/outbound
messages are separate actions.

Resume correction: after all nine warm calls exist, use `python3 continue-round-robin.py /path/to/main` after inspecting a pre-READY/pre-goal startup failure. It preserves the saved request and reconstructs the declared interleaving. The original runner resume loop can restart at the wrong policy slot during search. Before the warm corpus is complete, the runner's guarded --resume-infrastructure is suitable. The adapter/amendment and original sources are retained separately.

These reproduction instructions use persistent relative output folders and the packaged prior positive-contact calibration. A geometry/package change needs newly verified calibration. Oracle/model/policy code matches the measured source; report.py uses the retained reviewed renderer to clarify usable-label counts and first-reached animation captions. Original measured sources remain in source/. Fresh-machine reproduction is unverified.
