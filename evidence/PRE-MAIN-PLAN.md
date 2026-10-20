---
type: experiment
status: active
updated: 2026-10-20
tags: [robot-failure-lab, ai-guided-testing, gaussian-process, simulation]
sources: [robot-failure-lab-experiment-menu, robot-failure-lab-lessons, robot-failure-lab-05-frozen-scans, replay-and-world-model-plan, robot-failure-lab]
confidence: medium
---

# Robot Failure Lab 07: AI-guided failure search

## Selection and question

Founder selected menu choice 19 on October 20: “lets do this experiment AI-guided failure search.” Implement and execute a bounded local comparison of model-guided search, random search and a simple fixed sweep; retain honest results and a sharing package. This does not select the proposed [[robot-failure-lab-06-sensor-timeout]]. Publication/outbound messages are separate.

Question: can a small learned test selector find a reproduced obstacle-contact scenario using fewer simulator evaluations or less wall time than simple baselines, after accounting for its initial data cost?

Prior [[robot-failure-lab-05-frozen-scans]] supplies independently recorded geometry/contact and scan-provenance infrastructure, not model training data. Its [findings](../../artifacts/robot-failure-lab-05/FINDINGS.md) were read. [[replay-and-world-model-plan]] motivates a bounded model-versus-random-versus-fixed comparison; its older implementation descriptions remain dated history. A test selector is not an action-conditioned world model.

## Frozen first protocol

Separate code `code/robot-failure-lab/experiment07/`; earlier source/archives remain intact. Use the same ROS2 Jazzy/Nav2/Gazebo stack, map, start/goal, fresh AMCL branch and shared costmap/collision-monitor branch as Lab05. All navigation settings remain identical, including the enabled one-second monitor timeout and 0.25 m goal tolerance. Full original/output serialized scans, canonical payload hashes, observer receipts, world poses, contact events, commands and seven-node runtime configuration snapshots are retained.

New dimensions: duration of cached/restamped scan fault and lateral obstacle offset. Original acquisition continues. Same obstacle dimensions (0.30 x 0.45 x 0.60 m), parked pose, moving trigger and onset+0.20 s insertion request. Observe actual placement from world pose. Source truth never feeds navigation TF or robot control.

Training corpus: nine fresh-process runs at obstacle x=-0.60 m; duration {0,0.8,1.6}s x offset {-0.35,0,0.35}m relative to y=0.55. Training seeds 120..128 in declared Cartesian order. Require both binary labels to occur; otherwise stop main feasibility rather than invent a useful learning result.

Search context: obstacle x=-0.30 m, withheld from model training. Candidate duration {0,0.4,0.8,1.2,1.6}s x offset {-0.35,-0.175,0,0.175,0.35}m, 25 scenarios. The geometry grouping changes obstacle x by a full box length. Feasibility verification includes this context before main; it is withheld from model training, not a human-blinded external test. No context/parameter/model tuning is allowed after main starts.

Three search episodes. Discovery Gazebo seeds 201/202/203; immediate confirmation uses 301/302/303. AMCL/MPPI/scheduling are uncontrolled. Same-seed tests are not deterministic. Policies are interleaved model/random/fixed in episode 1, random/fixed/model in episode 2, fixed/model/random in episode 3, cycling one evaluation each while eligible.

Model: fixed-hyperparameter Gaussian process regression surrogate of a binary observed-contact indicator, with prior mean 0.5, observation noise std 0.2, normalized-feature length scales [0.6,0.65,1.0]. Acquire maximum posterior mean + 0.5 posterior std, ties by canonical scenario hash. These are acquisition scores, not calibrated probabilities. Fit with NumPy linear algebra; no scikit-learn installation/external training service is needed. Save complete ranking, input trial IDs and selection time before each live query. Updates see only the nine warm runs and that model policy's own episode history; other policies/episodes and verification labels are excluded.

Random baseline: predeclared seeded permutation (700+episode), without replacement except required confirmation. Fixed baseline: descending duration, centered obstacle then increasing absolute offset, with negative offset first on side ties. This deliberately strong simple baseline is specified before verification outcomes, rather than a weak sweep constructed to make the model win.

Each policy/episode receives total standalone budget 15 simulator evaluations. Model pays all nine warm calls, leaving six held-out query/confirmation calls. Baselines pay no training cost and receive fifteen held-out calls. A positive discovery must be immediately repeated at the same parameter values using the new confirmation seed; confirmation consumes a call. Stop the policy at its first two-positive confirmed scenario; otherwise right-censor it at budget, including any positive last call lacking a confirmation. Never extend the sample because a policy did poorly.

Physically generate the nine warm runs once and share them across episodes. Charge them in full in every standalone model comparison, and separately report actual unique workload and amortized totals with one physical training charge. The episodes share a dataset and are not independent retraining experiments. Maximum main workload is 117 evaluations (nine warm plus 3 x [6+15+15] search), with early termination after confirmed failures. Count fit/selection time and failed-start wall overhead. Report verification/setup costs separately; do not claim savings solely from fewer held-out calls.

## Failure target and measurements

Primary binary target: at least one received obstacle-specific contact event between independently confirmed appearance and exactly onset+6 simulated seconds. Store first contact and event count. Zero means no received event, not a continuous no-contact certificate. Existing Lab05 positive-contact calibration (667 messages) is reused only if the actual instrumented robot/world hashes match exactly; no unverified contact interface is introduced.

Each run observes through that fixed horizon, then intentionally cancels an active navigation action and retains at least 1.8 s passive tail. Contacts after cutoff do not train the model. Active at horizon/cancellation is not a mission timeout or a completed-goal failure. Natural task termination, if any, is recorded separately. This experiment targets contact discovery, not journey completion.

Retain independent sampled yaw-aware conservative-envelope clearance and actual sustained stop (translation <=0.01 m/s and absolute yaw <=0.03 rad/s for >=0.3 s, gaps <=0.15 s). Conservative-envelope overlap is not itself contact proof. No external proximity-stop guard or replacement goal.

Compare first-confirmed-failure total calls and wall time, held-out calls, censoring and exact discovered scenario. Show all nine policy/episode ledgers and every training/search/confirmation outcome. No single winner, general advantage, statistical significance, diverse failure mechanisms or physical safety is assumed.

## Verification, bounds and implementation checks

Six excluded navigation checks, declared before main: train-context fresh/1.6s centered fault (seed100), search-context fresh/1.6s centered fault (seed100), repeated search-context fresh/1.6s centered fault (seed111). Frozen main requires labels [0,1,0,1,0,1], validated provenance/routing/geometry/coverage and identical live settings across checks. These labels do not train the model or tune its parameters. A gate failure is retained and requires an explicit before-main protocol revision, not omitted evidence.

Bounds: startup 75 wall seconds after worker launch; pre-trigger 45 simulated seconds; scenario 150 wall seconds after sending goal; observation tail 30 wall seconds; whole worker 260 wall seconds. Startup/protocol errors stop the batch. Guarded continuation only after inspection of pre-READY/pre-goal failures, at most three starts per request; no completed outcome retry. Preserve all request decisions and attempts. Sequential ROS domain leases 10..99, unique Gazebo partitions; domains recycle only after full earlier process-group shutdown.

Thirteen focused tests pass before first verification, including analytic single-observation GP posterior, learning/uncertainty, conflicting repeats, own-history input audit, confirmation budget behavior, deterministic baselines, grouped holdout, gate boundaries and variable-obstacle geometry. Syntax compilation passed. Source/input/package hashes and exact plan are captured before any run. Next: complete six verification checks; if they establish feasibility, generate the nine warm rows and execute the frozen comparison.

## Technical basis and limits

Primary references checked October 20: [Adaptive Stress Testing](https://arxiv.org/abs/1811.02188) establishes prior art in simulation failure search; [Gaussian Processes for Machine Learning, chapter 2](https://gaussianprocess.org/gpml/chapters/RW2.pdf) supports the regression posterior used. This small GP regresses a binary indicator for ranking; it is not a GP classifier with calibrated probabilities or a novel search algorithm.

Only one route, one withheld placement context, three episodes and few labels. Prior knowledge from Lab05 and target-context feasibility checks mean this is not a blinded benchmark. No Vektor integration, action-conditioned world model, general simulator/physical-test reduction, customer adoption or publication is established. Retain ties and negative findings; outside reproduction remains unverified.

## Retained development history before main

Initial setup stopped before any simulator launch because the previous Lab05 `/tmp` calibration directory was absent. Referencing its immutable workspace archive passed the robot/world hash check. The subsequent `/tmp` verification recorded two outcomes visible in tool history: fresh label 0 (0.144 m clearance, validated) and 1.6 s fault label 1 (result observed, validation not confirmed). A cached-origin/stamp boundary assertion then stopped the batch. After exit, this WSL session no longer retained the temporary directories, so full original packets/logs/manifests for those attempts are unavailable. Their observations and the error are explicitly reconstructed in `artifacts/robot-failure-lab-07-working/excluded-checks/transient-verification-loss/EVIDENCE-GAP.json`; this is tool-history reconstruction, not recovered raw simulator evidence. The two outcomes are excluded from training/main and must never be described as two passed checks.

Pre-main correction: read trigger time after `isTaskComplete` has finished spinning callbacks and while holding the routing lock. Record scan-gate decision time separately from actual publication time, and audit the gate against its decision time. Both times and their difference remain available. Fourteen focused tests now pass. A fresh six-check verification set writes directly into persistent workspace storage at `artifacts/robot-failure-lab-07-working/verification-v3/`.

Before any main/training outcome, the excluded verification-outcome budget is explicitly revised from six total to eight: the two incompletely retained earlier outcomes plus the six new final checks. The pre-simulator setup error is separate. Source/configuration/model/search settings and observation target are not tuned using these outcomes; the change fixes capture/storage. Main still requires six complete validated checks with the frozen final source. The original evidence gap limits the development audit even if later main records pass.

The first persistent v3 check did not reach the trigger within 45 simulated seconds: the robot remained near its start while controller progress failures/recoveries occurred. Its complete logs/poses/scans and goal-submitted evidence are retained; no fault or obstacle insertion occurred. The earlier worker raised rather than producing a missing-target outcome. Before main, the worker now records a charged `scenario not reached` evaluation with null label, cancels any still-active goal and retains the passive tail. Such runs are never labeled contact-free, retried, or used to fit the GP. They consume the unchanged 9 warm-call/15 total-policy budgets. Training requires at least six usable rows and both observed labels among its nine calls. Capture/protocol errors still stop execution.

The excluded before-main budget is explicitly revised to nine navigation attempts (two temporary outcomes with the evidence gap, one retained unreached v3 attempt, six final v4 checks). Fifteen focused tests now pass, including exclusion of missing target labels from training. Final checks are at persistent `artifacts/robot-failure-lab-07-working/verification-v4/`. Scene, parameter ranges, model hyperparameters, policies and contact target remain unchanged. Animation selection is the first actually reached episode-1 scenario per policy; unreached attempts remain visible in all outcome/cost tables.
