# Robot Failure Lab 07: AI-guided failure search

Live Gazebo/Nav2 simulator queries. Model, random and fixed sweep, three search episodes in one held-out obstacle-placement context.

| Episode | Policy | Confirmed contact scenario | Held-out calls | Charged training calls | Total standalone calls | Charged wall seconds |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | model | not within budget | 6 | 9 | 15 (censored) | 997.8 |
| 1 | random | yes | 12 | 0 | 12 | 804.1 |
| 1 | fixed | yes | 3 | 0 | 3 | 157.9 |
| 2 | model | yes | 3 | 9 | 12 | 721.0 |
| 2 | random | yes | 4 | 0 | 4 | 262.2 |
| 2 | fixed | yes | 3 | 0 | 3 | 154.7 |
| 3 | model | yes | 4 | 9 | 13 | 905.5 |
| 3 | random | yes | 10 | 0 | 10 | 446.5 |
| 3 | fixed | yes | 2 | 0 | 2 | 83.9 |

## Cost and interpretation

Charged wall values sum recorded simulator attempts and saved selection timings. They exclude manual inspection pauses, coordinator replay checks and report generation; simulator-call budget is the primary comparison. BATCH-ACCOUNTING.json separately records observed batch elapsed time.

56 unique main simulator evaluations: nine warm runs plus 47 live search/confirmation calls; 3341.4 wall seconds including starts/shutdown and charged failed-start overhead. The nine warm runs were generated once, shared physically, and charged in full to the model in each standalone comparison. Baselines do not pay a model-training charge. Three episodes share that initial dataset and are not independent training experiments.

Amortized observed wall totals across all three episodes (one physical training charge): model 1432.1s, random 1512.8s, fixed 396.4s. These totals describe this chosen workload, not general compute savings.

A smaller held-out query count alone is not evidence of lower end-to-end testing cost. Compare the charged total, wall time, censoring and confirmation requirements. The fixed baseline deliberately tries the longest centered fault first, rather than an artificially weak sweep.

## What the model does

Initial corpus: 9 charged evaluations; 8 usable labels; 1 missing labels excluded from fitting. None in the outcome table means the scenario was not reached, never a zero-event label.

A fixed-hyperparameter Gaussian process regression surrogate of binary contact-event observations uses mean+0.5 posterior standard deviation to select tests. Scores are not calibrated failure probabilities. Training used obstacle x=-0.60m; every live search used previously withheld x=-0.30m. Later updates see only that policy/episode's observed labels plus the initial nine rows. Saved rankings/input IDs were replayed and checked without other policies' labels. Verification outcomes never train the model.

The task, obstacle dimensions and navigation settings are fixed. Search varies cached/restamped scan duration and obstacle lateral offset. A failure is an obstacle-specific contact event between independently measured obstacle appearance and onset+6s; a confirmed failure repeats the same case in a fresh process with a new Gazebo seed. Zero labels mean no received event, not certified physical clearance. Positive contact calibration is retained with matching model/world hashes.

Still-active navigation actions are intentionally canceled after the fixed observation horizon. That cancellation is not a mission timeout or goal failure. Contacts during the passive tail are excluded from model labels. Conservative-envelope clearance is independent geometry; zero overlap is not itself contact proof. AMCL receives fresh scans, so this is a synthetic downstream-branch fault, not a complete physical scanner failure.

This is one simulated route, one held-out context, three episodes, few labels and uncontrolled AMCL/MPPI/scheduling. Repeated seeds do not guarantee identical trajectories. No general search-efficiency, diversity-of-mechanisms, physical-safety, world-model, algorithmic-novelty, Vektor integration or customer-use claim follows.

## Every main evaluation

| Trial | Role / policy | Episode | Duration (s) | Offset (m) | Obstacle x (m) | Contact label | Confirmation attempt |
| --- | --- | --- | --- | --- | --- | --- | --- |
| trial-0001 | training | — | 0.0 | -0.350 | -0.60 | 0 | False |
| trial-0002 | training | — | 0.0 | 0.000 | -0.60 | 0 | False |
| trial-0003 | training | — | 0.0 | 0.350 | -0.60 | 0 | False |
| trial-0004 | training | — | 0.8 | -0.350 | -0.60 | 1 | False |
| trial-0005-attempt-2 | training | — | 0.8 | 0.000 | -0.60 | None | False |
| trial-0006 | training | — | 0.8 | 0.350 | -0.60 | 1 | False |
| trial-0007 | training | — | 1.6 | -0.350 | -0.60 | 1 | False |
| trial-0008-attempt-2 | training | — | 1.6 | 0.000 | -0.60 | 1 | False |
| trial-0009 | training | — | 1.6 | 0.350 | -0.60 | 1 | False |
| trial-0010 | model | 1 | 1.2 | -0.350 | -0.30 | 0 | False |
| trial-0011-attempt-2 | random | 1 | 0.4 | 0.350 | -0.30 | None | False |
| trial-0012 | fixed | 1 | 1.6 | 0.000 | -0.30 | None | False |
| trial-0013 | model | 1 | 1.2 | 0.350 | -0.30 | 0 | False |
| trial-0014 | random | 1 | 0.0 | 0.000 | -0.30 | None | False |
| trial-0015 | fixed | 1 | 1.6 | -0.175 | -0.30 | 1 | False |
| trial-0016 | model | 1 | 1.6 | 0.000 | -0.30 | None | False |
| trial-0017 | random | 1 | 1.2 | -0.350 | -0.30 | 0 | False |
| trial-0018 | fixed | 1 | 1.6 | -0.175 | -0.30 | 1 | True |
| trial-0019 | model | 1 | 1.2 | 0.000 | -0.30 | None | False |
| trial-0020 | random | 1 | 1.6 | 0.350 | -0.30 | 1 | False |
| trial-0021 | model | 1 | 0.8 | 0.000 | -0.30 | 0 | False |
| trial-0022 | random | 1 | 1.6 | 0.350 | -0.30 | None | True |
| trial-0023-attempt-2 | model | 1 | 1.6 | 0.175 | -0.30 | 1 | False |
| trial-0024 | random | 1 | 0.8 | 0.175 | -0.30 | 0 | False |
| trial-0025 | random | 1 | 0.8 | 0.350 | -0.30 | 0 | False |
| trial-0026 | random | 1 | 1.2 | 0.000 | -0.30 | None | False |
| trial-0027-attempt-2 | random | 1 | 1.2 | -0.175 | -0.30 | 1 | False |
| trial-0028 | random | 1 | 1.2 | -0.175 | -0.30 | None | True |
| trial-0029 | random | 1 | 1.6 | -0.175 | -0.30 | 1 | False |
| trial-0030-attempt-2 | random | 1 | 1.6 | -0.175 | -0.30 | 1 | True |
| trial-0031 | random | 2 | 1.2 | -0.175 | -0.30 | 0 | False |
| trial-0032 | fixed | 2 | 1.6 | 0.000 | -0.30 | None | False |
| trial-0033 | model | 2 | 1.2 | -0.350 | -0.30 | 0 | False |
| trial-0034 | random | 2 | 1.2 | 0.000 | -0.30 | 0 | False |
| trial-0035 | fixed | 2 | 1.6 | -0.175 | -0.30 | 1 | False |
| trial-0036 | model | 2 | 1.2 | 0.350 | -0.30 | 1 | False |
| trial-0037 | random | 2 | 1.6 | -0.175 | -0.30 | 1 | False |
| trial-0038 | fixed | 2 | 1.6 | -0.175 | -0.30 | 1 | True |
| trial-0039 | model | 2 | 1.2 | 0.350 | -0.30 | 1 | True |
| trial-0040-attempt-2 | random | 2 | 1.6 | -0.175 | -0.30 | 1 | True |
| trial-0041 | fixed | 3 | 1.6 | 0.000 | -0.30 | 1 | False |
| trial-0042-attempt-2 | model | 3 | 1.2 | -0.350 | -0.30 | None | False |
| trial-0043 | random | 3 | 0.0 | -0.175 | -0.30 | 0 | False |
| trial-0044 | fixed | 3 | 1.6 | 0.000 | -0.30 | 1 | True |
| trial-0045 | model | 3 | 1.2 | 0.350 | -0.30 | 0 | False |
| trial-0046 | random | 3 | 0.4 | -0.175 | -0.30 | 0 | False |
| trial-0047 | model | 3 | 1.6 | -0.350 | -0.30 | 1 | False |
| trial-0048 | random | 3 | 0.4 | 0.350 | -0.30 | 0 | False |
| trial-0049 | model | 3 | 1.6 | -0.350 | -0.30 | 1 | True |
| trial-0050 | random | 3 | 0.8 | 0.000 | -0.30 | None | False |
| trial-0051 | random | 3 | 1.2 | 0.350 | -0.30 | 1 | False |
| trial-0052 | random | 3 | 1.2 | 0.350 | -0.30 | 0 | True |
| trial-0053 | random | 3 | 0.8 | -0.175 | -0.30 | 0 | False |
| trial-0054 | random | 3 | 0.0 | 0.000 | -0.30 | 0 | False |
| trial-0055 | random | 3 | 1.6 | -0.350 | -0.30 | 1 | False |
| trial-0056 | random | 3 | 1.6 | -0.350 | -0.30 | 1 | True |

Full packet/pose/contact evidence, exact policies and acquisition rankings, startup logs, parameter audits, frozen source/model/map/configuration/package hashes and excluded verification are retained. Fresh-machine reproduction is unverified. Nothing has been posted or sent.
