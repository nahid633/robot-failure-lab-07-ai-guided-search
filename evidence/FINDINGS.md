# AI-guided failure search: measured results

Measured October 20, 2026. 56 unique main evaluations: nine initial data runs and 47 live search/confirmation calls.

| Policy | Confirmed episodes | Held-out calls per episode | Total standalone calls per episode, including training |
| --- | --- | --- | --- |
| model | 2/3 | 6, 3, 4 | 15+, 12, 13 |
| random | 3/3 | 12, 4, 10 | 12, 4, 10 |
| fixed | 3/3 | 3, 3, 2 | 3, 3, 2 |

Against random: model lower charged confirmation cost in 0/3 episodes; random lower in 3/3; equal confirmed cost 0; both exhausted budget 0.
Against fixed: model lower charged confirmation cost in 0/3 episodes; fixed lower in 3/3; equal confirmed cost 0; both exhausted budget 0.

Observed campaign with the initial corpus charged only once: model 22 calls / 2/3 confirmed episodes, random 26 calls / 3/3 confirmed episodes, fixed 8 calls / 3/3 confirmed episodes. Different confirmation success/censoring means these raw campaign totals alone are not an efficiency ranking.

Initial nine-call corpus: 5 received-contact labels, 3 zero-event labels, 1 scenario-not-reached/missing labels. The GP used 8 usable rows.

## Contact observations and missing targets

| Policy / episode | Positive contact calls (including confirmations) | Zero-event calls | Scenario not reached |
| --- | --- | --- | --- |
| model / 1 | 1 | 3 | 2 |
| random / 1 | 4 | 3 | 5 |
| fixed / 1 | 2 | 0 | 1 |
| model / 2 | 2 | 1 | 0 |
| random / 2 | 2 | 2 | 0 |
| fixed / 2 | 2 | 0 | 1 |
| model / 3 | 2 | 1 | 1 |
| random / 3 | 3 | 6 | 1 |
| fixed / 3 | 2 | 0 | 0 |

A positive call is not itself a confirmed two-run scenario. Positive discoveries without a positive confirmation inside budget remain visible here even when the search is censored.

## How to read the result

- Each confirmed failure has two observed-contact runs at the same parameters with different Gazebo seeds. Confirmations consume budget; + denotes censoring at the budget. No censored run is a completed successful search.
- Charged policy wall time sums simulator-attempt and saved fit/selection timings; it excludes manual inspection pauses, coordinator replay checks and report generation. BATCH-ACCOUNTING.json separately records observed batch elapsed. Call budgets are the primary comparison; early development evidence loss prevents exact full-session cost accounting.
- The model receives a total standalone budget of 15 calls including nine initial data calls, leaving six held-out calls. Random and fixed have 15 held-out calls and no training charge. Nine initial runs were physically generated once and shared across episodes. REPORT.md also reports wall time and amortized unique workload; the three episodes are not independent retraining experiments.
- The fixed baseline is deliberately strong: longest fault first, centered obstacle first. Fewer held-out calls alone do not establish savings after training. All methods may tie or the fixed sweep may win; no AI benefit is assumed.
- Pre-goal startup failures and scenario-not-reached calls expose oracle/infrastructure variability. With these few episodes, differences in missing labels or wall time cannot be attributed entirely to the selection algorithm. Report those counts alongside successful queries.
- Training and search use different obstacle-x contexts (-0.60m versus -0.30m). The latter was withheld from model training, but checked during feasibility verification, so this is not a human-blinded external benchmark. Online model updates see only its own episode plus initial data; other methods/episodes and verification labels are excluded.
- The fixed sweep's first centered long-fault case is also one of the excluded feasibility checks, repeated there with different seeds. Its severity ordering was specified before those outcomes, but this remains a prevalidated failure family, not blinded discovery of a wholly unknown bug class or a broad comparison of AI methods.
- The target is an obstacle-specific contact event within six simulated seconds of fault onset. A zero event label is not a continuous no-contact certificate. Source provenance, independent world geometry and calibrated contacts are retained. Conservative-envelope overlap is not itself physical contact. Still-active navigation is intentionally canceled after the horizon, which is not a mission failure or journey timeout.
- Scenario-not-reached evaluations consume budget with a missing label and never train the model. They are retained, never retried and never treated as contact-free. All physical attempts, model requests/rankings, budget/visibility audits and configuration hashes are archived.
- During main, a pre-READY parameter-service discovery failure exposed a resume-cursor defect in the coordinator. A separately hashed coordination adapter reconstructs the predeclared round-robin order from saved outcomes and resumes the pending request. COORDINATION-AMENDMENT.json records this change; original frozen worker/model/policy sources remain intact. Every final decision and the full chronology are independently replayed.
- One simulated route, one model-training-held-out context and three shared-training episodes do not establish general sample efficiency, physical safety, diverse failure mechanisms, a Nav2 defect, an action-conditioned world model, Vektor integration or customer adoption.

## Development audit and limits

Initial setup used an absent old /tmp calibration path and launched no simulator. A subsequent temporary verification recorded two outcomes visible in tool history, then failed a packet-boundary assertion. Their full transient files were lost after WSL exit. The archive explicitly retains a reconstructed evidence-gap record and source copy; it does not claim to retain those original packets/logs. The first outcome was validated; the second only had a recorded contact result before the failed audit.

Before main, trigger time was moved after pending callbacks under the routing lock, gate decision/publication times were separated, and output moved to persistent storage. A subsequent persistent verification never reached its scenario trigger; full evidence is retained. Explicit missing-label/budget handling was added before main. Revised final verification and every main record use matching frozen sources. Excluded outcomes never train the model. Verification-budget revisions are recorded in the wiki/protocol development notes, not hidden.

Fifteen focused tests passed before final verification. Main packet/origin/receipt/geometry/label audits, all nine policy decision replays, cross-run parameter checks, budget/confirmation accounting, frozen hashes and visual inspections are recorded in IMPLEMENTATION-CHECKS.json and EVIDENCE-VALIDATION.json. The prior contact calibration is included with matching robot/world hashes.

## Files

REPORT.md and SEARCH-LEDGER.json show every episode and trial. INITIAL-MODEL.json preserves fitted-data inputs and the first actual acquisition ranking. search-cost.png separates held-out and total calls; initial-model.png shows initial scores. first-search-recorded.gif reconstructs episode 1's first reached scenario per policy from measured simulated world poses, not camera footage.

reproduction/ contains standalone source and measured package versions. Public sharing and clean-machine reproduction remain unverified. SHARING-DRAFT.md is unsent.
