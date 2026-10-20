# Robot Failure Lab 05: live topics, frozen observations

Standalone ROS 2 Jazzy / Gazebo 8 / Nav2 TurtleBot3 comparison. All conditions
keep AMCL supplied with fresh scans. Both costmaps and collision monitor share
one experimental branch. Fresh / missing / frozen-original-stamp /
frozen-new-stamp policies affect the same two-second window; original sensing
continues upstream. Frozen-new-stamp explicitly models a faulty publisher.
No collision-monitor setting, goal tolerance or other navigation parameter varies.

Start (-2,-0.5), goal (1,0.5), yaw zero. Trigger after >=3 simulated seconds,
world x crosses -1.30 with y in [0.25,0.85] while moving >=0.05m/s for >=0.3s.
Cache the last pre-window observation. At onset +0.20s, request the same
0.30 x 0.45 x 0.60m stationary obstacle at (-0.60,0.55), z=0.35. It starts
parked at (-8,-8). Actual appearance is confirmed by Gazebo model world pose.
The obstacle remains until trial teardown. Fresh control must demonstrate
obstacle response before main execution. Restore original scans after two seconds.

Independent world geometry uses a conservative yaw-aware rectangular envelope
x [-0.20,0.08], y [-0.16,0.16] around the inspected SDF collision shapes. This
is explicitly an envelope, not the exact physical surface. Sampled zero clearance
means envelope overlap, not proof of contact. Record obstacle-specific Gazebo
contact events separately; calibrate that stream with a separate intentional
simulation overlap before navigation trials. Missing contact messages are not zero
collisions. No independent stop-on-proximity guard is used in this simulation.

Primary: actual payload origin vs advertised timestamp, delivery rate, source
invalidations, motion stop latency/distance and sampled envelope clearance/contact
events. Secondary: original task result and unchanged illustrative 0.30m arrival
requirement between result+1.0..1.5s. World pose is never fed into navigation TF.
Task bound: 60 simulated or 180 wall seconds. Startup: 75 wall seconds after worker
launch. Observe through at least fault onset+6s and result+1.8s; tail watchdog 30s.

Before main: revised budget of up to four contact-calibration process attempts and up to eight
excluded navigation verification trials, including retained development attempts;
the final four must cover all conditions with matching frozen source/config.
If this budget cannot establish instrumentation, retain evidence and revise the
protocol before any main run. Main: 12 trials, three blocks / Gazebo seeds
101/102/103; cyclic condition orders beginning fresh / missing / frozen.
AMCL/MPPI/scheduling remain uncontrolled. Fresh processes and a distinct ROS
domain per startup. No concurrent simulation workloads or task-outcome retries.

Pre-main revision: first calibration startup exposed unstable hashing of CDR
padding bytes; canonical field hashing replaced it. Second startup hit an upstream
launch race (Gazebo opened its world before xacro wrote it). The retained launch
copy now waits for xacro exit before starting Gazebo, equally in all conditions.
Neither attempt submitted a goal or performed the contact calibration. The initial
two-start calibration budget was revised to four before any navigation verification
or main outcomes; all startup attempts remain excluded and retained.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m unittest -v test_core
python3 run.py /tmp/rfl05-contact --calibration --domain 120
python3 run.py /tmp/rfl05-verification --verification --domain 125
python3 run.py /tmp/rfl05-main --verification-root /tmp/rfl05-verification --calibration-root /tmp/rfl05-contact --domain 150
python3 report.py /tmp/rfl05-main
```

Use new folders and idle domains. Main execution checks final verification
source/config/package and calibration model hashes. `--resume-infrastructure`
preserves every completed outcome and only permits retrying a startup before
READY/goal/terminal evidence, up to three startups per scheduled trial. A
startup failure stops the batch for inspection first. Do not alter source or
parameters mid-main or extend the sample based on outcomes.

Supported: Ubuntu/WSL, ROS 2 Jazzy with nav2_bringup, nav2_minimal_tb3_sim,
ros_gz_bridge and ros_gz_interfaces, Gazebo 8 vendor plugins; Python3/PyYAML,
NumPy, matplotlib and Pillow. Exact versions, original model/world/launch,
configuration, map and source hashes are captured. CDR scan payloads and
per-message provenance permit independent recomputation of all injected faults.

This diagnostic downstream-branch intervention is not a completely failed
physical laser, consumer-by-consumer causal isolation, general safety evidence,
Nav2 defect, Vektor integration or customer validation. Fresh-machine
reproduction remains unverified. Earlier experiments are preserved.
