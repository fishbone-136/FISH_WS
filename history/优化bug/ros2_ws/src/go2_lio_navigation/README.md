# Go2 HAP LIO navigation

This package brings up the following online mapping and navigation chain:

```text
HAP PointCloud2 (per-point timestamps) + HAP IMU
        -> FAST-LIO2 (odom_lio)
        -> Scan Context + GICP loop detection
        -> GTSAM iSAM2 pose graph (map -> odom_lio)
        -> 2.5-D elevation/traversability grids
        -> Nav2 SmacPlanner2D + DWB or MPPI
```

Both navigation costmaps and the local controller stay in `odom_lio`, while
loop corrections are applied only to `map -> odom_lio`. Map-frame goals and
terrain grids are transformed into that continuous frame. This also prevents
scan obstacle cells from remaining at their old map coordinates after a loop
correction. The saved environment map and RViz fixed frame remain `map`.
In LIO mode, `LatestGoalPose` transforms the clicked point into `odom_lio`
once at its click timestamp, then holds that same local position for the
action. Replans do not move it when the loop backend corrects the map. New
clicks create a new anchor. Stamp zero is used only on this spatial-reference
copy after conversion, so long actions do not expire with TF history. Original
input stays intact for goal-update detection. Non-LIO mode retains map goals.

`odom_lio -> base_link` is the planar navigation pose;
`odom_lio -> base_link_3d` retains the validated physical roll, pitch and height.
The physical sensor mount is `base_link_3d -> livox_frame`. Both projector lanes
filter in `base_link_3d` and level only their LaserScan XY into `base_link` using
the matching validated attitude. Genuine obstacle XYZ and ground planes stay
in `base_link_3d`; memory uses their full acquisition pose. A missing fresh
attitude cannot produce a new levelled navigation scan. This keeps scan cells,
terrain, remembered cells, footprint and path at the same world XY during gait.
RViz starts close to an overhead view to prevent measured ground height versus
the planar navigation plane creating a misleading perspective offset. Actual
2.5-D heights remain in the terrain displays and can still be inspected in 3-D.

An unexpected FAST-LIO exit now latches localization invalid, cancels the active
goal and pauses Nav2 while keeping the owning launch and RViz open. A red fault
message points to the LiDAR/USB connection. Restore the connection and restart
the launch to reinitialize localization; old goals are not resumed in a new
LIO coordinate session. Normal Ctrl+C still closes the whole launch.

## Dependencies and build

The current workspace uses the Livox driver installed in
`/home/fishbone/GO2/livox_ws` and GTSAM installed under `/usr/local`.

```bash
source /opt/ros/humble/setup.bash
source /home/fishbone/GO2/livox_ws/install/setup.bash
cd /home/fishbone/HAP_ws/ros2_ws
colcon build --symlink-install \
  --packages-up-to fast_lio go2_lio_navigation go2_navigation
source install/setup.bash
```

## Start

DWB is the lower-load default controller. Use the wrapper so CycloneDDS is
bound to the correct Go2 network interface:

```bash
cd /home/fishbone/HAP_ws
bash scripts/run_lio_navigation.sh
```

The DWB path now applies `GaitSafety` clearance/velocity limits before scoring,
keeps useful arc handovers across same-goal replans, and uses a supported
two-wall corridor to set the rectangular body's alignment tolerance. The
footprint critic retains its normal corner-clearance score outside corridors;
inside a supported corridor it suppresses duplicate soft inflation scores
while preserving collision rejection. Footprint, inflation radius and sensor
freshness limits remain unchanged. Diagnostic topics are
`/navigation/controller_diagnostics`, `/navigation/trajectory_diagnostics`,
and `/safety/motion_diagnostics`; the read-only trial recorder includes them.

Rebuild the changed packages with `bash scripts/build_navigation_fluency.sh`
from the project root. Run the synthetic controller/smoother/safety-chain
checks with `bash scripts/check_navigation_fluency.sh`. These tests do not
start a robot base or send robot motion requests. Real gait response, difficult
curved corridors and operation under actual LiDAR/LIO load still require
physical validation.

Use MPPI after the complete stack is stable at low speed:

```bash
bash scripts/run_lio_navigation.sh controller_mode:=mppi
```

For replay or when the HAP driver is already running, add
`start_hap_driver:=false`. To isolate the LIO front end before validating loop
closure, add `enable_loop_closure:=false`; this publishes a static identity
`map -> odom_lio` transform.

Keep the robot stationary through startup (usually 25-30 seconds including
driver discovery, eight seconds of IMU initialization, five seconds of initial
map collection, and the localization-health gate). Wait for a populated map,
healthy localization and active Nav2 before choosing a `Nav2 Goal`; elapsed
time alone does not mean the system is ready. The launch defaults Nav2 logs to `warn` to
avoid dynamic-grid INFO spam; use `nav2_log_level:=info` when debugging Nav2.

## Frames and mounting transform

The default physical transform is:

```text
base_link_3d -> livox_frame: xyz = [0.14, 0.00, 0.13] m, rpy = [0, 0, 0] rad
```

Override it at launch with `hap_x`, `hap_y`, `hap_z`, `hap_roll`, `hap_pitch`,
and `hap_yaw`. The translation and rotation must describe the sensor frame in
`base_link`, not the reverse transform. Measure and calibrate these values on
the actual bracket before autonomous operation.

FAST-LIO currently assumes the HAP LiDAR and its internal IMU are coincident
(`mapping.extrinsic_T/R` are identity). If the driver exposes different
physical LiDAR and IMU frames, replace those values with a calibrated
LiDAR-to-IMU extrinsic.

## Runtime interfaces

| Topic | Purpose |
| --- | --- |
| `/Odometry` | FAST-LIO sensor/IMU odometry |
| `/odom` | base-link odometry consumed by Nav2 |
| `/cloud_registered` | deskewed cloud in `odom_lio` |
| `/cloud_registered_body` | deskewed scan in the HAP body frame |
| `/scan` | deskewed obstacle scan for global geometry |
| `/scan_safety` | current raw HAP projection for local avoidance and emergency braking |
| `/obstacles/deskewed` | genuine filtered XYZ endpoints before scan gap fill or angular spread |
| `/safety/remembered_obstacles` | previously observed occupied cells in `odom_lio`; final motion gate also checks them |
| `/safety/navigation_local` | local terrain grid overlaid with retained obstacle cells |
| `/safety/navigation_global` | global terrain grid overlaid with retained obstacle cells |
| `/safety/obstacle_memory_status` | actual observation age, raw-frame pairing, processing time and memory faults |
| `/loop_closure/status` | accepted/rejected loop status |
| `/loop_closure/corrected_path` | GTSAM-optimized keyframe path |
| `/loop_closure/global_map` | optimized 3-D keyframe map |
| `/traversability/local` | rolling local terrain-cost grid in `odom_lio` |
| `/traversability/global` | recent terrain-cost grid in `odom_lio`; optional pose-graph mode uses `map` |
| `/traversability/terrain_surface` | live measured metric-height terrain with cost intensity |
| `/safety/cleared_obstacle_volumes` | confirmed free volumes shared with the environment reference |
| `/elevation/local` | encoded local relative-height grid |
| `/mapping/environment_map` | persistent reference terrain/occupancy grid |
| `/mapping/environment_cloud` | accumulated room/wall/furniture geometry |
| `/mapping/terrain_surface` | actual metric-height surface; intensity is terrain cost |
| `/mapping/save` | Trigger service exporting a unique session folder |

LIO navigation uses the overlaid safety grids by default. The raw `/scan_safety`
lane remains at 15 Hz. Occupied cells are written from genuine deskewed XYZ
endpoints, paired by acquisition timestamp with full 6-DoF `/Odometry_validated`
from the localization bridge. This stream is published only after the same
freshness, jump and startup checks that admit navigation odometry. Memory does
not wait for the equivalent dynamic TF to traverse a second callback queue.
Custom `full_pose_topic` inputs retain the independent timestamped TF check.
Latest-frame subscriptions and arrival-triggered processing avoid stale
observation queues. New occupied evidence publishes immediately; the 15 Hz
timer repeats its actual acquisition timestamp, and background clearing runs
separately from occupied writes. The final scan/memory gate also consumes only
the newest queued sensor message and transforms remembered points in a batch.
Artificial scan gap-fill/spread
points are never persisted. Each occupied cell retains measured world-height
bounds from its first observation; roll, pitch and height are included before
projecting into the XY grid. Writes do not wait for the clearing cloud worker.
The protective memory snapshot publishes
at 15 Hz with its actual scan timestamp. Expensive 3-D clearing runs in a single
spawned worker process at up to 6 Hz, isolating Python CPU work from the ROS
callback interpreter; stale results are discarded, and an old result
cannot erase or rewind a newer occupied hit. It retains obstacles
through unobserved sectors and missing returns, independently of terrain history
and Nav2 costmap clearing. Repeated matching 3-D rays passing through the
old obstacle's measured height volume clear static records. Raw returns still
supporting that volume veto clearing. Infinity in a ground-filtered 2-D scan
alone is not evidence that an obstacle has disappeared.

The `motion_evidence_v6` policy also tracks compact obstacle clusters in world
coordinates at the observation rate, up to 20 Hz. Four coherent positions spanning at least 0.18 s
and 20 cm confirm motion; robot rotation, shape changes and jitter cannot supply
that confirmation. For a confirmed moving object's previous positions, actual
farther background returns can prove clearance even when they hit only ground,
instead of crossing the former upper body's height. Three clear observations
spanning at least 0.35 s are required. Current occupied support or a nearer
non-ground occluder vetoes this clearing. Established static cells and thin
legs are not assigned a moving object's clearing policy. A strongly supported
current floor can separately correct old near-floor records whose entire
measured height lies within 6 cm of it. There is no time-only obstacle expiry.
Recently appearing geometry is separated from established static returns when
a person passes next to a box/wall and their XY components merge. Static cells
remain protected instead of being incorporated into the moving object's trail.
Track association bridges short observation gaps; a bounded motion corridor
includes fragmented tracked returns. Independent sparse landmarks and static
geometry confirmed before the moving track began remain protected. The bounded
raster and the per-frame voxel keys are reused within the tracking step.

Three-dimensional clearing accumulates at least eight genuine seen-through
observations inside a four-second confirmation window, spanning at least
0.5 seconds. A sparse frame without a matching ray preserves the partial
sequence inside that window; actual occupied support, a new occupied write,
or an observation gap longer than 0.6 seconds resets it. Expiring the window
only resets the evidence counter, never deletes the obstacle. The ROS
parameter `clearing_confirmation_window` controls this window. The original
two-dimensional model keeps its consecutive-frame semantics.

The clearing worker receives only in-range cells not occupied within the last
0.5 seconds. Other cells stay in the main memory. Completion is checked every
40 ms, with dispatch limited to 6 Hz and a single outstanding job. The
0.6-second result freshness limit and newer-hit merge protection remain active.
`/safety/obstacle_memory_status` reports `clearing_policy`, candidate counts,
completed jobs and removed cells for checking which implementation is running.
Motion diagnostics include `dynamic_tracks_confirmed` and
`dynamic_history_cells`. `/safety/save_obstacle_memory` exports a checkpoint of
occupied geometry and dynamic cell tags; `restore_state_file` accepts it only
in the exact same running FAST-LIO session and frame. The reload helper's
`--preserve-session-memory` mode uses this checkpoint before replacement.
Legacy nodes without this service require explicit `--reset-session-memory`.
For an operator-confirmed empty blind patch, the reload helper also supports
`--confirmed-right-clear --confirmed-region-reference FILE` together with a
preserved checkpoint. The reference binds the original pose, LIO session and
observed records: only old records in that confirmed region can be removed,
and any subsequent occupied observation survives. This is an explicit recovery
operation, not an automatic obstacle lifetime or whole-map reset.

The worker receives numeric cell arrays and returns only changed confirmation
counters/deletions. Main-process merges still reject updates to newer occupied
hits. Safety-grid publication targets 10 Hz and caches refinement of unchanged
terrain messages; aligned 8-to-4 cm refinement repeats cells exactly, including
unknowns and thin obstacles. The `memory_publish_rate`, `navigation_publish_rate`
and `clearing_rate` parameters can be changed online without losing memory.
New terrain grids have cell-aligned world origins and retain the actual cloud
acquisition stamp; an old scan does not become fresh just because a timer runs.

With independent obstacle memory enabled, `CurrentObstacleLayer` rebuilds only
the current scan contribution and includes the previous bounds when updating
the master costmap. This releases vacated cells and their inflation, without
keeping a second obstacle history that can outlive proven memory clearance.
Blind obstacles remain in memory. Standalone navigation without that memory
retains the stock persistent ObstacleLayer. There is no extra scan raytrace in
the current-only layer; repeated protection heartbeats reuse serialized clouds.

The memory is scoped to the running LIO session, pruned beyond 20 m, and is
not a saved-map obstacle database. Localization or memory freshness failures
close the final motion gate. Conservative retention may leave a temporarily
blocked region when there is no suitable 3-D view to prove it clear; an unseen
obstacle is still dependent on current sensing. `record_navigation_trial.py
--raw-clouds` can save sampled nearby raw XYZ, odometry, static transforms and
memory diagnostics for a user-issued trial without publishing motion goals.

Memory points are added at 4 cm local / 8 cm global resolution, rather than
expanding each remembered point to the terrain source's 10 cm cell. Refinement
preserves overlapping source obstacle costs and retains unknown cells when no
positive cost evidence overlaps them. It does not remove terrain obstacles or
change the robot footprint/inflation settings.

The default DWB velocity grid uses 11 forward, 1 lateral and 25 yaw samples
(275 nominal combinations, previously 375), keeping the yaw spacing and 1.7 s
collision horizon. Rear-path alignment tests the other initial turn direction
when the preferred side is blocked, with the same full-footprint sweep check.
If neither side is clear, the footprint-checked DWB arc remains the fallback.
The no-progress interval is 6 seconds instead of 10; required translation and
rotation progress thresholds remain unchanged. No unchecked reverse motion or
obstacle clearing is introduced by the recovery behavior.

## Persistent environment mapping and display

The default `config/lio_environment.rviz` shows the original environment map,
measured reference terrain, accumulated structure, live 2.5-D terrain,
robot heading, gold navigation route, corrected path and padded footprint.
The live surface uses actual metres for Z and green-to-red cost colours;
it is not a grayscale costmap presented as room geometry. The transparent
local Nav2 clearance overlay is enabled so reference-green floor is not
mistaken for a safe goal inside a remembered obstacle's inflated footprint.
Global Nav2 costs and raw scans remain optional displays.
The Navigation 2 panel, goal tool and orbit view are included. Rendering
targets 15 FPS; reference clouds publish at 1 Hz and live terrain at 8 Hz.
Only the selected DWB or MPPI controller is constructed at startup.
`config/lio_environment_full.rviz` carries the same restored display layout:

```bash
bash scripts/run_lio_navigation.sh \
  rviz_config:=/home/fishbone/HAP_ws/ros2_ws/src/go2_lio_navigation/config/lio_environment_full.rviz
```

The mapper still computes the occupancy/elevation reference map and export
geometry. It serializes structure/terrain point clouds only when they have
subscribers and reuses voxelized reference clouds while geometry is unchanged.
IMU initialization (8 seconds), stationary bootstrap map collection (5 seconds)
and localization health validation are unchanged. A faster RViz window is not
permission to send goals before localization and Nav2 are ready.

Reference terrain cells are 0.15 x 0.15 m, drawn as 0.14 m flat squares.
The live traversability grid uses 0.08 m cells; Nav2 uses 0.04 m local and
0.08 m global cells. Changing RViz's square size changes only its appearance.
Changing a grid's resolution changes geometry/detail, memory and processing:
halving the cell width gives about four times as many cells for the same area.
The stock Navigation 2 panel's `Localization: inactive` refers to AMCL, which
this FAST-LIO mapping launch does not start. Check `/lio/localization_valid`
instead; Nav2's `Navigation` status should be `active` before sending a goal.

`environment_mapper.py` pairs cached dense body scans with the timestamps in
the corrected keyframe path. History is rebuilt from retained body scans
when optimized poses change, so a loop correction does not keep both the old
and corrected copies. Proven clearance from obstacle memory removes matching
old above-floor points from these scans, so later pose correction cannot put
a departed moving object back. A newer keyframe may observe a real object in
the same place again. Floor points and unobserved volumes stay in the reference.
The coarse graph fallback is disabled by default: it lacks observation identity
and can repeatedly reinsert removed objects. `use_pose_graph_fallback:=true`
is an optional environment-mapper parameter for inspecting an old graph after
starting that node late. A fresh local scan provides a preview before
the robot has moved enough to create a new keyframe.

These reference layers retain history, unlike the 0.35-second recent-observation
window used by the live 2.5-D safety map. They deliberately do not feed old
dynamic obstacles back into Nav2. Bounds default to 400 keyframes, an 8 m
observation radius and at most 60 m grid extent, with a 3 Hz map update.
When subscribed, historical structure and reference terrain publish at 1 Hz;
the 2-D reference map updates on changed geometry, at up to 3 Hz. Recent live terrain uses at most three clouds,
reducing gait smear and duplicate ray-tracing work.
In the default recent-observation mode, the local grid is cropped from the
global live grid: classification, slope fitting and free-space raytracing run
once per observation. Slopes use a 24 cm neighbourhood plane instead of one
adjacent-cell difference; the independent 15 cm step and 22 cm vertical-extent
checks retain real discontinuities. Unknown cells are not filled by the fit.
Unknown terrain stays unknown. History integration pauses while localization
is invalid. Parameters are in `config/environment_mapping.yaml`.

This is online mapping and navigation: no prebuilt map is needed. The measured
heights and terrain costs form a 2.5-D representation, projected to Nav2's 2D
costmaps for SmacPlanner2D and DWB. Persistent reference mapping updates
at 3 Hz; live terrain and the global costmap update at 8 Hz
(125 ms nominal). Global/local costmap visualization target about 8/7.5 Hz.
Their publication ceiling is 10 Hz so costmap timer boundaries do not skip every
other publication at an equal configured update/publish rate. Local obstacle processing
and control run at 15 Hz; the smoother and motion gate both run at 50 Hz. The loop-closure global
cloud is keyframe-driven, not a fixed-frequency stream. Actual rates and latency
depend on sensor freshness, CPU scheduling, and growing-map size.

## Obstacle safety and runtime audit

The scan converter estimates a spatially supported near-field ground plane
instead of clipping at one fixed sensor height. Returns 10 cm or more above
that plane enter the obstacle scan. Lower returns do not become positive
2-D obstacle walls or free-ray evidence; the 2.5-D grid keeps step/drop geometry.
With `ground_pose_topic:=/Odometry_validated`, the converter tracks a stable
floor in continuous odometry and refines it against each current scan. This
separates body pitch/height changes from ground movement. A spatially supported
current floor can renew the model when a competing raised surface wins RANSAC;
an unobserved model cannot renew itself. A slightly delayed attitude may use
the measured plane only with a matching base-height. The raw safety lane still
works without this pose reference. Unsupported/expired ground estimation does not
clear unknown space. Angular sectors without observations remain NaN, not
infinite clearing rays. The converter library has a unique basename
`libgo2_pointcloud_to_laserscan.so`: the previous generic name could silently
load the apt-installed library instead of this workspace's implementation.
LIO mode projects the reliable FAST-LIO body cloud directly, with a depth-one
queue. It bypasses the legacy `/trans_cloud` copy/transfer hop, which could
drop large best-effort pointcloud frames. This deskewed lane feeds global
geometry. A separate depth-one projection of `/livox/lidar` produces
`/scan_safety` without waiting for FAST-LIO, so local costmaps, collision
monitoring and the final gate see a new moving obstacle sooner. Both lanes
use ground/self filtering, single-frame data and explicit unknown sectors.
Raw safety scans retain their actual acquisition timestamp; they are not
restamped or interpolated to manufacture freshness. A raw frame can have
motion distortion; deskewed scans still own the global geometry, and the
existing padded footprint/localization checks remain enabled.
Single-cell obstacles are retained:
a real thin chair leg must not be removed merely because it fits in one cell.

DWB uses the rectangular ObstacleFootprint critic and
`GaitTrajectoryGenerator` to reject nonzero translation below 0.22 m/s
before trajectory simulation. Humble's ordinary iterator accepts minimum
translation OR minimum yaw, which otherwise admits tiny walking arcs when
turning quickly. Genuine in-place turns and normal walking arcs remain;
all retained candidates still use the full trajectory footprint checks.
The behavior tree
replans at 2 Hz. The stable-rotation shim preserves its ramp/direction for a
replan to the same goal. Commands pass through the independent collision
monitor and observation freshness gate before reaching the base:

```text
controller -> smoother (/cmd_vel) -> collision monitor (/cmd_vel_safe)
           -> reactive scan gate (/cmd_vel_guarded) -> Go2 base
HAP -> raw projection (/scan_safety) -> local costmap / collision monitor / gate
HAP -> FAST-LIO -> deskewed projection (/scan) -> global costmap
```

The motion gate requires fresh forward observations (at most 0.35 seconds old),
a fresh published robot footprint (at most 0.5 seconds old), finite velocity
commands, and a command age below 0.2 seconds. This is necessary
because this Humble collision-monitor version ignores stale observation sources
rather than independently guaranteeing a stop. Existing localization/watchdog
guards remain enabled. These layers are not a substitute for a supervised
physical obstacle test, blind-sector awareness, or an emergency stop.

The driver defaults to `hap_publish_freq:=15.0`; FAST-LIO's scan-rate hint and
the converters' scan-period metadata follow this setting. This changes ROS
point-cloud batching, not the HAP's physical point sampling rate. Actual
frame rate must be measured under full load; setting a parameter to 15 does
not guarantee a sustained 15 Hz from every downstream node.

Large point clouds can overflow Linux's common 208 KiB UDP receive ceiling.
`scripts/env.sh` requests up to 8 MiB receive buffers for these CycloneDDS
participants while retaining the selected Go2 interface and robot discovery
peer. Install the prepared kernel upper-bound configuration once on the AGX:

```bash
sudo install -m 644 config/99-go2-lidar-buffers.conf /etc/sysctl.d/99-go2-lidar-buffers.conf
sudo sysctl -p /etc/sysctl.d/99-go2-lidar-buffers.conf
```

This increases only `rmem_max`/`wmem_max`; system default buffer sizes stay
unchanged. Restart navigation afterwards so existing DDS sockets request the
larger buffer. If the configuration cannot be installed, the stack still
accepts the OS default buffer, but a stable 15 Hz must not be assumed. Audit
without triggering optional 3D outputs:

```bash
source scripts/env.sh
/usr/bin/python3 scripts/check_navigation_runtime.py --seconds 30 --output /tmp/navigation_latency.json
```

The final gate also checks the hard-stop rectangle as soon as a scan arrives,
without waiting for the next upstream velocity command. A per-return
range-rate guard reserves 0.20 s response time. It matches nearby returns,
compensates fresh measured ego motion, and requires consistent approach
evidence across two intervals unless contact is imminent. A side-return
switch is not projected onto other frontal points, and lowering a recovery
command does not make static obstacles appear to approach faster. First-frame
geometric braking and the hard stop still run independently. This is not
person recognition or a guarantee of avoiding every moving person. A separate recovery ramp bounds acceleration after a safety
stop/slowdown independently on each axis. This avoids restarting the forward
gait whenever a small yaw command reverses. Since this can change curvature,
the gate rechecks the resulting braking arc and approaching-obstacle contact
before emission, then records the command actually permitted. New stops and
braking reductions apply immediately. The base receives depth-one velocity commands
so old ramp samples do not queue ahead of a stop.


The forward-observation gate retains the timestamp of the last actual forward
coverage across a sparse HAP frame. Empty/single-ray frames never refresh the
0.35-second observation deadline. During such a gap it can only continue the
previously safe command at equal/lower component magnitudes and unchanged
requested curvature; it cannot release a stop, reverse, or start a new axis.
An obstacle stop always resets this continuation budget to zero. New forward
observations are required before restarting. Scan reception, scan header age,
footprint and command deadlines remain independently enforced.

The Go2 EDU profile caps forward speed at 0.55 m/s and turn speed at
1.20 rad/s. The command bridge uses the same yaw ceiling. Fifty-Hz open-loop
smoothing applies acceleration limits of 0.70 m/s² and 1.60 rad/s², with
1.80 rad/s² yaw deceleration and small output deadbands (0.04 m/s, 0.05 rad/s)
so ramping does not repeatedly jump between StopMove and a large command.
Within 0.45 m of the goal the cap decreases continuously toward 0.24 m/s at
the position tolerance, instead of stepping abruptly to a fixed slow speed.
The outdoor profile uses 0.35 m/s and 0.70 rad/s while path alignment caps
at 0.75 rad/s. The stable goal
checker accepts position within 0.05 m and ignores final yaw in the default
position-only mode. Replanning does not change the anchored goal position.

The 0.76 x 0.40 m nominal body/stance rectangle now has 0.05 m padding on
all sides: the effective planning envelope is 0.86 x 0.50 m. This reserves
space for foot swing and pose uncertainty; it is not a measurement of the
actual gait or a guarantee of 5 cm real clearance. Verify the moving feet
and sensor extrinsics on the physical robot. Both costmaps use this envelope.
For the indoor profile, local/global inflation radii are both 0.28 m
with cost scaling factor 8. The protected half-width is 0.25 m,
leaving a nominal 0.03 m gradual preference band.
Local planning uses a 4 x 4 m rolling window centred on the robot.
The existing 4/8 cm grid cells quantize the visible boundary. Rough outdoor
profiles retain wider inflation fields and their former cost gradients.
Inflation radius is measured from an obstacle and is not a hard clearance bound.
The default RViz Map display now shows this actual navigation costmap instead
of the historical reference map, making blocked and high-cost regions visible.
Persistent reference mapping and export continue in the background.

The fixed hard-stop rectangle matches that padded planning envelope:
0.45 m forward/backward and 0.27 m to either side. It no longer demands a
separate 0.76 m-wide passage after the planner has accepted a narrower path.
Protection beyond the fixed rectangle uses a speed-dependent braking arc,
including scan age, 0.25 s reaction allowance and uniform deceleration at
the configured 0.90 m/s² / 1.50 rad/s² limits. At 0.45 m/s with 80 ms scan
age, the forward checked extent is about 0.71 m. These are command-model
assumptions, not a measured
physical braking guarantee. Humble's
STOP condition uses `count > max_points`, so `EmergencyStop.max_points: 0`
stops for even one valid obstacle return. APPROACH keeps `max_points: 1`:
using zero there would stop an empty scene due to its different current-pose
comparison. It predicts the live padded footprint for 1 second.

The downstream gate checks every observed point through that braking arc,
including combined forward/turn motion and rear-corner sweep. If needed it
selects a slower checked arc, stopping when no usable command is clear. The
approach guard also follows the commanded arc instead of treating every side
return as a frontal collision. Within 20 cm of the protected footprint,
close-clearance translation/yaw caps remain 0.22 m/s and 0.65 rad/s. Across
the next 15 cm the caps continuously join normal speed, avoiding the former
binary boundary. Initial safety scaling preserves the requested arc; recovery
may change it only with a new collision check. A clear away-turn can continue.
It does not demand an arbitrary 1.5-second constant-speed turn past a short
manoeuvre. `/safety/turn_clearance_valid` reports the braking-envelope check.
Dimensions automatically grow to match larger rectangular published
footprints, with conservative handling of nonrectangular shapes. This assumes
the configured footprint is centered at base_link.

The progress allowance is 6 seconds for a cautious manoeuvre. The independent
base execution watchdog still rejects sustained non-executed commands.

The rotation shim checks its requested maximum yaw rate during ramp-up, instead
of testing only the tiny first velocity sample. Its lookahead is 1.8 seconds
and orientation samples are at most 0.02 rad apart. If a full-rate sweep is
blocked, it first tries slower complete sweeps down to the configured gait
minimum, retaining turn direction; it does not immediately abandon alignment
for an unstable forward arc. The common checked sweep is not computed twice.
Pure-turn caution uses clearance to the padded body's entire swept circle;
translation and final collision/braking checks retain their own constraints.
These protections cover
observed/mapped obstacles; HAP's side/rear blind sectors and localization drift
still limit real-world safety. Narrow spaces may now require stopping or be
rejected rather than fitting the former smaller envelope.

Rebuild/install and use the supported LIO wrapper to load all safety stages:

```bash
cd /home/fishbone/HAP_ws
bash scripts/run_lio_navigation.sh
```

Unit and isolated pipeline checks (the latter never drives the physical robot):

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  ros2_ws/src/go2_lio_navigation/test/test_turn_clearance.py -v
source scripts/env.sh
ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/go2_obstacle_pipeline_logs \
  /usr/bin/python3 ros2_ws/src/go2_navigation/test/check_obstacle_pipeline.py
```

Blocked-path recovery waits briefly and replans without clearing real obstacles
from the costmaps. The project-root `navigation_tuning_comparison_20261023.json`
contains the earlier tuning attempt and isolated validation results. The later
`navigation_drift_fix_20261023.json` records the subsequent drift investigation
and active parameters.
The later `navigation_close_obstacle_display_20261023.json` records the
near-person/display adjustment and measured render performance. Humble's Wait
action stores integer seconds, so recovery now requests one second explicitly;
the old fractional 0.25-second setting became zero and aborted too quickly.
Reference-map voxel grouping uses a collision-free packed integer key when
the extent fits int64, retaining the same first points and output order.
Point transforms also avoid a temporary float64 copy of complete reference
clouds. Tracking state and calibration precision remain unchanged.

FAST-LIO now discards stale startup packets and requires one second of
continuous fresh input before its eight-second IMU initialization. Keep the
robot still through IMU initialization, initial-map collection and validation.
During tracking, stale scans or IMU gaps over 50 ms stop the frontend and require
a restart. A scan update needs at least 50 plane matches, a 5% match ratio and
mean residual no larger than 0.15 m. Rejected updates do not publish clouds or
odometry and do not extend the map; three consecutive weak matches stop the
frontend. These checks detect several failure conditions, but a good match
count alone does not establish full geometric observability or eliminate drift.
The base bridge separately rejects odometry timestamps over 0.3 seconds old.

Read-only measurement, without submitting any motion goal:

```bash
cd /home/fishbone/HAP_ws
source scripts/env.sh
python3 scripts/check_navigation_runtime.py --seconds 12
```

The report separates configured update frequencies, observed publication rates,
header-age latency, lifecycle/health flags, whole-machine CPU usage and per-process
single-core CPU usage. `/plan` and controller-command rates cannot be measured
from an idle robot; a zero-message count is not a planner failure. Full costmap
publication also differs from the internal update rate and incremental updates.

Save the current session:

```bash
cd /home/fishbone/HAP_ws
bash scripts/save_environment_map.sh
```

The response gives a new `maps/environment_YYYYMMDD_HHMMSS_...` directory:
`environment.pcd` stores geometry, `elevation.npz` stores floating-point heights
in metres plus terrain costs and grid origin, and `map.pgm`/`map.yaml` provide
a reference occupancy image. `metadata.json` records the reference frame and
whether the grid was bounded. Saving is a snapshot, not automatic saved-map
relocalization; restarting the full LIO stack begins a new coordinate frame.

Useful checks before sending a navigation goal:

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 topic hz /Odometry
ros2 topic hz /cloud_registered_body
ros2 run tf2_ros tf2_echo odom_lio base_link
ros2 run tf2_ros tf2_echo map odom_lio
ros2 topic echo /loop_closure/status
```

First validate stationary drift, then walk a closed loop while recording a
rosbag, and finally command short low-speed goals. A loop-closure event should
change `map -> odom_lio` and the corrected global path, but it must not jump
`odom_lio -> base_link`.

## Tuning entry points

Loop verification registers recent body scans against a historical submap in
the candidate keyframe's coordinates. Neighboring historical poses come from
the optimized graph; current source scans cannot appear in their own target.
LIO's relative pose is the first GICP seed. Scan Context yaw is an alternative
relative heading seed, rather than a second world-frame rotation.

HAP descriptors require at least six shared sectors and 50% angular coverage.
Up to three descriptor candidates receive geometric verification. A loop needs
at least 100 correspondences within 0.30 m, 45% source coverage, and no more
than 0.20 m inlier RMS error (`gicp_fitness_threshold: 0.04` is in square metres).
Independent matched surface normals must constrain all translation directions;
a floor-only or parallel-wall match is rejected even if its distance is low.
Vertical/tilt correction limits are 0.35 m and five degrees. These gates may
reject loops in feature-poor corridors; rejection does not establish that LIO
is drift-free there.

Verified loop factors use Huber weighting, with five extra iSAM2 updates, so
large valid loop residuals still influence the graph. Three nearby source
keyframes improve the HAP's limited field of view. Candidates must be at least
10 seconds and 2 m of accumulated travel apart; stationary rotations alone
cannot repeatedly close to themselves. Registration and TF publication use
separate callback groups on two executor threads, keeping `map -> odom_lio`
available while registration runs. Odometry/cloud synchronization is limited
to 5 ms because both FAST-LIO messages carry the same scan-end timestamp.

Regression checks after building with `-DBUILD_TESTING=ON`:

```bash
cd /home/fishbone/HAP_ws/ros2_ws/build/go2_lio_navigation
ctest --output-on-failure
cd /home/fishbone/HAP_ws
source scripts/env.sh
ROS_DOMAIN_ID=82 ROS_LOCALHOST_ONLY=1 CYCLONEDDS_URI='' \
  /usr/bin/python3 ros2_ws/src/go2_lio_navigation/test/check_loop_closure_integration.py
```

The isolated check publishes synthetic data in domain 82 and does not send
motion commands. Optional `GO2_LOOP_REAL_CLOUD=/path/to/xyz.txt` enables the
captured-cloud regression (ASCII XYZ rows) with an injected known pose error;
this is not a real traversed-loop accuracy measurement. Rebuild and restart
navigation to apply C++/parameter changes. Save the current environment first:
the existing launch has no pose-graph checkpoint/restore and starts a new map.

The local controller uses `go2_navigation::StableRotationShim`. It transforms
path points into the local costmap frame using fresh latest TF rather than
requesting a sensor transform at wall-clock time. A rearward goal holds its
initial turning direction across the +/-pi boundary and accelerates up to
1.20 rad/s; the profile slows near path alignment. Final-heading alignment
is disabled in the default position-only mode. Both directions are checked against the robot footprint.
Transforms and robot poses older than 0.5 seconds stop the controller for a
retry instead of silently abandoning heading alignment.

The base bridge's legacy `min_pure_rotation_command` boost defaults to zero.
Keep it disabled so the controller's collision checks and velocity smoother
describe the yaw actually sent to Go2. Linear speed remains limited to
0.55 m/s. After rebuilding `go2_navigation` and `go2_core`, restart the launch
before evaluating these changes. The new heading messages are INFO logs;
start with `nav2_log_level:=info` if those messages are needed.

- `config/fast_lio_hap.yaml`: IMU noise, filters, LiDAR/IMU extrinsics.
- `config/loop_closure.yaml`: keyframe spacing, descriptor threshold, GICP
  fitness and correction limits, factor noise.
- `config/traversability.yaml`: slope, step, roughness and obstacle thresholds.
- `go2_navigation/config/nav2_params.yaml`: Smac, DWB and MPPI behavior.

The provided values are conservative startup values, not a substitute for
IMU Allan-variance measurement, extrinsic calibration, and site-specific
terrain validation.


## Position-only arrival and runtime ownership

Default arrival is within **0.05 m of the original clicked position**, using
the current localization estimate; final yaw is ignored. This is a software
criterion, not a guarantee of 5 cm world-referenced localization accuracy.
`StableGoalChecker.check_yaw=false` bypasses heading and capture hysteresis;
`rotate_to_goal_heading=false` prevents the controller from aligning the
arrow on arrival. The final 0.30 m uses a footprint-checked position approach
at usable gait speed rather than DWB samples below its output deadband.
If that arc is obstructed, the regular checked obstacle-detour controller
remains available. Initial in-place alignment can similarly fall back to a
full-footprint-checked tracking arc when rotation alone is obstructed.

The position controller does not independently emit zero just because its
pose sample briefly enters the tolerance circle. Humble ControllerServer
performs the authoritative strict goal check using a subsequent pose and
publishes the terminal zero. Competing stop decisions at that boundary can
otherwise stop and restart the gait while the action is still active. Final
steering accepts a forward ray through the inner 80% of the position circle,
instead of continuously turning toward its exact centre. This changes the
collision-checked approach arc, not the goal checker's 0.05 m criterion.
Final steering now matches curvature to remaining distance. If the permitted
turn radius cannot enter the accepted disk, it tries a footprint-checked
alignment or the normal DWB detour rather than driving past the target with
a fixed yaw gain. The complete final arc is checked at 25 ms intervals.

LIO selects `ExactGoalSmac2D` with zero goal substitution tolerance. Humble's
Smac2D normally ends at a cell centre, or returns the start pose for a same-cell
goal. The wrapper preserves its safe route and appends the original requested
endpoint only within the same nonlethal checked goal cell. It rejects a route
ending in a different cell. A target inside an obstacle must be moved by the
user; navigation cannot silently declare success at a nearby substitute.

One Nav2 server and one final scan gate are allowed per ROS domain. Process
locks and legacy-process detection reject duplicates before conflicting action
servers or velocity publishers are created. `scripts/reload_lio_navigation.py`
can perform an identified idle-session reload without resetting mapping. Its
detached supervisor follows the original main process identity, and each
child also receives a parent-death signal if the supervisor crashes. Tested
normal-owner exit and supervisor-crash paths stop their managed children.
No navigation goal is resumed automatically.

`/safety/motion_diagnostics` publishes scan/footprint freshness, command age,
hard-zone state, braking/approach scaling, and final velocity at 2 Hz. The
runtime audit includes it and counts publishers, so a blocked gate can be
distinguished from duplicate command writers. A green RViz path alone is not
an executing goal; check the Navigation 2 feedback and lifecycle states.


The final recovery check uses the physical braking envelope instead of applying
the soft approach fraction a second time to each 50 Hz ramp sample. Static
returns retain the decelerating swept arc; confirmed moving returns also cover
the complete response/stopping interval. Each candidate ratio is checked
itself. This prevents multiplicative low-speed stagnation while keeping
imminent moving contacts and new safety stops immediate. Short sparse HAP
frames cannot create a new axis or reverse one, release a previous stop, or
refresh the 0.35 s observation deadline. Suppressing only the unsupported axis
avoids an unnecessary full StopMove when a small yaw update crosses zero.

## Standing after navigation and surface approach estimates

The LIO launch starts `idle_stand_guard.py` alongside the base bridge. After an
observed navigation action ends, it waits for 0.8 seconds of zero final velocity,
fresh stationary sport feedback and no Move RPC. It sends one acknowledged
StopMove (1003), then BalanceStand (1002) only after the stop succeeds. This
explicitly ends the walking state which can otherwise persist at zero speed.
An active action paused by obstacle avoidance does not trigger standing.
New goals, resumed motion and interactive control interrupt the transition;
failed or missing acknowledgements are reported without repeated pose commands.
Status is available on `/go2/idle_stand_status`. Startup alone sends no standing
command, and this node never publishes Move or a navigation goal.

Approach prediction uses ego-compensated local surface-normal displacement so
HAP samples sliding along a static diagonal face do not imply an approaching
object. Consistent isolated thin returns retain their own rate estimate; new
nearby hazards still receive immediate geometric braking. Ambiguous corners
do not become supported surface tracks. NumPy correspondence and per-frame
surface caching bound processing cost. The hard stop rectangle, swept-footprint
checks and sensor-age limits remain active independently. This estimator is
not object identification or a guarantee of physical dynamic-obstacle avoidance.

After building, the standing guard can be tested against a mock sport service
without contacting the robot:

```bash
source scripts/env.sh
ROS_DOMAIN_ID=84 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 \
  ros2_ws/src/go2_lio_navigation/test/check_idle_stand_ros.py
```

## Fault recovery and repeatable offline checks

Once localization has become healthy, its pose continuity checks also apply
during freshness recovery. A stale/future packet cannot disable the jump guard
for the next accepted pose. The base stops both navigation and interactive Move
requests when sport feedback disappears; fresh feedback and an idle command
are required before clearing that execution fault.

The localization exit guard retries unavailable, rejected, failed and unanswered
cancel/pause requests with bounded frequency. It never reopens localization.
Scan levelling requires an explicit physical target frame, planar scan frame
and attitude topic. Merely renaming a frame is rejected. Levelling works with
ground filtering independently enabled or disabled.

The obstacle clearing worker delegates Ctrl-C to its ROS parent. If the worker
crashes, the memory node retains its geometry, publishes unavailable safety
evidence and a fault, and remains available for diagnosis. This requires a
memory-node reload to recover; no stale memory is silently considered healthy.

After building the packages, run the offline suite from the workspace root:

```bash
bash scripts/check_navigation_regressions.sh
```

The script sources the installed workspace, disables user Python packages,
forces localhost Cyclone DDS peers and uses isolated domains 83, 85–91. Its
navigation goals and fake sport RPCs never enter the physical robot's domain.
It covers worker termination, feedback loss, pose discontinuities, frame
alignment, obstacle clearing, planner/controller behavior and rear goals.
It saves each check's output under `reports/navigation_regressions_*` and also
checks native child exits, because launch can return zero after a child crashes.
The installed Humble composed Nav2 container can still exit with SIGSEGV during
SIGINT teardown; the suite reports failure when it sees that error even if
all functional assertions passed. This remains an unresolved shutdown defect.
