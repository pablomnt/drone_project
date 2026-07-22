# drone_project

Software for an autonomous vision-based quadcopter. The drone runs PX4 firmware, uses a RealSense
D435i + OKVIS2 for visual-inertial state estimation and RTAB-Map for SLAM/mapping, plans with RRT*
+ minimum-snap trajectories, and tracks them with a custom cascaded-PID / differential-flatness
controller that outputs attitude + collective thrust to PX4 over uXRCE-DDS.

## Architecture

The repo is split into two top-level domains so the autonomy logic never depends on the middleware
it happens to run under. The long-term goal is being able to drop ROS (and eventually PX4) without
rewriting any guidance or control code.

- **`core/` — the ROS-free autonomy core (`drone_core`).** Pure C++17, plain CMake, builds and
  unit-tests with zero ROS installed. Depends only on ROS-free libraries: Eigen, octomap (the
  library, not `octomap_msgs`), OMPL, NLopt, OSQP and DecompUtil. All guidance and control logic lives here. It
  carries a `COLCON_IGNORE` so colcon never tries to build it.
- **`ros2/` — thin ROS wrappers + vendored third-party packages.** The colcon/ament side. The one
  first-party node (`autonomy_node`) does only message↔core-type translation, frame conversions and
  PX4 I/O; the heavy third-party packages (`okvis`, `px4_msgs`) live under `ros2/third_party/` as
  in-tree copies of their upstream repos.

```
src/
├── core/                       # ROS-FREE autonomy core (plain CMake, COLCON_IGNORE)
│   ├── common/                 # types.hpp, frames.{hpp,cpp}, logging.hpp
│   ├── control/                # position_control, flatness_mapper, trajectory_tracker
│   ├── planning/               # geometric_planner, min_snap_trajectory
│   └── autonomy/               # autonomy_core (the orchestrator)
└── ros2/                       # everything colcon/ament
    ├── autonomy_node/          # the one first-party ROS node + launch files
    ├── drone_interfaces/       # custom msgs (ControllerDebug)
    ├── third_party/            # okvis2 (pkg name 'okvis'), px4_msgs (vendored copies)
    ├── sim/pc_publisher/       # sim-only static .pcd publisher
    └── tools/system_monitor_pkg/   # python CPU/RAM telemetry
```

## The autonomy core (`core/`)

`AutonomyCore` (`core/autonomy/`) is the orchestrator and the object a host (the ROS node, a sim
harness, or a test) drives. It is middleware-free and runs three internal cadences: RRT* geometric
replanning (~0.2 Hz) and minimum-snap trajectory generation (~1 Hz) on a background worker thread
(receding-horizon — trajgen re-anchors to the current state), and trajectory tracking + flatness
mapping at 50 Hz on whatever thread calls `stepControl()`. The fast control path never blocks on
planning; finished trajectories are handed over by an atomic swap. A blocked committed path replans
unconditionally; an *improve* replan is adopted only past a hysteresis margin, and it's scored against
the committed path's **remaining** cost from the drone's current position (not its full original
cost), so the drone doesn't abandon its route for a fresh one just because it has moved closer to the
goal.

### `common/`

Shared plain types (`State`, `Reference`, `Command`, `Trajectory`, `Goal`, `MapHandle`) and
`frames`, which holds *all* ENU↔NED/FRD and OKVIS-yaw conversions — extracted out of the old node so
they are finally unit-testable. `logging.hpp` provides `DRONE_LOG_*` macros that replace ROS logging
inside the core.

### `planning/`

- `GeometricPlanner` — a **runtime-selectable** OMPL planner over an SE(3) octree (X/Y ±15 m,
  Z −1.5–2.5 m). `PlannerType` picks among `RRTstar`, `BITstar`, `ABITstar`, `AITstar`, `EITstar`
  (the BIT* lineage is heuristic/informed and concentrates the search on the start→goal corridor
  instead of sampling the whole box, which plain RRT* does); a small `makePlanner()` factory builds
  and configures the chosen one. **Every per-planner tunable lives in `PlannerConfig` in
  `geometric_planner.hpp`** — the single place to tune them — and only the selection
  (`PLANNER_TYPE`) is a ROS param. The clearance objective gives the informed/heuristic planners both
  an edge heuristic (`motionCostHeuristic`) and a state→goal cost-to-go (`goalRegionCostToGo`) — both
  the Euclidean distance, admissible since the integrand is ≥1; without the cost-to-go one OMPL warns
  informed sampling has "little to no effect". Collision checking is **EDT-based**: a state is free
  when its clearance (3D Euclidean distance to
  the nearest obstacle, from a `DynamicEDTOctomap` passed in as the clearance function) exceeds
  `kCollisionMargin` (0.5 m) — one O(1) lookup that also enforces *vertical* clearance. (A
  horizontal-only octree box scan remains as a fallback for standalone use when no field is set.)
  Inside a sphere around the start (`kStartEscapeRadius`, 0.5 m) the required clearance drops to
  `kStartMargin` (0 m) so a parked or just-lifting drone — sitting within the normal margin of the
  mapped floor — can still root the search and take off, without ever passing *through* an obstacle.
  Approximate solutions that stop more than `kGoalFlexibility` (0.3 m) short of the goal are
  rejected (treated as no path) rather than committed to. The same field also drives a clearance-aware
  cost (`setClearance`). These margins are `static constexpr` in the class. The raw planner path is
  post-processed by a **cost-aware shortcut** in clearance mode (`shortcutClearanceAware`): it removes
  zig-zag waypoints but only when the straight bypass stays collision-free *and* does not raise the
  clearance-aware cost, so the result is straighter without hugging the obstacles the cost routed
  around (plain length-only `simplifyMax` is used only when no clearance field is set).
- `MinSnapTrajectory` / `MinSnapTimeOptimizer` — KKT minimum-snap solve plus an NLopt BOBYQA outer
  loop for per-segment time allocation ("mode A" of ETH Zurich's `mav_trajectory_generation`). The
  optimizer keeps the actual per-segment polynomial coefficients + segment times (rather than
  re-sampling to points) so the flatness mapper can differentiate it analytically. On its own it
  ignores obstacles *between* waypoints — the smooth polynomial can bow through a wall the waypoints
  cleared — which is what the corridor pipeline below exists to fix.
- `corridor.{hpp,cpp}` / `corridor_trajectory.{hpp,cpp}` — **corridor-constrained minimum-snap**
  (gated by `USE_CORRIDOR_QP`), a provably collision-free replacement for the plain solve above. It
  runs over a **dual view** of the map: the geometric search uses the raw **optimistic** map, where
  unknown space reads as free, so the informed planners accept a goal beyond the mapped frontier
  instead of refusing it outright; all safety comes from the **conservative**, frontier-stamped view,
  where unknown space reads as occupied. `truncatePath` walks the planned path against that
  conservative field and cuts it where clearance would fall below `FRONTIER_MARGIN`, yielding a
  committed prefix that never reaches into unexplored space and whose endpoint ratchets forward as
  the map grows. The required clearance ramps from zero at the drone up to the full margin over
  `ESCAPE_RAMP_DIST`, which serves the same purpose as the planner's start-escape sphere without its
  dead band; the ramp *length* is decoupled from the margin, because ramping over the margin itself
  makes the requirement climb at 1 m/m and, on a thinly mapped scene, the rising requirement meets
  the shrinking clearance within centimetres. `buildCorridor` then resamples
  that prefix and grows one maximal free **convex polyhedron** per segment using DecompUtil's
  ellipsoid inflation (Liu et al., RA-L 2017 — the safe-flight-corridor decomposition FASTER uses):
  an ellipsoid spanning the segment inflates and cuts a half-space at each obstacle that binds, so
  the region follows the path. Axis-aligned boxes were tried first and fail structurally — a box
  seeded on a diagonal segment's bounding box is mostly volume the path never visits, so it is
  rejected by geometry the drone would never approach. Faces are then pulled in by the margin plus
  the voxel half-diagonal (obstacle points are voxel *centres*), and each region is checked for a
  non-empty overlap with its neighbour, since C0 continuity pins the junction into that intersection.
  The **first** region is relaxed, because a convex region has no interior gradient: one plane holds
  over the whole region, so a uniform margin next to the drone would refuse every prefix truncation
  deliberately committed for a vehicle parked near a wall. The relaxation is bounded in extent (the
  first segment is split at `ESCAPE_RAMP_DIST`; every later region keeps the full margin) and in
  magnitude (the region is shrunk by the largest amount that still contains the drone, so it relaxes
  no more than the geometry forces and heals itself as the map fills in), with a floor at half a
  voxel diagonal — below that the region would contain points inside an occupied cell rather than
  merely close to it, which is a hard failure instead. `CorridorTrajectoryOptimizer` solves the trajectory as a **QP** (OSQP): degree-7
  segments in the monomial basis, reusing the same snap cost matrix, with C⁰–C⁴ continuity and
  rest-to-rest ends as equalities, and the **Bézier** control points of position confined to the
  regions and of velocity/acceleration/jerk bounded per axis by `VMAX`/`AMAX`/`JMAX` as
  inequalities. A polyhedron face mixes x, y and z, so unlike an axis-aligned box it cannot be split
  into three per-axis problems — the QP is solved once over all three axes jointly.
  Because a Bézier curve lies inside the convex hull of its control points, bounding those points
  bounds the whole curve — the guarantee holds everywhere, not just at sampled instants. Interior
  waypoints are deliberately *not* pinned, so the trajectory is free to cut corners anywhere inside
  its corridor. An outer BOBYQA loop searches the segment times, scoring an infeasible QP as a large
  penalty so it is pushed back toward feasibility. Everything is written against a **clearance
  oracle** (`CorridorClearanceFn`, the same shape as the planner's `ClearanceFn`) rather than against
  octomap directly, so region growth and truncation are unit-tested with analytic data. Any stage
  failing stages **nothing** — no trajectory is handed to the tracker, which rides out what it has
  and then hovers — and logs which stage failed and why. There is deliberately no min-snap fallback
  here: min-snap ignores obstacles, which makes it least defensible in precisely the situation that
  would produce it, namely the corridor stage reporting that it cannot certify a safe trajectory.

### `control/` — flight-critical, handle with care

`position_control.{hpp,cpp}` holds flight-tested logic for a real vehicle. The PID gains,
hover-thrust estimator constants and open-loop takeoff ramp have been tuned and proven stable in
flight; do not touch the gains or control math unless a task specifically requires it.

- **Open-loop takeoff override**: on a ground `reset()` it primes a takeoff; when a setpoint above
  0.5 m arrives it ramps thrust open-loop (flat attitude) until liftoff, then hands to the PID,
  because closed-loop control near the ground with noisy VIO causes skidding.
- **Online hover-thrust estimation**: back-calculates hover thrust from the filtered command and
  measured vertical acceleration, de-weighting the estimate at high vertical speed; overridable via
  `MPC_HOVER_THRUST`.
- **Differential-flatness feed-forward** (added on top, default OFF): `setReference()` accepts
  `vel_ff`/`acc_ff`; with feed-forward disabled the controller is byte-identical to the baseline (a
  unit test asserts this), and it is suppressed during the takeoff ramp. The accel→attitude/thrust
  map *is* the flatness output stage and already existed — which is why trajectory tracking did not
  require a new controller.

`flatness_mapper` samples the trajectory for pos/vel/acc and derives yaw from the velocity heading
(so the forward camera leads the motion), holding the last yaw only when slow *and* the heading is
spinning. `trajectory_tracker` composes the mapper + controller and runs the watchdog state machine:
`kDirect` (explicit setpoint — takeoff/manual hover), `kTracking` (follow a fresh planner
trajectory), and `kHoverHold` (failsafe — latch the current position when guidance goes stale). The
stale fallback hovers; it does not land, because PX4 rejects offboard land commands.

## The ROS side (`ros2/`)

### `autonomy_node/`

The single thin wrapper. Per 50 Hz tick it publishes the offboard heartbeat, checks arm/offboard
(resetting the core on disarm or on leaving offboard, re-priming takeoff), assembles an ENU `State`
from the active estimator (VIO in normal mode; PX4 odometry in `USE_SIM_MODE`), feeds the core, reads
back the `Command`, applies the yaw-drift correction toward PX4's yaw and the ENU→NED/FRD conversion
via `core/common/frames`, and publishes the attitude setpoint plus `ControllerDebug` telemetry. The
state is fed to the core **whenever the position source is fresh, independent of arm/offboard**, so the
planner roots at the drone's live position even on the disarmed bench (control output stays gated on
arm+offboard). Goals
arrive on `/planner/goal`; a `POS_SP` parameter provides the default takeoff/hover setpoint. All PX4
message types and frame conversions are confined to this file. It also exposes an **opt-in planner
debug visualisation** — `/planner/search_tree` (the RRT* tree as a MarkerArray) and
`/planner/clearance_field` (the EDT as an intensity-coloured PointCloud2) — behind the single
`DEBUG_PLANNER_VIZ` parameter (default off, zero-cost when off) for tuning planning by eye in
Foxglove. The same switch exposes `/planner/corridor`, which draws the corridor pipeline's
intermediate products — the free polyhedra as translucent face outlines, the truncated committed prefix as a
white line, and an orange sphere at the truncation endpoint, the intermediate goal in known-safe
space. Where that white line stops short of the green geometric path is exactly where truncation
refused to commit into the unknown. Any future debug/instrumentation should follow the same rule: one
default-off switch, nothing computed or published when it is off, and never in the flight-critical
path. When the planner *cannot* run it now says so — the worker logs the missing precondition (no
goal, or no map) rather than going silent, and the node reports the resolved map topic and its
publisher count until the first octomap arrives, which separates "nothing is publishing" from "we are
not receiving". Launch files live in `launch/`:
`autonomy_vision_launch.py` (real flight — full camera/VIO/mapping stack + Foxglove + the
`body→camera_link` static TF), `autonomy_launch.py` (xterm-per-process variant) and
`autonomy_sim_launch.py` (SITL: `USE_SIM_MODE:=true`, `MicroXRCEAgent udp4`).

**Sensor-health watchdog.** The node guards the three estimator streams it depends on — PX4 odometry,
IMU (`sensor_combined`) and VIO — by timestamp, deeming each *healthy* only while its last sample is
within `SENSOR_TIMEOUT` (0.5 s). Two gates use this. **Before takeoff** the controller refuses to
*engage* until every required stream has been continuously healthy for `SENSOR_WARMUP` (5 s), so it
never takes off on a stream that arrived once and then died (the old check only asked whether a stream
had *ever* been received). **In flight**, if any required stream goes stale it commands `NAV_LAND`
(PX4 AUTO.LAND) and latches — re-commanding every tick until PX4 confirms the mode, since a single
`VehicleCommand` can be dropped — and re-arms only after touchdown/disarm. That is a real landing,
distinct from the core's `kHoverHold` (which only hovers): `NAV_LAND` is a mode switch that works from
offboard, whereas the *core's* offboard-attitude stream has no land primitive. Both `SENSOR_TIMEOUT`
and `SENSOR_WARMUP` are live-reconfigurable params.

### `drone_interfaces/`

Custom message definitions (`ControllerDebug.msg`). Message generation has to stay ROS-side.

### `third_party/`

Vendored in-tree copies of upstream repos (kept as copies, not git submodules): `okvis2` (package
name `okvis`, a large VIO library with its own `external/` deps) and `px4_msgs` (matching the PX4
v1.17 topic set). Don't hand-edit these; they mirror upstream — with **one deliberate local patch**:
`okvis_ros2/src/Publisher.cpp` (`setBodyTransform`) no longer broadcasts the `body→camera_link` static
TF, because it collided with the launch's static publisher and randomly corrupted the voxel map (see
*Frames & TF* below). If you ever re-vendor okvis2 from upstream, re-apply that change.

### `sim/`, `tools/`

`pc_publisher` is a sim-only static `.pcd` publisher; `system_monitor_pkg/cpu_monitor.py` publishes
`/telemetry/cpu_usage_total` and `/telemetry/ram_usage_total` at 2 Hz.

## Runtime pipeline

```
RealSense → OKVIS2 (VIO: /okvis/okvis_odometry) → RTAB-Map (ray-traced 3D occupancy
         octomap: /rtabmap/octomap_binary, + /rtabmap/octomap_global_frontier_space)
                                   │
                      ┌────────────┴──────────── autonomy_node (thin ROS wrapper) ──────────────┐
                      │  subscribes: octomap, okvis odom, px4 odom, sensor_combined,             │
                      │              vehicle_status, /planner/goal, /joy                          │
                      │  owns a drone_core::autonomy::AutonomyCore, drives it at 50 Hz,           │
                      │  converts ENU↔NED/FRD at the boundary, publishes attitude+thrust to PX4   │
                      └─────────────────────────────────────────────────────────────────────────┘
                                   │
                                  PX4 (via MicroXRCEAgent, serial /dev/ttyUSB0 or UDP for SITL)
```

## Frames & TF (OKVIS ↔ RTAB-Map)

Two different things get placed into the world, and they depend on *different* transforms — which is
why a broken camera extrinsic corrupted the voxel map while the odometry/trajectory still looked
perfectly correct. A depth point measured by the RealSense reaches the world frame through this chain:

```
world ──(OKVIS VIO, dynamic)──▶ body ──(static: body→camera_link)──▶ camera_link ──(RealSense, REP-103)──▶ camera_*_optical_frame
```

- OKVIS estimates the pose of `body` in `world` and publishes it as `/okvis/okvis_odometry` — this is
  the trajectory RTAB-Map tracks against.
- `body→camera_link` is the fixed camera-mount transform — a **pure rotation, zero translation**
  (`pitch -1.5708, roll 1.5708`), published by the `static_transform_publisher` in
  `autonomy_vision_launch.py`.
- The RealSense driver supplies `camera_link → *_optical_frame` (the standard REP-103 optical rotation).
- RTAB-Map (base frame `camera_link`) assembles `/rtabmap/cloud_map` in the `map` frame; octomap_server
  voxelizes that into `/occupied_cells_vis_array`.

**Why the odometry was fine but the voxels weren't.** Because `body→camera_link` has **zero
translation**, `body` and `camera_link` share an origin — only their *orientation* differs. So the
trajectory, which is the sequence of frame **origins**, is identical whether that rotation is right or
wrong, and the path always looked correct. The voxels, however, are depth points projected outward
along the camera's **orientation**; get that rotation wrong by 90° and every point swings 90° about the
trajectory, so the whole map lands off to the side ("trash on the right") even though the path it was
built from is perfect.

**The bug (fixed).** OKVIS *also* broadcast `body→camera_link`, setting it to `T_BS` from
`okvis2/config/realsense_D435i.yaml` — which is **identity** (no rotation). That collided with the
launch's correct static publisher: two latched `/tf_static` publishers of the *same* edge, and tf2
keeps whichever it received last, nondeterministically per run. Identity winning → 90°-off map; the
launch's value winning → good map; a late overwrite → correct map with a few phantom startup voxels.
This is exactly why pre-publishing the static TF didn't help and why a post-hoc RTAB-Map/octomap reset
was an unreliable band-aid.

**The fix.** We do **not** touch `T_BS` — it also defines OKVIS's body frame for state estimation, so
rotating it would rotate the odometry the controller consumes. Instead we removed OKVIS's broadcast of
that edge — a local patch in `ros2/third_party/okvis2/okvis_ros2/src/Publisher.cpp` (`setBodyTransform`);
it only existed to hang the camera mesh in RViz, which we don't use (we visualize in Foxglove). The
launch's `static_transform_publisher` is now the single, authoritative owner of `body→camera_link`, and
RTAB-Map's `wait_for_transform:=3.0` makes it block for that latched static TF before assembling its
first cloud. (This is a deliberate edit to vendored third-party code — see the `third_party/` note.)

## Build

Because the core is no longer a ROS package (it carries `COLCON_IGNORE`), building is **two steps**:
build+install the standalone core with plain CMake, then colcon-build the ROS side pointing at that
install. Run everything from the **colcon workspace root** (`~/ws_paramio`), not from this `src/`.

```bash
cd ~/ws_paramio

# 1. Build + install the ROS-free core (plain CMake). Do this with ROS NOT sourced to prove it.
cmake -S src/core -B build/core -DCMAKE_BUILD_TYPE=Release
cmake --build build/core
cmake --install build/core --prefix install_core    # any prefix; goes on CMAKE_PREFIX_PATH below

# 2. Build the ROS wrappers. Put install_core on the CMAKE_PREFIX_PATH *env var* with an ABSOLUTE
#    path so find_package(drone_core) resolves — see the warning below.
export CMAKE_PREFIX_PATH=/home/dron/ws_paramio/install_core:$CMAKE_PREFIX_PATH
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_NN=OFF
source install/setup.bash
```

`autonomy_node` links the core's static libs, so there is no runtime dependency on the core install —
only a build-time one via `CMAKE_PREFIX_PATH`. `-DUSE_NN=OFF` is required (okvis2 defaults it on,
which needs LibTorch); `-DCMAKE_BUILD_TYPE=Release` is strongly wanted (VIO+SLAM are too slow on the
NUC un-optimized).

Besides the usual apt packages the core needs `libdynamicedt3d-dev` (version-matched to octomap),
**OSQP** (a system CMake install providing `libosqpstatic.a`, double precision) and **DecompUtil**
(header-only, from https://github.com/sikang/DecompUtil; configure it with
`-DCMAKE_POLICY_VERSION_MINIMUM=3.5` on CMake >= 4, and keep its source outside `src/` since it ships
a `package.xml` colcon would pick up). OSQP is found by
concrete library path rather than `find_package`, and linked PRIVATE into `drone_core_planning` with
its C API confined to one `.cpp`, so it never enters the core's public export and consumers need no
`find_dependency` for it.

**Pointing colcon at the core — use the env var, not `-D`.** Do *not* rely on
`--cmake-args -DCMAKE_PREFIX_PATH=$PWD/install_core`: `$PWD` has to be exactly the workspace root, and
colcon manages `CMAKE_PREFIX_PATH` per-package and can shadow the `-D` cache override, so
`find_package(drone_core)` fails with *"Could not find a package configuration file ... drone_core"*
even though the core installed correctly. Exporting an absolute `CMAKE_PREFIX_PATH` (as above) is
reliable because colcon *extends* that env var for every package.

**Incremental colcon builds are the norm — don't wipe to rebuild.** Re-running `colcon build`
rebuilds only what changed; a full wipe forces a ~4–5 min `px4_msgs` plus a long `okvis2` rebuild for
nothing. The only situation that warrants a clean is the stale-cache error *"source ... does not match
the source ... used to generate cache"*, which colcon throws when a package's path moves (it bit us
once during the restructure when files moved under `src/ros2/`). Even then, remove just the offending
package's `build/<pkg>` and `install/<pkg>` rather than the whole tree. Treat a full workspace wipe as
a deliberate, rare action — never a routine build step.

## Test

```bash
# Core unit tests, no ROS — the proof the core is ROS-free.
cmake -S src/core -B build/core -DDRONE_CORE_BUILD_TESTS=ON
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

Tests (plain CTest, no gtest): `frames`, `position_control`, `feedforward` (proves feed-forward
OFF ≡ baseline), `flatness_mapper`, `planner`, `corridor` (truncation, polyhedral corridor generation
including the start relaxation and its floor, and the corridor QP, against analytic clearance fields
and synthetic obstacle clouds — fast, no map),
`autonomy_core` (plan→track→watchdog).

**Keep any test run under 30 s** (`ctest --timeout 30 -R <name>`) and never run the full suite —
`planner` drives real OMPL solves and has been seen spinning for minutes.

## Deployment notes

The onboard computer is an Intel NUC (i5, 16 GB RAM, no GPU), which is tight for VIO + SLAM +
planning at once — the camera is capped at 640×480@15fps, OKVIS runs CNN-less with ≤400 keypoints,
RTAB-Map is tuned light, and RRT* runs on its own thread (≤3 s) so it never stalls the 50 Hz control
loop. The operator drives the NUC over SSH/tmux with the laptop and NUC on a shared phone hotspot,
and uses Foxglove (`foxglove_bridge`, port 8765) for remote visualization. Before flying any planned
trajectory, verify map-frame consistency: the octomap/RTAB-Map "map" frame must share an origin with
the controller's VIO-rooted ENU (the TF tree is patched by the `body→camera_link` static transform
folded into `autonomy_vision_launch.py`).
