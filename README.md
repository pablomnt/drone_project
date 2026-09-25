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

Everything in these types is world-frame ENU with one deliberate exception, called out because it
looks like a violation: **`State::thrust_accel`**. It is a scalar, it is in the *body* frame, and it
reads **9.81 in hover rather than 0** — because it is not an acceleration. It is thrust per unit mass
along the vehicle's own up axis, which is the only thing a quadrotor's accelerometer can actually
report: gravity is not felt, and lateral acceleration comes from tilting, which keeps thrust along
body z. Its sole consumer is the hover-thrust estimator, whose model assumes thrust acts along body
z — the same assumption the sensor makes, which is why rotating it into the world would *introduce*
an error rather than remove one. It is a scalar precisely so a world-frame acceleration cannot be
passed in by mistake. If you need a genuine measured acceleration, do not use this field; take
`VehicleLocalPosition.ax/ay/az` and add it as a separate one.

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
- **Online hover-thrust estimation**: filters the thrust gain — `State::thrust_accel` (measured
  thrust per unit mass along body up, *not* a vertical acceleration, see `common/`) divided by the
  motor-lag-filtered command — and takes hover thrust as 9.81 ÷ gain, clamped to [0.2, 0.5], with
  the learning rate de-weighted at high vertical speed; overridable via `MPC_HOVER_THRUST`. The
  noisy accel is kept on top of the fraction on purpose: the old per-sample `command·9.81/accel` put
  the noise underneath and read 0.31 against a true 0.29 under flight vibration. That change
  (`2deb93a`, 2026-09-14) flew on 2026-09-16 and holds altitude correctly. This is the only consumer of a measured IMU quantity
  anywhere in the controller.
- **Differential-flatness feed-forward** (added on top, default OFF): `setReference()` accepts
  `vel_ff`/`acc_ff`; with feed-forward disabled the controller is byte-identical to the baseline (a
  unit test asserts this), and it is suppressed during the takeoff ramp. The accel→attitude/thrust
  map *is* the flatness output stage and already existed — which is why trajectory tracking did not
  require a new controller.

`flatness_mapper` samples the trajectory for pos/vel/acc and derives yaw from the velocity heading
(so the forward camera leads the motion), holding the last yaw only when slow *and* the heading is
spinning. `trajectory_tracker` composes the mapper + controller and runs the watchdog state machine
described in the next section.

**How vehicle state reaches the controller.** The node assembles one `common::State` and hands the
whole struct to `AutonomyCore::setVehicleState()`, which stores it under a lock; the planner worker
reads it for its start state, and `stepControl()` passes it to `TrajectoryTracker::update()`. The
tracker is what unpacks it into the controller's two setters: `PositionControl::setState(pos, vel,
yaw)` for the feedback quantities the PID closes on, and `setThrustAccel()` for the hover-thrust
calibration, which calibrates a slowly-varying scale factor rather than feedback and so gets its own
door (the input itself is very noisy; the estimator averages it).
The core method is named `setVehicleState` rather than `setState` specifically so it does not read
like a call to the controller's unrelated `setState` one layer down. `State::stamp` is the time the
position sample was taken (not the tick time) and feeds the D term's measurement clock (below);
nothing else reads it, and it is not a freshness guard.

The velocity **D term differentiates the measurement on the estimator's clock, not the control
clock.** `_vel` is a zero-order hold — the node re-sends the newest VIO sample every 20 ms whether or
not a new one arrived — so differencing per tick yielded zero on held ticks and double the true rate
on the others. The raw derivative is now recomputed only when `State::stamp` advances, divided by the
real measurement interval — so a varying sample rate needs no assumption about the rate — and a
low-pass (`MPC_VEL_D_TAU`) runs every tick to keep the term smooth in between. Differentiating the measurement rather than the error also drops the setpoint's own
derivative, which removes the kick a stepped `POS_SP` used to inject.

### Control modes (`TrajectoryTracker`)

The drone is always in exactly one of three control modes. They live in the `Mode` enum in
`core/control/trajectory_tracker.hpp` and are chosen fresh on every 50 Hz tick inside
`TrajectoryTracker::update` (`core/control/src/trajectory_tracker.cpp`). Nothing outside the tracker
sets the mode — there is no command to enter one. The tracker looks at what guidance it currently
holds and picks, and the operator's influence is entirely indirect: publish a goal, or fire a preset,
and the mode follows.

The three modes, in the order they are tested:

1. **`kTracking` — a fresh trajectory exists, so follow it.** This is the only mode that flies a
   trajectory, and the only one where differential-flatness feed-forward is active; the other two
   explicitly disable it. "Fresh" means a trajectory is installed and the last one *arrived* within
   `STALE_TIMEOUT` seconds. Freshness is stamped when the trajectory is handed over, not when it
   starts playing, so a planner that has died trips the timeout even if the trajectory it last
   produced is still perfectly valid and still running.
2. **`kHoverHold` — a trajectory is installed but guidance has gone stale, so latch the current
   position and hover.** This is the failsafe for the planner stalling or dying. It captures the
   position and yaw once on entry and holds them; it does **not** land, because PX4 rejects an
   offboard land command. This is also the mode the tracker starts in after `reset()`, and the
   fallback when nothing has been commanded at all.
3. **`kDirect` — no trajectory at all, so follow a single explicit position setpoint.** That
   setpoint is `POS_SP`, the ROS parameter, pushed in by the node every tick. It is the mode used for
   takeoff and for manual hover.

The ordering is the part worth internalising, because it is strict and it surprises people: a
trajectory outranks the direct setpoint **even when that trajectory is dead**. A stale trajectory
falls to `kHoverHold`, not back to `kDirect`. So `POS_SP` is a pre-takeoff and no-goal setpoint only
— there is no "return to setpoint" path through it, and changing it while anything is installed does
nothing at all. The only things that restore `kDirect` are `reset()` (a disarm or leaving offboard)
and an explicit `clearTrajectory()`.

Two things about `kDirect` matter for the `PRESET_WAYPOINTS` test in particular.

**First, `POS_SP` stops having any effect the moment the preset's trajectory is staged.** The node
keeps reading the parameter and pushing it into the tracker on every tick, and the tracker keeps
storing it — silently, with no warning — while `kTracking` ignores it. This is why
`firePresetSquare` writes `POS_SP` *at fire time*, before the trajectory exists, rather than when the
preset finishes: at completion it would be too late to matter on the tick it is needed. Moving
`POS_SP` mid-preset accomplishes nothing; the value that counts is the one set at the flip.

**Second, returning to `kDirect` when the preset finishes is deliberate, not automatic.** A preset is
solved once and never replanned, so nothing re-stamps its freshness the way normal planning does —
left alone it would go stale within `STALE_TIMEOUT` and, by the precedence above, land in
`kHoverHold`, latching wherever the vehicle happened to be. So `AutonomyCore::stepControl` does two
things explicitly: it keeps the preset fresh for its whole duration so it holds `kTracking` to the
end, and then calls `clearTrajectory()` at `preset_end_` so control drops to `kDirect` on `POS_SP`
instead of latching a hover. That explicit clear is the entire reason the vehicle returns to its
setpoint rather than parking in place.

A third point is worth noting for reading the results: feed-forward runs **only** during the preset
itself. The hover before and after is plain PID on a fixed setpoint, so the preset window is the only
part of the flight where trajectory generation and the flatness feed-forward are actually under test.

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

**Threading.** The node runs a `MultiThreadedExecutor` over two *mutually exclusive* callback groups:
a **fast** group holding the 20 ms control tick and every estimator stream that feeds it (PX4
odometry, VIO, `sensor_combined`, vehicle status, joystick, goals), and a **slow** group holding the
octomap and frontier callbacks and the 500 ms visualisation timer. The map callback blocks for
150–520 ms on every update — it deserializes the tree, deep-copies it for the conservative view,
stamps the frontier and walks every leaf — and on the previous single-threaded executor that work sat
directly in front of the control tick. A flight on 2026-07-31 stalled the loop 39 times in 137 s, once
per map update; two of those stalls exceeded the 0.5 s `SENSOR_TIMEOUT`, so the tick that followed saw
all three estimator streams stale at the same instant and the watchdog auto-landed the aircraft, even
though every stream had been publishing normally the whole time. Replaying that same bag after the
split gives zero stalls over 0.06 s. Because both groups are mutually exclusive, state that stays
inside one group needs no locking; only the few members read across the two (the drone position that
centres the frontier keep-out, and the goal marker fields) are guarded, by `cross_mutex_`. If you add
a callback, put anything that can block longer than a tick in the slow group — see `CLAUDE.md` for
the full rules.

**Sensor-health watchdog.** The node guards the three estimator streams it depends on — PX4 odometry,
IMU (`sensor_combined`) and VIO — by timestamp, deeming each *healthy* only while its last sample is
within `SENSOR_TIMEOUT` (0.5 s). Two gates use this. **Before takeoff** the controller refuses to
*engage* until every required stream has been continuously healthy for `SENSOR_WARMUP` (5 s), so it
never takes off on a stream that arrived once and then died (the old check only asked whether a stream
had *ever* been received). **In flight**, if any required stream goes stale it commands `NAV_LAND`
(PX4 AUTO.LAND) and latches — re-commanding every tick until PX4 confirms the mode, since a single
`VehicleCommand` can be dropped — and re-arms only after touchdown/disarm. That is a real landing,
distinct from the core's `kHoverHold` (which only hovers): `NAV_LAND` is a mode switch that works from
offboard, whereas the *core's* offboard-attitude stream has no land primitive. The timeout error names
every stream that went quiet, with its topic and how long it has been silent, and the pre-takeoff gate
announces `READY TO FLY` once the warmup completes so the operator gets a positive signal rather than
the hold warnings merely stopping. Both `SENSOR_TIMEOUT` and `SENSOR_WARMUP` are live-reconfigurable
params.

One caveat to read the log with: the watchdog cannot tell a dead sensor from a control loop that did
not get to run — both leave the same stale timestamps. If it reports several streams stale at once,
by nearly the same amount, suspect a stall in the node rather than the sensors; independent streams
from different processes do not die in the same millisecond.

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

## Threading model

Everything the drone runs sits in separate ROS 2 processes. `autonomy_node` is the one this repo
owns; the perception stack around it is third-party. On the NUC (i5-8259U, 4 cores / 8 threads) a
flight session looks roughly like this, with the CPU figures from a 2026-09-23 bench run:

```
 NUC (8 logical CPUs)
 ├─ okvis_node        ~22 threads, ~170% CPU   visual-inertial odometry
 ├─ rtabmap           ~91 threads,  ~32% CPU   SLAM + the occupancy octomap
 ├─ realsense2_camera ~29 threads,  ~16% CPU   camera driver
 ├─ MicroXRCEAgent                             PX4 uORB bridge
 └─ autonomy_node     ~16 threads,  ~21% CPU   this repo (below)
```

Inside `autonomy_node`, four threads matter. The rest are DDS internals (discovery, delivery,
timers) that we neither create nor tune.

```
 autonomy_node
 ├─ main thread ................. spins the executor, then idle
 │
 ├─ executor thread A — FAST callback group (mutually exclusive)
 │     50 Hz control tick: AutonomyCore::stepControl -> tracker -> PX4 attitude setpoint
 │     estimator callbacks: VIO odom, PX4 odom, sensor_combined, vehicle_status, joy, goal
 │
 ├─ executor thread B — SLOW callback group (mutually exclusive)
 │     onOctomap (150-500 ms per map), onFrontier, the 2 Hz debug visualisation
 │
 ├─ executor thread C — default group
 │     parameter services only
 │
 ├─ SEARCH thread (AutonomyCore::searchLoop, ticks at RRT_MONITOR_PERIOD)
 │     re-checks the committed path, runs the geometric search when it is blocked
 │     or on the improve cadence, adopts a new committed path
 │
 └─ TRAJGEN thread (AutonomyCore::trajgenLoop, ticks at TRAJGEN_PERIOD)
       reads the committed path, truncates it, grows the corridor, solves the QP,
       stages the trajectory for the tracker; also plays one-shot presets
```

**Why the two callback groups.** A single-threaded executor let `onOctomap` block the control tick
for 150-520 ms per map update; the first tick afterwards saw all three estimator streams stale at
once and the in-flight watchdog landed the aircraft (2026-07-31). Both groups are *mutually
exclusive*, so state used within one group needs no locking; only what genuinely crosses them is
guarded (`cross_mutex_` in the node).

**Why the search and trajgen threads are separate.** They used to be one loop, with the search
first. A geometric search that overran its budget — measured at 30-59 s against a 1 s budget on the
2026-09-17 bench — therefore stopped trajectory generation for as long as it ran, and past
`STALE_TIMEOUT` the tracker latches hover-hold and the drone brakes. Split, a slow search only
delays a *better* path: trajgen keeps regenerating on the committed path every `TRAJGEN_PERIOD`.
That is safe because each trajgen tick re-truncates the path and regrows the corridor against the
*current* map, so a path running into something newly mapped is cut short of it rather than flown.
Shared state is small and each lock is held briefly: the committed path (`path_mutex_`), the two
cached distance fields (`edt_mutex_`), the staged trajectory and viz snapshots (`traj_mutex_`), and
the inputs from the host — state, map, goal, transform (`io_mutex_`). Each planner thread keeps its
own copy of the config, as the control thread already did.

**Rule of thumb when adding work:** anything that can block for longer than one 20 ms control tick
belongs in the slow group or on a thread of its own, never on the fast group.

## Runtime parameters

Every parameter below is declared by `autonomy_node` and is **live-reconfigurable**, armed or not:
`ros2 param set /autonomy_node <NAME> <VALUE>` reaches the planner within one planner cycle and the
controller on its next tick, and the node logs `Parameter <NAME> = <VALUE>` when it has taken it. A
value of the wrong type is refused with a warning rather than applied. (Before 2026-09-17 neither
held: a change pushed the *previous* value, and planning parameters only reached the core once the
drone was armed, so on a disarmed bench they never changed after launch.) There are no launch-file
overrides and no YAML parameter file: the defaults in the table are the `declare_parameter` calls in
`ros2/autonomy_node/src/autonomy_node.cpp`, and that file is the only place to change what the drone
comes up with.

Two defaults are currently set for **bench and tuning work** rather than for a real flight; they are
marked ⚑ and listed again at the end of this section.

### Mode and control

| Parameter | Type / default | What it does |
|---|---|---|
| `USE_SIM_MODE` | bool, `false` | Take state from PX4 odometry instead of VIO, and require only that stream to be healthy. For SITL. |
| `ENABLE_FEEDFORWARD` | bool, `true` | Differential-flatness feed-forward (`vel_ff` before the velocity PID, `acc_ff` after it). Active only in `kTracking`, and suppressed during the takeoff ramp. False makes the controller byte-identical to the flight-tested baseline. |
| `POS_SP` | double[3], `[0, 0, 1.5]` | Takeoff / manual-hover setpoint in ENU metres. Used **only** in `kDirect` — ignored while any trajectory is installed. |
| `MPC_XY_P` / `MPC_Z_P` | double, `0.95` / `1.0` | Position-loop proportional gains (outer cascade). Flight-tuned. |
| `MPC_XY_VEL_P/I/D` | double, `2.0` / `0.9` / `0.5` | Horizontal velocity-loop PID gains. Flight-tuned. While tracking with feed-forward, D damps the measured acceleration relative to the trajectory's `acc_ff` rather than the raw measurement, so it does not oppose planned acceleration (`0f3ebf9`, not yet flown). |
| `MPC_Z_VEL_P/I/D` | double, `2.6` / `0.8` / `0.2` | Vertical velocity-loop PID gains. Flight-tuned. |
| `MPC_VEL_D_TAU` | double, `0.04` s | Low-pass time constant on the D term. The raw derivative refreshes only when a new VIO sample lands (~25 Hz), so this keeps the term smooth between measurements. Roughly one sample period; raise for less noise, lower for less phase lag. |
| `MPC_INT_ERR_MAX` | double, `0.2` m | Position error above which the velocity integrator is frozen (held, not reset), judged separately for XY (norm) and z. Stops a long move winding it up and overshooting on arrival. `≤ 0` disables. |
| `MPC_HOVER_THRUST` | double, `0.33` | Seed for the online hover-thrust estimator (normalised 0–1). Setting it live re-seeds the estimate. |

### Safety and timeouts

| Parameter | Type / default | What it does |
|---|---|---|
| `STALE_TIMEOUT` | double, `2.0` s | How long after a trajectory's *arrival* the tracker keeps tracking it before falling to `kHoverHold`. Guards against a dead planner, not a stale map. |
| `MAX_TRACKING_ERROR` | double, `1.0` m | If the vehicle gets further than this from the trajectory's reference, it gives up on that trajectory: it holds its current position, drops anything planned against the old reference, and the planner searches again and generates a new trajectory from there, starting at rest. Latched — it never resumes the abandoned trajectory. Catches what `STALE_TIMEOUT` cannot: guidance still arriving on time while the vehicle has been knocked off course. Logged as `[track] vehicle … m from the trajectory reference`. During a preset it holds until the preset's scheduled end, then returns to `POS_SP` (presets are never replanned). `≤ 0` disables. |
| `BENCH_TEST_REPLAN_DISABLER` | bool, `false` | **Bench only.** Every replan starts at rest from the drone's measured position instead of splicing onto where the current trajectory says the drone should be by now. On a disarmed bench nothing flies the trajectory, so without this each replan starts further along it and the trajectory shrinks to nothing within its own duration. **Never fly with it on**: every replan would restart from zero velocity, a stutter every `TRAJGEN_PERIOD`. The node warns every 2 s if it is on while the controller is engaged. |
| `SENSOR_TIMEOUT` | double, `0.5` s | A stream counts as healthy if it produced a sample within this window. Drives both guards below. |
| `SENSOR_WARMUP` | double, `5.0` s | Continuous stream health required before the controller will *engage*. Any lapse resets the streak, so every takeoff re-proves it. |

Two guards use these. Before takeoff, the controller refuses to engage until every required stream
has been healthy for `SENSOR_WARMUP`, and the node prints a positive "you may arm" on the rising edge
rather than just falling silent. In flight, any required stream going stale commands `NAV_LAND` and
latches it until PX4 confirms `AUTO.LAND`. Note the in-flight watchdog cannot distinguish "the
sensors died" from "the control loop did not get to run" — see the caveat in `CLAUDE.md`.

### Planning

| Parameter | Type / default | What it does |
|---|---|---|
| `PLAN_TRAJECTORY` | bool, `true` | Master gate on trajectory generation from the **planner**. False stops the worker after the geometric search (path still published for viz) and control stays on `POS_SP`. Does **not** gate `PRESET_WAYPOINTS`. With it on, **do not have a goal live when you arm**: there is no airborne gate, so a staged trajectory can pre-empt the takeoff ramp (see `CLAUDE.md`). |
| `PRESET_WAYPOINTS` | bool, `false` | **Momentary trigger, not a mode.** A `false→true` edge fires one preset trajectory through waypoints hardcoded in `firePresetSquare`, then the node resets it to false. Currently four waypoints in `map`: the drone's position, then +0.5 m in x at 1.3 m, +1.5 m x / +0.5 m y at 2.0 m, and +2.5 m x at 1.3 m. It does **not** end where it started: `POS_SP` is moved to the first waypoint on fire, so after completion the drone flies back there on `POS_SP`. Bypasses the geometric planner, solves the corridor QP once, holds it to completion, then returns control to `POS_SP`. Also clears any active goal (the only goal-cancel path there is). Needs a map; refuses a fire below 0.8 m while flying. |
| `PLANNER_TYPE` | string, `"EITstar"` | Which OMPL planner to build: `RRTstar`, `BITstar`, `ABITstar`, `AITstar`, `EITstar`. Per-planner internals are **not** parameters — they live in `PlannerConfig` in `geometric_planner.hpp`. |
| `RRT_MONITOR_PERIOD` | double, `1.0` s | How often the worker re-checks the committed path for collisions. |
| `RRT_IMPROVE_PERIOD` | double, `10.0` s | How often it attempts an improvement search on an already-valid path. |
| `RRT_SOLVE_TIME` | double, `1.0` s | Optimisation budget per solve. All the planners are anytime, so this is a direct quality/latency dial. |
| `REPLAN_IMPROVE_RATIO` | double, `0.85` | Hysteresis gate: adopt an improvement only if its cost ≤ ratio × the committed path's **remaining** cost from the drone's current position. Prevents replan chatter. |
| `BEST_EFFORT_GOAL` | bool, `true` | Accept a path that stops short of an unreachable goal (closest reachable point) instead of reporting failure, and keep advancing the endpoint as the map grows. |
| `TRAJGEN_PERIOD` | double, `1.0` s | Trajectory-generation cadence; each run re-anchors onto the outgoing trajectory. |

### Cost shaping

| Parameter | Type / default | What it does |
|---|---|---|
| `CLEARANCE_WEIGHT` | double, `1.0` | Weight on the obstacle-proximity penalty. Raising it pushes the search off walls — and, when frontier stamping is on, off the frontier too, which is the main lever on how much of a path survives truncation. Costs longer detours. |
| `CLEARANCE_THRESHOLD` | double, `1.0` m | Distance at which the proximity penalty saturates; also the EDT's `maxdist`. |
| `UNKNOWN_WEIGHT` | double, `0.5` | Flat extra cost per metre routed through never-observed space. Read as "how many metres of detour through mapped space is one metre through unmapped space worth". **Keep it low** — it defocuses the informed planners badly, because their sampling ellipse is built from straight-line estimates that cannot see this term. Ignored entirely when `TREAT_FRONTIER_AS_OBSTACLE` is false. |

### Corridor and limits

| Parameter | Type / default | What it does |
|---|---|---|
| `USE_CORRIDOR_QP` | bool, `true` | Use the corridor-constrained QP instead of plain min-snap. False is the legacy, obstacle-blind path. |
| `TREAT_FRONTIER_AS_OBSTACLE` | bool, `false` | **The single switch for "is unmapped space a hazard".** Gates the stamped frontier shell, `UNKNOWN_WEIGHT`'s surcharge, and truncation's stop at unobserved cells. On a fresh map almost everything is frontier, so with this on the drone is boxed in until it has scanned around itself — leave it off for bench and preset work, on for real exploration. |
| `VMAX` / `AMAX` / `JMAX` | double, `1.0` / `1.5` / `3.0` | Per-axis velocity, acceleration and jerk limits enforced by the QP. Box bounds, so the true norm can reach √3× in the corner case. |
| `FRONTIER_MARGIN` | double, `0.5` m | Clearance the committed *path prefix* keeps from unknown space during truncation. |
| `ESCAPE_RAMP_DIST` | double, `1.0` m | Distance over which truncation's clearance requirement ramps from zero at the drone up to the full margin, so a vehicle in a tight spot can still commit a path. Also where the corridor's first-region relaxation ends. Deliberately independent of the margin. `≤ 0` disables both. |
| `CORRIDOR_MARGIN` | double, `0.4` m | Clearance the corridor *regions* keep from obstacles. A strictly harder test than the planner's own check — it must hold over a whole 3D volume, not just a centreline. **This is the clearance you actually fly with**, so weigh it against the airframe's half-width. First suspect when a decomposition fails. |
| `MAX_SEGMENT_LEN` | double, `2.0` m | Corridor resample cap; one convex region per piece. Lowering it is the lever against convex over-conservatism, but costs QP size — and needs `CORRIDOR_BBOX` pinned or you lose in region width what you gain in length. |
| `CORRIDOR_BBOX` | double[3], `[1, 2, 2]` | Minimum usable half-extents of the region-growth window, in the **segment-aligned** frame (0 = along-track, 1/2 = lateral) — a floor, not a literal size. Exists to decouple window size from `MAX_SEGMENT_LEN`. All zeros restores purely derived behaviour. |
| `TRAJ_PATH_WEIGHT` | double, `0.0` (off) | How hard the QP pulls the trajectory toward the planned path. The knob against wide, corner-cutting turns: the QP otherwise scores smoothness alone, so inside a roomy corridor the widest turn is the cheapest one and only the corridor holds the curve near the plan. Going faster cannot fix it — the same curve is simply flown faster. Soft, so unlike hard waypoint pinning it can never make a feasible corridor infeasible, and it leaves tight scenery its full set of options. Weighted by segment length, so the weight keeps one meaning however finely the corridor happens to be split. Try 20-100; the unit test's L-corner goes from 0.30 m to 0.19 m of deviation at 50. |
| `TRAJ_SOLVE_BUDGET` | double, `1.0` s | Wall-clock cap on the QP's time-allocation search. When it runs out the best allocation found so far is used — feasible, just slower. Covers the QP only, not truncation or corridor building. `<= 0` = unlimited. |

### Debug

| Parameter | Type / default | What it does |
|---|---|---|
| `DEBUG_PLANNER_VIZ` | bool, `true` ⚑ | Single switch for the planner debug visualisation: search tree, EDT clearance field, and the corridor stages on `/planner/corridor`. Zero-cost when off (nothing is extracted, sampled or published). Turn it off for a real flight so the NUC pays nothing. |
| `DEBUG_CONTROL_VIZ` | bool, `true` ⚑ | Publishes `/control/pos_ff`, a sphere at the reference point the controller is chasing. Runs on the 50 Hz control tick, so unlike the planner viz it is **not** free when on; kept as its own switch for that reason. |

### ⚑ Currently set for bench and tuning work

| Parameter | Current default | Flight value | Why |
|---|---|---|---|
| `DEBUG_PLANNER_VIZ` | `true` | `false` | `/planner/corridor` is what separates a margin collapse from a failed validation from a QP infeasibility when a trajectory refuses to generate. |
| `DEBUG_CONTROL_VIZ` | `true` | `false` | `/control/pos_ff` is what shows tracking lag in RViz; it costs a publish every control tick. |

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

## Commands used in flight

Day-to-day operator commands, in roughly the order you use them. Run from the workspace root
(`~/ws_paramio`) with the workspace sourced (`source install/setup.bash`); the NUC is driven over
SSH/tmux from the laptop.

**Connect to the drone (NUC):**
```bash
ssh dron@172.20.10.3
```

**Bring up the full stack** (camera/VIO/mapping + control + Foxglove):
```bash
ros2 launch autonomy_node autonomy_vision_launch.py              # with RViz
ros2 launch autonomy_node autonomy_vision_launch.py rviz:=false  # headless (no RViz)
```

**Set the takeoff / hover setpoint** (`POS_SP`, ENU metres). Pre-takeoff / no-goal only — a live goal
overrides it and it is unreachable while any trajectory is installed (see *Control modes*):
```bash
ros2 param set /autonomy_node POS_SP "[0.0, 0.0, 1.5]"
```

**Fire the preset trajectory** (`PRESET_WAYPOINTS`) — the trajectory-generation and feed-forward test,
with the planner bypassed entirely. Take off and settle into a stable hover on `POS_SP` above 0.8 m
first, and confirm the node has logged `First octomap received` (the corridor pipeline needs a map or
the fire is dropped). The parameter is a momentary trigger and resets itself, so set it again to fly
it again:
```bash
ros2 param set /autonomy_node PRESET_WAYPOINTS true
```
Watch for `PRESET_WAYPOINTS fired: 4 waypoints from (…) in map` from the node, then either `[preset]
trajectory staged: L m / T s` or `[preset] trajectory generation FAILED — staying on POS_SP` from the
core. On failure nothing is staged and the vehicle keeps hovering — there is no fallback to an
unchecked polynomial. Leaving offboard is the abort; there is no in-flight cancel.

**Send a navigation goal** (`/planner/goal`, position only — yaw is ignored). Goals are planned in
RTAB-Map's `map` frame: `frame_id: map` (or empty) is used as is, `frame_id: world` (OKVIS
coordinates) is converted to `map` once on arrival, anything else is rejected. Planning waits for
RTAB-Map's `map→world` transform, so a goal sent before RTAB-Map is up logs `[plan] idle: … no
map->world transform yet` until it arrives:
```bash
ros2 topic pub --once /planner/goal geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0, z: 1.3}}}"
```

**Record a flight.** Curated, low-CPU topic set (controller telemetry, VIO, PX4 I/O, committed plan,
octomap, the `/okvis/cam0_matches/compressed` "what the drone sees" view, and `/rosout` — the console
log of every ROS node, including the core's own planner/trajgen lines, which `main()` bridges from the
core's `std::cerr` into ROS logging). Start it by hand when
you decide to capture; bags land in `~/flight_logs/` (override with `FLIGHT_LOG_DIR`). The scripts
auto-pick mcap storage if installed and fall back to sqlite3 otherwise:
```bash
./src/ros2/autonomy_node/scripts/record_flight.sh   # always-on flight recorder — Ctrl-C to stop
./src/ros2/autonomy_node/scripts/record_debug.sh    # heavy tier: planner viz + camera + depth (debugging only)
```

The same two tiers by hand, when you want the topic list visible and editable on the spot (drop
`--storage mcap` if the mcap plugin isn't installed; sqlite3 is the default and playback auto-detects
either). **Standard set** — what `record_flight.sh` records:
```bash
cd ~/flight_logs && ros2 bag record --storage mcap --max-bag-duration 120 \
  /debug/telemetry /okvis/cam0_matches/compressed /okvis/okvis_odometry /okvis/okvis_path \
  /fmu/in/offboard_control_mode /fmu/in/vehicle_command /fmu/in/vehicle_attitude_setpoint_v1 \
  /fmu/out/vehicle_status /smooth_trajectory /planner/geometric_path /planner/goal_marker \
  /control/pos_ff /rtabmap/octomap_binary /telemetry/cpu_usage_total /rosout /tf /tf_static
```

**Estimator-debug set** — the standard set plus every stream the node's sensor-health watchdog
actually times, so a `SENSOR TIMEOUT (> 0.50s) … Stale: <stream>` auto-land can be reconstructed
afterwards: which stream stopped, when, and whether the others kept flowing.
```bash
cd ~/flight_logs && ros2 bag record --storage mcap --max-bag-duration 120 \
  /debug/telemetry /okvis/cam0_matches/compressed /okvis/okvis_odometry /okvis/okvis_path \
  /fmu/out/sensor_combined /fmu/out/vehicle_odometry /fmu/out/vehicle_status_v1 \
  /fmu/in/offboard_control_mode /fmu/in/vehicle_command /fmu/in/vehicle_attitude_setpoint_v1 \
  /fmu/out/vehicle_status /smooth_trajectory /planner/geometric_path /planner/goal_marker \
  /control/pos_ff /rtabmap/octomap_binary /telemetry/cpu_usage_total /rosout /tf /tf_static
```

Two things to know about the extra topics. `/fmu/out/sensor_combined` is the raw IMU stream (~100 Hz
received in flight) — by far the heaviest thing in either list, so use this tier when you are chasing a
timeout, not as the always-on recorder on the NUC. (It is bridged with **no rate limit**, unlike
every other topic in PX4's `dds_topics.yaml`, and carries no vibration filtering — so it looks very
noisy and that is expected. `onSensorCombined` accumulates every sample and the 50 Hz
control loop uses the per-tick mean, so vibration is averaged at message rate rather than aliased.) And `/fmu/out/vehicle_status_v1` is the topic the
node actually subscribes to (PX4 v1.17); the standard set's `/fmu/out/vehicle_status` is the legacy
name and can land in the bag with zero messages — if `ros2 bag info` shows that, record the `_v1`
topic instead (or add the `qos_overrides.yaml` flag noted in `record_flight.sh`, since a best-effort
QoS mismatch produces the same empty result).

**Record per-thread CPU while flying** (optional, for chasing slow trajectory solves). A preset solve
takes 1-3 s on the NUC and gets slower as the map grows, and there are two very different reasons it
could: the solve is doing more work (more obstacle points, more corridor faces), or the planning
thread is simply getting less CPU because RTAB-Map and OKVIS have grown heavier. Per-thread CPU tells
them apart — during a slow solve, a planning thread near 100% means more work, while one well below
means it is being starved. Start it in its own tmux pane once the stack is up, and Ctrl-C it after
landing:
```bash
top -H -b -d 1 -p $(pgrep -f autonomy_node | head -1) > ~/flight_logs/top_threads_$(date +%F-%H%M%S).txt
```
Line it up afterwards with the `[preset] firing` / `trajectory staged` timestamps in `/rosout` (the
gap between them is the solve time) and with `/telemetry/cpu_usage_total` in the bag.

**Review afterwards.** Playback is storage-agnostic — the same command works for sqlite3 or mcap bags:
```bash
ros2 bag info ~/flight_logs/rosbag2_<timestamp>
ros2 bag play ~/flight_logs/rosbag2_<timestamp>
```

**Read the console log back** (`/rosout`: every ROS node plus the core's `[plan]`/`[trajgen]` lines).
Reads the bag directly — no playback needed:
```bash
./src/ros2/autonomy_node/scripts/show_log.py ~/flight_logs/rosbag2_<timestamp>
```
Interactively instead, with severity/node filters: `ros2 bag play <bag>` in one terminal and
`ros2 run rqt_console rqt_console` in another.

**Watch the camera / plot the controller.** The camera is recorded already-compressed, so nothing has
to be decompressed — `rqt_image_view` decodes it directly. Play the bag, then in a second terminal:
```bash
ros2 run rqt_image_view rqt_image_view    # pick /okvis/cam0_matches/compressed
```
For position / velocity / acceleration / PID plots, PlotJuggler reads the bag file directly (no
playback): `ros2 run plotjuggler plotjuggler`, then Data → Load File and drag `pos/x` + `pos_sp/x`
(etc.) from `/debug/telemetry` onto a plot. Foxglove Studio can also open the `.mcap` directly and
show camera, plots and log on one timeline. Note `/debug/telemetry` only publishes while **armed and
in offboard**, so it is empty in a bench recording.
