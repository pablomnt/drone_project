# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**Answering style: be brief.** Lead with the answer. Don't develop reasoning at length unless asked.

**HARD RULE — 30-second test budget.** Never let a test run longer than **30 s**: pass
`--timeout 30` to `ctest`, and if something is still going at 30 s, **kill it** and report that it
was killed. Do not wait it out, do not background it and check back, do not "give it another
minute". **Never run the full suite** (`ctest` with no `-R`) — run only the specific fast test for
what you changed, e.g. `ctest --test-dir build/core --timeout 30 -R corridor`. `planner` and
`autonomy_core` drive real OMPL solves and are the slow ones; `planner` in particular has been
observed spinning for **8+ minutes** — treat it as long-running and do not run it casually.

## Project

`drone_project`: software for an autonomous vision-based quadcopter. The drone runs PX4 firmware,
uses a RealSense D435i + OKVIS2 for visual-inertial state estimation and RTAB-Map for SLAM/mapping,
plans with RRT* + minimum-snap trajectories, and tracks them with a custom cascaded-PID /
differential-flatness controller that outputs attitude + collective thrust to PX4 over uXRCE-DDS.

The repo is split into **two top-level domains** (this `src/` is the only git repo; the colcon
workspace root `~/ws_paramio` holds `build/`, `install/`, `log/` and is not under version control):

1. **`core/` — the ROS-free autonomy core (`drone_core`).** Pure C++17, plain CMake, **builds and
   unit-tests with zero ROS installed**. Depends only on ROS-free libraries: Eigen, octomap (the
   library, not `octomap_msgs`), OMPL, NLopt. This is where all guidance and control logic lives.
   It carries a `COLCON_IGNORE` so colcon never tries to build it.
2. **`ros2/` — thin ROS wrappers + vendored third-party ROS packages.** The colcon/ament side. The
   one first-party node (`autonomy_node`) does only message↔core-type translation, frame
   conversions, and PX4 I/O; the heavy third-party packages (`okvis`, `px4_msgs`) live under
   `ros2/third_party/` as in-tree copies of their upstream repos.

The long-term goal is a clean, startup-grade codebase that can drop ROS (and eventually PX4)
without rewriting the autonomy logic — hence ROS is confined to the I/O shell and PX4's failsafe is
**not** relied upon (the core has its own watchdog; see below).

**Deployment/operation**: the onboard computer is an Intel NUC (i5, 16GB RAM, no GPU) — this drives
the tuning choices noted under *NUC resource constraints*. The operator controls the NUC over SSH
in a tmux session, with the laptop and NUC tethered to a phone hotspot; Foxglove (`foxglove_bridge`
on port 8765) is used for remote visualization. This workflow is manual and not codified in the
repo — there are no tmux/SSH/Foxglove config files to find.

## Directory layout

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
    ├── third_party/            # okvis2 (pkg name 'okvis'), px4_msgs  (vendored copies)
    ├── sim/pc_publisher/       # sim-only static .pcd publisher
    └── tools/system_monitor_pkg/   # python CPU/RAM telemetry
```

## Build

Two steps: build+install the standalone core, then colcon-build the ROS side pointing at it.

```bash
# 1. Build + install the ROS-free core (plain CMake). Do this with ROS NOT sourced to prove it.
cmake -S src/core -B build/core -DCMAKE_BUILD_TYPE=Release
cmake --build build/core
cmake --install build/core --prefix install_core      # any prefix; put it on CMAKE_PREFIX_PATH below

# 2. Build the ROS wrappers, pointing find_package(drone_core) at the core install.
#    Put install_core on the CMAKE_PREFIX_PATH *env var* with an ABSOLUTE path — colcon manages
#    that var per-package, so this reliably reaches autonomy_node. Do NOT rely on
#    `--cmake-args -DCMAKE_PREFIX_PATH=$PWD/install_core`: $PWD must be exactly the ws root, and
#    colcon can shadow the -D cache override, so find_package(drone_core) fails ("Could not find ...
#    drone_coreConfig.cmake") even though the core installed fine.
export CMAKE_PREFIX_PATH=/home/dron/ws_paramio/install_core:$CMAKE_PREFIX_PATH
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_NN=OFF
source install/setup.bash
```

`autonomy_node` links the core's **static** libs (`drone_core::autonomy` etc.), so there is no
runtime dependency on the core install — only a build-time one via `CMAKE_PREFIX_PATH`.

The `--cmake-args` that matter (all consumed by `okvis2`; other packages ignore them):

- **`-DUSE_NN=OFF` — required to build.** `okvis2` defaults `USE_NN=ON`, which needs LibTorch
  (not installed, no GPU). The runtime config disables the CNN anyway (`use_cnn: false`), so this
  costs nothing.
- **`-DCMAKE_BUILD_TYPE=Release` — strongly wanted.** `okvis2` only *warns* otherwise, but VIO+SLAM
  are too slow on the NUC un-optimized. Also build the core Release for flight.
- **`-DBUILD_ROS2=ON` — redundant** (already the okvis2 default).

Package names vs. directories mostly match, except `ros2/third_party/okvis2/` whose package name is
`okvis`. `okvis2` is a large vendored VIO library with its own `external/` deps (ceres, brisk,
DBoW2, opengv, googletest); see `ros2/third_party/okvis2/README` for its CMake options.

## Test / Lint

- **Core unit tests (no ROS).** This is the primary test path and the proof the core is ROS-free.
  Always bound the run and select the test for what you changed — see the 30-second rule at the top
  of this file:
  ```bash
  cmake -S src/core -B build/core -DDRONE_CORE_BUILD_TESTS=ON
  cmake --build build/core
  ctest --test-dir build/core --timeout 30 --output-on-failure -R corridor   # NOT the whole suite
  ```
  Tests (plain CTest, no gtest): `frames`, `position_control`, `feedforward` (proves feed-forward
  OFF ≡ baseline), `flatness_mapper`, `planner`, `corridor` (fast, map-free corridor-QP checks
  against analytic oracles — run it with `ctest -R corridor` on every planning edit),
  `autonomy_core` (plan→track→watchdog).
- ROS packages use `ament_lint_auto` / `ament_lint_common` via `colcon test`; the python packages
  (`pc_publisher` C++ aside, `system_monitor_pkg`) carry the standard flake8/pep257/copyright triplet.

## Runtime architecture

The pipeline is now fully connected: perception → mapping → **planning → control** → PX4.

```
RealSense → OKVIS2 (VIO: /okvis/okvis_odometry) → RTAB-Map (ray-traced 3D
         occupancy octomap: /rtabmap/octomap_binary)
                                   │
                      ┌────────────┴───────────── autonomy_node (thin ROS wrapper) ─────────────┐
                      │  subscribes: octomap, okvis odom, px4 odom, sensor_combined,            │
                      │              vehicle_status, /planner/goal, /joy                         │
                      │  owns a drone_core::autonomy::AutonomyCore, drives it at 50 Hz,          │
                      │  converts ENU↔NED/FRD at the boundary, publishes attitude+thrust to PX4  │
                      └────────────────────────────────────────────────────────────────────────┘
                                   │
                                  PX4 (via MicroXRCEAgent, serial /dev/ttyUSB0 or UDP for SITL)
```

Launch files live in `ros2/autonomy_node/launch/`: `autonomy_vision_launch.py` (real flight, full
camera/VIO/mapping stack + Foxglove + the `body→camera_link` static TF), `autonomy_launch.py`
(xterm-per-process variant), `autonomy_sim_launch.py` (`USE_SIM_MODE:=true`, `MicroXRCEAgent udp4`).

### The autonomy core (`core/`)

`AutonomyCore` (`core/autonomy/`) is the orchestrator and the object a host (the ROS node, a sim
harness, or a test) drives. It is middleware-free and runs cadences on a background worker thread
and the fast control thread:

**Background worker (planning, geometry-first phase):** one loop ticking at the monitor cadence
(`rrt_monitor_period`, ~2 Hz). Each tick builds **one** planner and does:
- **Monitor (every tick):** re-checks the committed path via `GeometricPlanner::isPathValid`. If a
  point's clearance has dropped below the margin it's `path_invalid`.
- **Improve (every Nth tick,** `N = round(rrt_improve_period / rrt_monitor_period)` ≈ 10): flagged
  only with a committed path and obstacles mapped.
- A **search runs iff** `path_invalid || improve_run`. If `path_invalid`, adopt the result
  **unconditionally** (safety); else (improve, path valid) adopt **only if**
  `candidate_cost ≤ replan_improve_ratio × remaining_committed_cost` (hysteresis, prevents chatter).
  The baseline is the committed path's **remaining** cost from the drone's *current* position — not
  its full original cost — computed by `remainingCommittedSuffix()` (exact point-to-segment projection
  of the drone onto the committed polyline, then `pathCost([drone_pos, wp_ahead…, goal]`)). Both sides
  are thus rooted at the drone, so a candidate can't win merely because the drone has advanced toward
  the goal (which would shrink a drone-rooted candidate while the full committed cost still billed the
  already-traversed prefix). One search ⇒ one
  EDT use per tick. Both validity and cost use the **same cached `DynamicEDTOctomap`** (`maxdist =
  clearance_threshold`), rebuilt only when the map object changes (see `clearanceField`), not per
  tick. Clearance-aware cost `= ∫(1 + w·max(0,thresh−d)) ds`. Path post-processing: with **no**
  clearance field, plain length-only `simplifyMax`; in clearance mode, a **cost-aware shortcut**
  (`shortcutClearanceAware`) drops an interior waypoint only when the straight bypass is
  collision-free *and* doesn't raise the clearance-aware cost — so it removes the RRT* zig-zag
  without the wall-hugging that pure `simplifyMax` would cause (a `kMaxShortcutSegment` cap keeps
  waypoint density for min-snap). Each tick logs one line (phase, committed cost, min clearance,
  blocked/why, outcome); OMPL's own console is set to `LOG_WARN` to keep the terminal clean.
  A tick that **cannot** plan (no goal, or no map) logs `[plan] idle: …` naming the missing
  precondition, on every change of reason and then every 5 s — otherwise "I sent a goal and nothing
  happened" is indistinguishable from a dead worker, since the planning branch is the only one that
  produces output. The node pairs this with a throttled warning while no octomap has arrived, which
  reports the **resolved** map topic and its publisher count: zero publishers means the map source is
  silent (RTAB-Map only emits on motion-gated map-graph updates, so it needs camera motion *and*
  usable odometry), a non-zero count means a QoS or remap mismatch on our side.
- **Trajectory generation (~1 Hz, `trajgen_period`):** re-anchors min-snap to current position.
  **Gated by `plan_trajectory` flag** — when false the worker is a pure geometric planner and
  control stays on `kDirect` / POS_SP.
- **Trajectory tracking + flatness mapping (50 Hz)** on whatever thread calls `stepControl()`.

Module roles:

- **`common`** — shared plain types (`State`, `Reference`, `Command`, `Trajectory`, `Goal`,
  `MapHandle`) and **`frames`**, which holds *all* ENU↔NED/FRD and OKVIS-yaw conversions (extracted
  from the old node so they are unit-testable). `logging.hpp` replaces ROS logging in the core.
- **`planning`** — `GeometricPlanner` (a **runtime-selectable** OMPL planner over an SE(3) octree,
  X/Y ±15 m, Z −1.5–2.5 m — `PlannerType` ∈ {RRTstar, BITstar, ABITstar, AITstar, EITstar}, built by
  a small factory `makePlanner()`; the BIT* lineage is heuristic/informed and far better at focusing
  the search on the start→goal corridor than plain RRT*). **All per-planner tunables live in
  `PlannerConfig` in `geometric_planner.hpp`** (the single place to tune them — edit + rebuild the
  core); only `PLANNER_TYPE` is a ROS param. For the informed/heuristic planners to work the clearance
  objective must supply **both** an edge heuristic (`motionCostHeuristic` → Euclidean distance between
  states) **and** a state→goal cost-to-go heuristic (`setCostToGoHeuristic(goalRegionCostToGo)`); both
  are admissible lower bounds because the integrand is ≥1. Without the cost-to-go one OMPL warns that
  informed sampling "will have little to no effect" and RRT*'s ellipse / the BIT* goal heuristic stay
  blind.
  **Collision check is EDT-based**: a state is free when its clearance (3D Euclidean distance to the
  nearest obstacle, via a `DynamicEDTOctomap` passed as the clearance fn) exceeds `kCollisionMargin`
  (0.5 m) — one O(1) lookup, and because the distance is 3D it enforces **vertical** clearance too
  (the old horizontal-only box is now only a fallback in `isStateValid` for standalone/test use with
  no field set). **Start-state escape sphere**: within `kStartEscapeRadius` (0.5 m) of the start the
  required clearance drops to `kStartMargin` (0 m) so a parked/just-lifting drone — sitting within the
  normal margin of the mapped floor — can root the tree; obstacles are never skipped (clearance must
  still exceed 0, i.e. not inside a voxel). Anchored at the start in `planPath`, at the first waypoint
  in `isPathValid`. `kGoalFlexibility` (0.3 m): RRT* approximate solutions that stop farther than this
  from the goal are **rejected** (`planPath` returns false → caller sees infinite cost) rather than
  committing to a path that ends short. All four are `static constexpr` in the class (deliberately not
  ROS params). The same field feeds the **clearance-aware cost** via `setClearance(fn, weight,
  threshold)` (obstacle-proximity integral penalty up to `clearance_threshold`); `isPathValid()`
  re-checks a committed path under the same model; `pathCost()` scores a waypoint list for the
  hysteresis gate; `minClearance()` reports a path's tightest clearance (diagnostic / logging).
  `MinSnapTrajectory` / `MinSnapTimeOptimizer` (KKT min-snap + NLopt BOBYQA
  time allocation, "mode A" of ETH's mav_trajectory_generation). The optimizer emits a `Trajectory`
  of **per-segment polynomial coefficients + durations** (not sampled points) so the flatness mapper
  can differentiate it analytically.
  **Corridor-QP trajectory generation (Stage 1, gated by `USE_CORRIDOR_QP`)** replaces plain
  min-snap with a provably collision-free pipeline over a **dual map view** (see `setMap`): the
  geometric search runs on the raw **optimistic** map (unknown = free, so EIT*/BIT* accept a goal
  beyond the mapped frontier instead of failing "no goal states available"), while safety comes from
  the **conservative** (frontier-stamped) view. `corridor.{hpp,cpp}`: `truncatePath` cuts the
  committed path where conservative clearance drops below `FRONTIER_MARGIN` (the requirement ramps
  from 0 at the drone to the full margin over `ESCAPE_RAMP_DIST` — the escape-sphere idea without its
  dead band, and with the ramp *length* decoupled from the margin so a large margin doesn't make the
  ramp steep), `resamplePath`
  + `buildCorridor` grows one maximal free **convex polyhedron** (at `CORRIDOR_MARGIN`) per
  ≤`MAX_SEGMENT_LEN` piece via **DecompUtil**'s ellipsoid inflation (Liu et al. RA-L 2017, the
  decomposition FASTER uses): the ellipsoid grows along the segment and cuts a half-space at each
  obstacle that binds, so the region follows the path instead of a bounding box. It consumes an
  obstacle **point list** (occupied + frontier-stamped voxel centres, windowed around the prefix by
  `runTrajgen` — a whole room at 5 cm is 1e5-1e6 points and DecompUtil rescans per segment), while
  `truncatePath` keeps using the EDT **clearance oracle**; tests drive both with analytic data. Faces
  are pulled in by `CORRIDOR_MARGIN` + the voxel half-diagonal (obstacle points are voxel *centres*),
  then each region is validated for a non-empty overlap with its neighbour, since C0 continuity pins
  the junction into that intersection and shrinking can empty it. `corridor_trajectory.{hpp,cpp}`: `CorridorTrajectoryOptimizer`
  solves degree-7 min-snap as an **OSQP QP** — monomial coefficients (same snap `Q`), C0–C4
  continuity, rest-to-rest ends, Bézier control points of position confined to the regions and of
  vel/acc/jerk within per-axis `VMAX/AMAX/JMAX` (hull property ⇒ the whole curve complies; solved in
  per-segment normalized time or OSQP stalls on conditioning), plus a feasibility-aware BOBYQA time
  search (infeasible ⇒ flat penalty; velocity-consistent seed grown until feasible). Fallback ladder
  in `runTrajgen`: corridor failure → plain min-snap on the **truncated prefix** (never the full
  optimistic path); empty truncation → stage nothing (tracker rides out the old trajectory into the
  stale hover-hold). Each fallback logs which stage failed and why — `truncated to N wp / L m`
  (pipeline refusing to commit toward unknown space), `decomposition FAILED (<which check>) …
  tightest conservative clearance C m vs required M m` — the parenthetical names the actual check
  that rejected it (margin shrink pushed a region past the start/goal, or two regions stopped
  overlapping), since the clearance figure is context, not the cause — or `QP INFEASIBLE over S
  regions` (corridor fine, no trajectory fits it within `VMAX/AMAX/JMAX`). These are distinct faults
  needing opposite fixes, hence distinct messages. A successful corridor logs one line only when
  `DEBUG_PLANNER_VIZ` is on.

  **A known, correct-looking-wrong interaction.** `truncatePath` ramps its requirement near the drone
  (`ESCAPE_RAMP_DIST`) so a vehicle in a tight pocket can root a path at all, but a convex region
  cannot be "less safe at one end" — the corridor applies `CORRIDOR_MARGIN` uniformly, including
  right next to the drone. So truncation can commit a short prefix the corridor then refuses. That is
  the pipeline telling the truth (there is no trajectory with that clearance through that space), not
  a bug to hunt.
- **`control`** — see the flight-critical note below.

### control (`core/control/`) — flight-critical, be careful

**`position_control.{hpp,cpp}` holds flight-tested logic for a real vehicle.** The PID gains,
hover-thrust estimator constants, and open-loop takeoff ramp have been tuned and proven stable in
flight ("Drone flies :)"). Do not change the gains or control math unless the task specifically
requires it; validate any change in `USE_SIM_MODE`/SITL first. Notable behavior:

- **Open-loop takeoff override**: on a ground `reset()` it primes a takeoff; when a setpoint above
  0.5 m arrives it ramps thrust open-loop (flat attitude) until liftoff, then hands to the PID —
  because closed-loop control near the ground with noisy VIO causes skidding.
- **Online hover-thrust estimation**: back-calculates hover thrust from filtered command + measured
  vertical accel, de-weighting the estimate at high vertical speed; overridable via `MPC_HOVER_THRUST`.
- **Differential-flatness feed-forward (added on top, default OFF)**: `setReference()` accepts
  `vel_ff`/`acc_ff`; with feed-forward disabled the controller is byte-identical to the baseline
  (the `feedforward` unit test asserts this). Toggle live with the `ENABLE_FEEDFORWARD` parameter.
  Feed-forward is suppressed during the takeoff ramp. The accel→attitude/thrust map *is* the flatness
  output stage and already existed — that's why trajectory tracking didn't require a new controller.

`flatness_mapper` samples the trajectory for pos/vel/acc and derives **yaw from the velocity
heading** (camera leads the motion), holding the last yaw only when slow *and* the heading is
spinning. `trajectory_tracker` composes the mapper + controller and runs the **watchdog state
machine**: `kDirect` (explicit setpoint — takeoff/manual hover), `kTracking` (follow a fresh planner
trajectory), `kHoverHold` (failsafe: latch current position when guidance goes stale, i.e. the
planner stalled/died). The stale fallback hovers; it does not land (PX4 rejects offboard land).

### autonomy_node (`ros2/autonomy_node/`)

Thin wrapper. Per 50 Hz tick: publishes the offboard heartbeat, checks arm/offboard (resets the
core on disarm or leaving offboard, re-priming takeoff), assembles an ENU `State` from the active
estimator (VIO in normal mode; PX4 odom in `USE_SIM_MODE`), feeds the core, reads back the `Command`,
applies the **yaw-drift correction** (toward PX4's yaw) and the ENU→NED/FRD conversion via
`core/common/frames`, and publishes the attitude setpoint + `ControllerDebug` telemetry. Goals
arrive on `/planner/goal` (geometry_msgs/PoseStamped, position only — yaw ignored); a `POS_SP`
parameter provides the default takeoff/hover setpoint. All PX4 message types and frame conversions
are confined to this file.

**Sensor-health watchdog (node-level, distinct from the core's `kHoverHold` / `STALE_TIMEOUT`
planner-stale→hover path).** The node stamps a per-stream last-receive time (`t_px4_odom_ /
t_sensor_ / t_vio_odom_`) in each estimator callback; `streamHealthy()` deems a stream healthy when
its last sample is within `SENSOR_TIMEOUT` (default 0.5 s) and `streamsFresh()` ANDs the required set
(all three normally, PX4 odom only in sim). There are **no** `has_*` "ever received" flags — freshness
subsumes them. Two guards use this:
- **Pre-takeoff gate** (replaces the old `dependenciesReady()`): the controller will not *engage*
  until every required stream is healthy **and** has been continuously healthy for `SENSOR_WARMUP`
  (default 5 s), tracked via `all_healthy_since_` (any staleness resets the streak; any interruption —
  disarm or leaving offboard — resets it too, so each takeoff re-proves 5 s of health). Gated on
  `!controller_running_` so it only blocks the engage transition; it must never early-return while
  flying (that would silently disengage instead of landing). A throttled log names which streams are
  stale and the warmup progress.
- **In-flight watchdog**: every tick while flying (`controller_running_`), if `streamsFresh()` is
  false it commands `land()` (`VEHICLE_CMD_NAV_LAND` → PX4 AUTO.LAND) and **latches**
  (`failsafe_landing_`), re-commanding LAND each tick until `vehicle_status_.nav_state` reads
  `NAVIGATION_STATE_AUTO_LAND` (a single VehicleCommand can be dropped, and NAV_LAND itself drops us
  out of offboard — hence the latch block sits *above* the armed/offboard gate). The latch clears only
  on the **disarm edge** (touchdown), never on leaving offboard, so it cannot un-latch mid-descent.

`SENSOR_TIMEOUT` and `SENSOR_WARMUP` are live-reconfigurable node params.

**State feed is independent of arm/offboard** (fixes an earlier bug where plans rooted at the map
origin on the disarmed bench). The ENU `State` assembly + `core_->setState()` sit **above** the
`armed && offboard` gate, so the core — and thus the planner worker, which reads `state_` for its
`start` — always has the drone's live position, armed or not. The feed is gated on the **position
source** being fresh (`streamHealthy(t_vio_odom_)` in normal mode, `t_px4_odom_` in sim) rather than
on the full `streams_healthy` set: on the battery-off bench the flight controller may be unpowered, so
PX4 odom + `sensor_combined` never arrive, but VIO alone is all the planner's start needs. Control
(`stepControl` + attitude publish) stays **below** the arm/offboard gate, so output is still only
produced when armed+offboard; every tick reaching it is a healthy tick (the watchdog lands otherwise),
so the state it reads is always populated.

**Visualization publishers (at 2 Hz via `viz_timer_`):**
- `/planner/geometric_path` (`visualization_msgs/MarkerArray`) — raw RRT* waypoints as a green
  LINE_STRIP + blue SPHERE_LIST, frame `map`. Populated even when `PLAN_TRAJECTORY=false`.
- `/planner/goal_marker` (`visualization_msgs/Marker`) — red sphere at the active goal position.
  Published every viz tick so late-joining RViz sessions see it immediately.
- `/smooth_trajectory` (`nav_msgs/Path`) — sampled min-snap trajectory. Dormant while
  `PLAN_TRAJECTORY=false`.

**Debug-only planner visualisation** (gated by `DEBUG_PLANNER_VIZ`, default off — see below). These
publish nothing and cost nothing when the flag is off:
- `/planner/search_tree` (`visualization_msgs/MarkerArray`) — the search tree from the most recent
  solve: faint grey `LINE_LIST` edges + orange `POINTS` nodes. Per-solve snapshot (persists between
  searches). Lets you watch where the planner explored — A/B `PLANNER_TYPE` and tune `RRT_SOLVE_TIME`
  (and per-planner knobs in `geometric_planner.hpp`) by eye.
- `/planner/clearance_field` (`sensor_msgs/PointCloud2`, `intensity` = clearance distance) — coarse
  (0.15 m) samples of the cached EDT, colour near→far. Shows exactly what the clearance cost "sees",
  for tuning `CLEARANCE_WEIGHT` / `CLEARANCE_THRESHOLD`. Sampled on the worker thread only when the
  map changes.
- `/planner/corridor` (`visualization_msgs/MarkerArray`) — the corridor-QP pipeline's intermediate
  products, **published on failed ticks too**, stage by stage, because a failing corridor is what you
  need to look at (clearing the drawing made "viz broken", "flag off" and "failing every tick" look
  identical). Face **outlines** rather than filled hulls, since regions overlap by construction and
  stacked translucent solids turn to mush:
  - grey outlines — the polyhedra **as decomposed**, before `CORRIDOR_MARGIN` is applied;
  - cyan outlines — the same regions **after** the margin pull-in, when the corridor was accepted;
  - red outlines — ditto when it was **rejected**;
  - white line strip — the truncated committed prefix (drawn as soon as truncation succeeds);
  - orange sphere — the truncation endpoint, the intermediate goal in known-safe space, which should
    ratchet toward the red goal marker as the room is mapped.

  Reading it: grey present but nothing coloured ⇒ the margin collapsed the regions, so there is less
  room than `CORRIDOR_MARGIN` demands. Red ⇒ regions survived the shrink but failed validation (the
  `[trajgen]` line names which check). Where the white line stops short of the green
  `/planner/geometric_path` is where truncation refused to commit into unknown space. Needs
  `PLAN_TRAJECTORY` + `USE_CORRIDOR_QP` + `DEBUG_PLANNER_VIZ`.

**Planning-related node parameters** (all live-reconfigurable via `onParameterChange`):
- `PLAN_TRAJECTORY` (bool, default `true`) — the geometry-first phase gate. False stops the worker
  after the geometric search (path published for viz, control stays on `POS_SP`); true runs trajgen
  and hands trajectories to the tracker. **Now defaulted true for Stage 1 bench validation** — set it
  false to go back to pure geometric planning.
- `RRT_MONITOR_PERIOD` (double, default `0.5` s) — path validity re-check cadence.
- `RRT_IMPROVE_PERIOD` (double, default `5.0` s) — clearance-aware improvement search cadence.
- `RRT_SOLVE_TIME` (double, default `5.0` s) — planner optimisation budget per solve (all planners
  are anytime). Drop it to ~1–3 s while tuning so each solve refreshes the tree viz quickly.
- `PLANNER_TYPE` (string, default `"RRTstar"`) — which OMPL planner the worker builds:
  `RRTstar | BITstar | ABITstar | AITstar | EITstar`. Live-reconfigurable, so you can A/B them on the
  bench against the tree viz. **Per-planner internals (RRT* range/goal-bias, BIT*/AIT*/EIT* batch
  sizes, rewire factors, …) are NOT ROS params — they live in `PlannerConfig` in
  `geometric_planner.hpp`**; edit there and rebuild the core to tune a specific planner. (RRT*'s old
  `RRT_RANGE` param is gone — it's now `PlannerConfig::rrtstar.range`.)
- `REPLAN_IMPROVE_RATIO` (double, default `0.85`) — adopt candidate iff cost ≤ ratio × committed.
- `CLEARANCE_WEIGHT` (double, default `1.0`) — obstacle-proximity penalty weight. Lowered from 4.0
  during bench work; raising it pushes the search off the walls, which is one way to buy the corridor
  stage the room it needs to grow regions.
- `CLEARANCE_THRESHOLD` (double, default `1.0` m) — clearance saturation / EDT maxdist.
- `DEBUG_PLANNER_VIZ` (bool, default `true`) — **single switch** for the debug planner
  visualisation (search tree, clearance field, corridor). Designed to be zero-cost when off: with it
  false the planner never extracts the OMPL tree (`getPlannerData`), the EDT is never sampled, the
  corridor snapshot is never taken, and the publish methods early-return. **Now defaulted true for
  bench validation** — turn it off for a real flight so the NUC pays nothing for it.
- `TREAT_FRONTIER_AS_OBSTACLE` (bool, default `true`) — stamp RTAB-Map's frontier cloud
  (`/rtabmap/octomap_global_frontier_space`, remapped in the launch) as occupied voxels into a deep
  **copy** of each incoming octomap, fed to the core as the **conservative** map view (the raw map
  stays the optimistic search view — dual-map; a small keep-out ball around the drone stays
  unstamped so it can root the search). With `USE_CORRIDOR_QP` off the search itself runs on the
  conservative view (legacy behavior); with it on, the conservative view only drives truncation +
  corridor. NOTE: on a fresh map almost everything is frontier — the drone is boxed in until it has
  scanned its surroundings.
- `BEST_EFFORT_GOAL` (bool, default `true`) — accept an approximate geometric solution that stops
  short of the goal (the reachable point closest to it) instead of reporting "no path"; the worker
  keeps advancing the endpoint as the map grows.
- `USE_CORRIDOR_QP` (bool, default `true`) — switch trajgen from plain min-snap to the
  corridor-constrained QP pipeline (see the planning module notes). Off = byte-identical legacy
  trajgen. Needs `TREAT_FRONTIER_AS_OBSTACLE` for the frontier to count as unsafe (without it the
  conservative view equals the raw map and nothing guards against unknown space).
- `VMAX` / `AMAX` / `JMAX` (double, defaults `1.0` / `1.5` / `3.0`) — per-axis velocity /
  acceleration / jerk limits enforced by the corridor QP (conservative box bounds; the true norm can
  reach √3× in the corner case).
- `FRONTIER_MARGIN` (double, default `0.5` m) — clearance the committed *path prefix* keeps from
  unknown space (the truncation margin against the conservative EDT). Enforced exactly as set; a
  committed point must still have strictly positive clearance whatever the value, so the prefix can
  never reach into an occupied or unknown voxel.
- `ESCAPE_RAMP_DIST` (double, default `1.0` m) — distance over which truncation's required clearance
  ramps from 0 at the drone up to the full `FRONTIER_MARGIN`. **Deliberately independent of the
  margin.** Ramping over the margin itself makes the requirement climb at 1 m/m, so on a thinly
  mapped scene the rising requirement meets the shrinking clearance within centimetres and the
  committed path collapses to almost nothing (observed on the bench: 0.25 m committed out of a
  2.47 m path). A longer ramp commits further before demanding full clearance. `<= 0` disables the
  ramp entirely. Clearance must still be strictly positive everywhere, so leniency near the start is
  never blindness — the prefix cannot enter an occupied or unknown voxel at any setting.
- `CORRIDOR_MARGIN` (double, default `0.4` m) — clearance the *corridor regions* keep from obstacles
  and unknown space. **This is a strictly harder test than the planner's `kCollisionMargin`**: the
  search validates only its centreline (and exempts a sphere at the start), while the corridor needs
  the margin over a whole 3D volume of the frontier-stamped map. Lowering it does not forfeit the
  safety guarantee — the trajectory stays provably confined to the regions, so it keeps exactly this
  clearance — but it is the clearance you actually fly with, so weigh it against the airframe's
  half-width. If `[trajgen] corridor: decomposition FAILED` reports a tightest conservative clearance
  below the required margin, this is the knob.
- `MAX_SEGMENT_LEN` (double, default `2.0` m) — corridor resample cap; one free region per piece.
- `CORRIDOR_BBOX` (double[3], default `[0, 0, 0]` = derive) — **minimum usable** half-extents of the
  region-growth window in the **segment-aligned** frame (component 0 = slack along the segment, 1/2 =
  lateral), not world axes. A floor, not a literal size: `buildCorridor` scales it with the longest
  segment (lateral ≥ segment length, along-track ≥ half of it, so consecutive regions overlap
  generously) and adds the margin pull-in on top, so the window planes can never eat into the volume
  the trajectory can actually use. Raise a component for wider regions, at the cost of more obstacle
  points per decomposition.

**Planning mode matrix.** Three flags compose into the pipeline's operating modes. `PLAN_TRAJECTORY`
gates trajgen entirely; `USE_CORRIDOR_QP` picks the trajgen algorithm; `TREAT_FRONTIER_AS_OBSTACLE`
decides whether a conservative map view exists at all (without it the conservative view *is* the raw
map, so nothing distinguishes unknown space from free space).

| `PLAN_TRAJECTORY` | `USE_CORRIDOR_QP` | `TREAT_FRONTIER` | Search map | Trajgen | Guards unknown space |
|---|---|---|---|---|---|
| false | *(any)* | false | raw | none — control stays on `POS_SP` | n/a |
| false | *(any)* | true | conservative | none — control stays on `POS_SP` | search only |
| true | false | false | raw | plain min-snap | **no** |
| true | false | true | conservative | plain min-snap | search only (trajectory may still cut corners between waypoints) |
| true | true | false | raw | corridor QP | obstacles only — corridor is collision-free but unknown reads as free |
| true | true | true | **optimistic** (raw) | corridor QP | **yes** — full Stage 1: optimistic search, conservative truncation + corridor |

Notes: with `PLAN_TRAJECTORY=false` the corridor snapshot is never populated, so `/planner/corridor`
stays empty regardless of the other flags — trajgen is where truncation and corridor growth happen.
`BEST_EFFORT_GOAL` is orthogonal and matters whenever the goal is unreachable or unmapped: true
(default) commits to the closest reachable point and ratchets forward, false holds. The last row is
the only configuration that delivers the Stage 1 safety claim.

> **Instrumentation principle (apply to any future debug/viz):** keep it behind a *single*,
> *default-off* runtime parameter and make it *zero-cost when off* — no extraction, no allocation, no
> publishing — so it never taxes a real flight on the NUC, and never touches the flight-critical
> control path. `DEBUG_PLANNER_VIZ` is the reference example.

**Additional dependency**: `libdynamicedt3d-dev` (apt, version-matched to octomap 1.9.7). Used in
`autonomy_core.cpp` only; linked into `drone_core_autonomy`. Headers at
`<dynamicEDT3D/dynamicEDTOctomap.h>`. Re-install if rebuilding on a fresh machine.

**DecompUtil (safe-flight-corridor decomposition)**: system CMake install at `/usr/local` from
https://github.com/sikang/DecompUtil — **header-only**, so `find_package(decomp_util)` supplies only
`${DECOMP_UTIL_INCLUDE_DIRS}`, added PRIVATE to `drone_core_planning` with its types confined to
`corridor.cpp`; nothing is linked and no target reaches the `drone_core` export. Its CMake declares
`cmake_minimum_required(VERSION 2.8.3)`, so on CMake >= 4 it must be configured with
`-DCMAKE_POLICY_VERSION_MINIMUM=3.5`. Keep the source out of `src/` — it ships a `package.xml` and
colcon would try to build it. Note an upstream bug we route around: `EllipsoidDecomp::add_global_bbox`
(3D) places both the +Y and -Y planes at `global_bbox_max_(1)`, so the `(origin, dim)` constructor is
never used — only `set_local_bbox`.

**OSQP (corridor-QP solver)**: system CMake install at `/usr/local` (v1.x C API,
`libosqpstatic.a`). Deliberately located as a **concrete library path** (`find_library` in
`src/core/CMakeLists.txt`), *not* `find_package(osqp)`, and linked **PRIVATE** into
`drone_core_planning` with the C API confined to `corridor_trajectory.cpp` — so the exported
`drone_coreTargets.cmake` carries no `osqp::` target and consumers need no `find_dependency(osqp)`.
The solver requires a double-precision OSQP build (static_assert on `OSQPFloat`).

### Frames & TF (OKVIS ↔ RTAB-Map) — why odom can look fine while the voxel map is wrong

A RealSense depth point reaches the world through
`world ──(OKVIS VIO)──▶ body ──(static body→camera_link)──▶ camera_link ──(RealSense REP-103)──▶ *_optical_frame`.
OKVIS estimates `world→body` (the trajectory); `body→camera_link` is the fixed camera mount, a
**pure rotation with zero translation** (`pitch -1.5708, roll 1.5708`) published by the
`static_transform_publisher` in `autonomy_vision_launch.py`; RTAB-Map (base `camera_link`) builds and
publishes the occupancy octomap directly (`/rtabmap/octomap_binary`; see *Map source* below).

Because that edge has **zero translation**, `body` and `camera_link` share an origin: the trajectory
(a sequence of origins) is unaffected by a wrong rotation, but the voxels (depth projected along the
camera's *orientation*) swing 90° when it's wrong — so a bad extrinsic gives correct-looking odometry
with a sideways map. **The bug:** OKVIS *also* broadcast `body→camera_link`, set to `T_BS` (identity
in our config), colliding with the launch's publisher — two latched `/tf_static` publishers of the
same edge race nondeterministically per run (random 90°-off maps; pre-publishing the static TF or
post-hoc map resets can't fix it). **The fix:** leave `T_BS` alone (it also defines OKVIS's estimation
body frame, so rotating it would rotate the controller's odometry) and instead drop OKVIS's broadcast
(local patch in `Publisher.cpp::setBodyTransform`, RViz-mesh-only and unused). The launch now solely
owns the edge; RTAB-Map's `wait_for_transform:=3.0` blocks for the latched TF before its first cloud.

### Map source (RTAB-Map octomap) — why it's not octomap_server anymore

The occupancy map fed to the planner is **RTAB-Map's own 3D octomap** (`/rtabmap/octomap_binary`),
not a standalone `octomap_server`. The motivation was **clearing**: `octomap_server` was voxelizing
the *assembled* `/rtabmap/cloud_map`, which carries no per-scan sensor origin, so it could only ever
*add* occupied voxels — phantom obstacles never cleared even when you flew up to them. RTAB-Map builds
its grid per-node with the real viewpoints, so `Grid/RayTracing true` carves free space along each ray
and stale voxels decay back to free. The relevant `Grid/*` args (in `autonomy_vision_launch.py`):
`Grid/3D true`, `Grid/RayTracing true`, `Grid/CellSize 0.05`, `Grid/RangeMax` (far-noise vs.
look-ahead), `Grid/DepthDecimation 1` + `Grid/NormalsSegmentation false` (density — rtabmap otherwise
decimates and drops "ground", which for a flying drone is a real obstacle), and a gentle
`Grid/NoiseFiltering*` pair for isolated speckle. There is **no `map_always_update`** — RTAB-Map only
republishes the assembled octomap on map-graph updates (motion-gated), which is fine because the node
latches and the subscriber retains the last map (below).

**Two non-obvious gotchas the switch exposed (both fixed in `autonomy_node.cpp`):**
1. **`ColorOcTree`, not `OcTree`.** RTAB-Map publishes a `ColorOcTree` (it stores voxel colour);
   `octomap_server` published a plain `OcTree`. The core/`DynamicEDTOctomap` needs an `OcTree`, so a
   naive `dynamic_cast<OcTree*>` silently dropped *every* map and the planner sat idle ("New goal"
   then nothing). `onOctomap` now converts via the `toOcTree()` helper (copies occupancy, drops
   colour, **expands coarse/pruned leaves** to resolution voxels so walls don't get holes).
2. **QoS durability.** RTAB-Map's octomap publisher is `RELIABLE` + `TRANSIENT_LOCAL` (latched) and
   publishes sparsely (only on motion-gated updates). A `VOLATILE` subscriber never receives the
   retained sample, so on a static scene `map_` stayed null. The octomap subscription is now
   `TRANSIENT_LOCAL` so it pulls the last map on connect. (`octomap_server` masked this by
   republishing continuously as clouds streamed in.) `onOctomap` also logs `First octomap received:
   N nodes` once, and throttled warnings on deserialize/type failure — the fastest way to tell
   whether the core is actually being fed a map.

### px4_msgs / okvis (`ros2/third_party/`)

Vendored in-tree copies of upstream repos (kept as copies, **not** git submodules). `px4_msgs`
matches the PX4 v1.17 topic set. Don't hand-edit these; they mirror upstream — **except one
deliberate local patch**: `okvis2/okvis_ros2/src/Publisher.cpp` (`setBodyTransform`) no longer
broadcasts `body→camera_link` (see *Frames & TF* above). Re-apply it if you re-vendor okvis2.

## Hardware / external process dependencies

Required at runtime, referenced by path/command in the launch files:

- `MicroXRCEAgent` — ROS2 ⇄ PX4 uORB bridge, serial (`/dev/ttyUSB0`) or UDP (`8888`, SITL).
- `realsense2_camera`, `rtabmap_launch`, `foxglove_bridge`, `joy`, `tf2_ros` —
  external ROS2 packages launched from the autonomy_node launch files. (`octomap_server` is no
  longer used — RTAB-Map publishes the octomap itself; see *Map source*.)
- The OKVIS config (`ros2/third_party/okvis2/config/realsense_D435i.yaml`) and the cpu monitor are
  referenced by expanded `$HOME` paths in the launch files — update those paths if the workspace moves.
- **Map-frame consistency (verify before flying planned trajectories)**: the octomap/RTAB-Map "map"
  frame must share an origin with the controller's VIO-rooted ENU. The TF tree is patched by a
  `body→camera_link` static transform (`--pitch -1.5708 --roll 1.5708`), now folded into
  `autonomy_vision_launch.py` rather than run by hand.

### NUC resource constraints

The NUC (i5, 16GB, no GPU) is tight for VIO + SLAM + planning at once — keep this in mind before
suggesting higher resolution/rate/quality:

- Camera capped at 640×480@15fps (depth/infra/RGB) in `autonomy_vision_launch.py`.
- `okvis2/config/realsense_D435i.yaml`: `use_cnn: false` (no GPU), ≤400 keypoints, `octaves: 0`,
  2 optimization threads, 40 ms realtime budget.
- RTAB-Map tuned light (`rtabmap_viz:=false`, `--Vis/MinInliers 12`, `--Rtabmap/DetectionRate 1`);
  OctoMap at 0.05 m. Min-snap trajgen is ms-cheap; RRT* runs on its own thread (≤3 s) so it never
  stalls the 50 Hz control loop.
- `system_monitor_pkg/cpu_monitor.py` publishes `/telemetry/cpu_usage_total` and
  `/telemetry/ram_usage_total` at 2 Hz (it does not write a file; the root `cpu_log.csv` is an
  unrelated one-off `dstat` capture).
