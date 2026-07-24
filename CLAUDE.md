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
  `goal_projection` (invalid-goal → nearest-valid-state projection; analytic clearance fields and
  0.5 s solve budgets, ~3 s total, so it also runs on every planning edit),
  `unknown_cost` (unknown-space surcharge + truncation's hard stop; analytic predicates, ~0.5 s,
  also on every planning edit), `autonomy_core` (plan→track→watchdog).
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
  EDT use per tick. Validity and cost use **separate cached `DynamicEDTOctomap`s** when a distinct
  conservative view exists: validity reads the search map's field, the cost reads the conservative
  one (see *Two fields, two questions* below). Both are rebuilt only when their map object changes
  (`clearanceField` / `conservativeField`), not per tick, and the conservative field is the same one
  truncation and corridor growth already use, so the split costs no extra distance transform.
  Clearance-aware cost `= ∫(1 + w·max(0,thresh−d)) ds`. Path post-processing: with **no**
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
  in `isPathValid`. `kGoalFlexibility` (0.3 m): solutions ending farther than this from the
  **requested** goal are **rejected** (`planPath` returns false → caller sees infinite cost) rather than
  committing to a path that ends short — whether the search stopped short or the goal was projected
  (below). All four are `static constexpr` in the class (deliberately not
  ROS params). `setClearance(fn, weight, threshold)` sets that validity field and the
  **clearance-aware cost**'s weight/threshold (obstacle-proximity integral penalty up to
  `clearance_threshold`); `isPathValid()` re-checks a committed path under the same model;
  `pathCost()` scores a waypoint list for the hysteresis gate; `minClearance()` reports a path's
  tightest clearance (diagnostic / logging).

  **Goal projection — why an invalid goal is not a refusal.** The collision check applies to *every*
  state OMPL touches, the goal included, and OMPL discards invalid goal states inside
  `PlannerInputStates::nextGoal` and then aborts (`EIT*: No solution can be found as no goal states
  are available`). So a goal within `kCollisionMargin` of a mapped obstacle used to produce **no path
  at all** rather than an approach that stops at the margin, and `BEST_EFFORT_GOAL` could not rescue
  it — best-effort accepts a search that *stopped short of a valid goal*, and here the search never
  ran. Note this is about mapped obstacles only: unknown space reads as free in the optimistic
  validity field, so a goal beyond the frontier was never the problem.
  `planPath` therefore projects the goal before solving (`projectGoal`): clamp into the state-space
  bounds, and if it still collides, walk outward in `kGoalProjectStep` (0.05 m) shells over the six
  axis directions plus a 128-point Fibonacci lattice, taking the first radius that yields a valid
  point and, among those, the candidate nearest the start — every point on the shell is equally far
  from the goal, but the one on the drone's side is the one the search can reach and cannot be a point
  through a thin wall. The axes are searched explicitly because octomap obstacles are boxes, so the
  shortest way out is usually a face normal. `kGoalProjectRadius` (2.0 m) bounds the displacement;
  past it `planPath` returns false without solving and `lastGoalProjection()` reports infinity, which
  the plan log turns into `[goal (…) is inside an obstacle: no point clearing 0.5m nearby]`. The
  radius is deliberately generous because both modes already have their own guard: strict mode
  rejects anything ending more than `kGoalFlexibility` from the requested goal, and best-effort wants
  the closest safe approach whatever the distance. `lastGoalGap()` is measured from the solution
  endpoint to the **requested** goal (not OMPL's `getSolutionDifference()`, which would measure to
  the projection), so it stays comparable with the host's own endpoint-to-goal distances in the
  best-effort ADVANCE test. A successful projection is logged as a suffix on the tick's `[plan]`
  line — the drone will deliberately stop short of the commanded point, which the operator has to be
  able to see. Covered by the fast `goal_projection` test (`ctest -R goal_projection`, ~3 s), which
  also duplicates the walled-off-goal assertions from the slow `planner` test.

  **The unknown-space surcharge — `setUnknownPenalty`.** A distance field cannot make unmapped
  space expensive, and this is structural rather than a tuning failure. The EDT saturates at its
  `maxdist`, which the host sets equal to `CLEARANCE_THRESHOLD`, so any point further than the
  threshold from a mapped obstacle scores a proximity penalty of **exactly zero** — that covers the
  interior of unmapped space and everything outside the field's bounding box (where
  `makeClearanceFn` returns the saturation distance by convention). Stamping the frontier buys a
  gradient within `CLEARANCE_THRESHOLD` of the stamped shell and nothing beyond it, and the shell is
  not reliably closed: the sensor's field of view leaves gaps, and `kFrontierKeepOutRadius`
  deliberately leaves one around the vehicle. The observed failure was the planner routing
  *backwards* out through a gap and then flying to the goal through unmapped space, because that
  route was genuinely cheaper. Raising `CLEARANCE_WEIGHT` cannot fix it — it multiplies a zero.
  The fix adds a third term to the objective, giving
  `∫(1 + w·max(0, threshold − clearance) + unknown_weight·[unknown]) ds`, where `[unknown]` is a
  **predicate on the octree** (`map->search(p) == nullptr`, i.e. no node ⇒ never observed) rather
  than a field lookup. That is O(log n), allocation-free, has no dependence on the shell's geometry,
  and is correct outside the map's bounding box. Three properties are deliberate:
  - **Flat, not a ramp.** A distance-based penalty fades out as the path gets further from the
    boundary, so a route diving deep into unmapped space eventually stops paying. A flat per-metre
    charge is paid for every metre out there, so among routes that must end in unknown space (a goal
    beyond the frontier) the search prefers the one entering latest and travelling least far inside.
  - **Cost only, never validity.** Unknown space stays *enterable*: the collision check still reads
    it as free, so `EIT*` still accepts a goal beyond the frontier and `BEST_EFFORT_GOAL` still
    chases it. Making it invalid would wall the search into the mapped region and reintroduce the
    "no goal states are available" failure. Keeping the vehicle out is truncation's job, not the
    search's.
  - **Read from the RAW map**, not the conservative one: frontier stamping writes voxels into its
    copy, and a stamped point that did not already exist would make genuinely unobserved space read
    as observed.
  - **Gated on `TREAT_FRONTIER_AS_OBSTACLE`**, carried into the core as `Config::
    treat_unknown_as_hazard` and driving truncation's stop as well, so the flag is a genuine master
    switch: with it off, nothing anywhere in the pipeline distinguishes unknown from free.
    **Gated on the flag itself, NOT on a conservative map view existing** — that was a real bug
    worth not repeating. The node only builds the conservative view once a frontier cloud has
    arrived (`TREAT_FRONTIER_AS_OBSTACLE && frontier_cloud_`), so keying off it meant a late or
    missing `/octomap_frontier` silently switched off both guards with the flag still on — visible
    only as `unk_cost` quietly vanishing from the `[plan]` line. Neither guard needs the frontier
    cloud: both read the raw octree, where a cell with no node has never been observed. The node now
    warns (throttled, naming the resolved topic and its publisher count) when the flag is on but no
    cloud has arrived, since that state is otherwise invisible.

  Admissibility is preserved (the term is ≥ 0, so the integrand stays ≥ 1 and both heuristics remain
  lower bounds). **Accuracy caveat:** OMPL integrates the objective trapezoidally over states
  interpolated at the state-validity resolution (~0.5 m for this state space), so a step-function
  surcharge is smeared across the straddling segment and the total lands within about
  `step × weight` of exact. Fine for steering the search; **not** a safety mechanism — a sliver of
  unobserved space thinner than the step can be missed entirely. `CostBreakdown` gained an
  `unknown` field so the `[plan]` line reports `len=… + clr_cost=… + unk_cost=…` (the last shown
  only when charged), because "long", "hugs a wall" and "routes through unmapped space" need
  different fixes. Covered by `ctest -R unknown_cost` (~0.5 s).

  **Truncation stops at unobserved space (when `TREAT_FRONTIER_AS_OBSTACLE` is on).** The same
  predicate is a hard stop in `truncatePath`, checked *before* the clearance test and **exempt from
  the escape ramp**. The caller supplies the predicate only when a conservative view exists and
  passes an empty one otherwise, so with the flag off truncation is exactly the clearance walk it
  always was. It is passed down as the predicate rather than as a map handle specifically so that
  decision sits at the call site beside the `conservative` handle it depends on, instead of being
  re-derived inside `runTrajgen` from which map objects happen to be distinct. This is a
  genuine hole in the clearance test rather than a refinement of it: the conservative field measures
  distance to the stamped shell, so a path leaving mapped space through a gap in that shell reads as
  high-clearance the whole way out and was committed in full — after which the corridor grew regions
  out there and the QP certified them, since the obstacle list only contains occupied and stamped
  voxels. The ramp exemption is the point: the ramp trades margin for the ability to move at all,
  which is defensible against a hazard whose distance we can measure, and unobserved space is not
  that. **This makes free cells load-bearing.** RTAB-Map's `Grid/RayTracing true` carves free space
  along each ray, `toOcTree` copies free leaves with their log-odds, and free nodes survive the
  binary octomap round-trip — so the flight volume has nodes and truncation commits normally. But a
  map carrying *only* obstacles now reads as "nothing here has ever been observed" and commits
  nothing; the `autonomy_core` test had to start writing free space for this reason. The empty-
  truncation log names which test stopped it (`path leaves observed space at the drone` vs
  `clearance below the ramped margin`), since those want opposite responses.

  **Two fields, two questions — `setCostClearance`.** The cost may be scored against a *different*
  field from the collision check, and under `USE_CORRIDOR_QP` + `TREAT_FRONTIER_AS_OBSTACLE` it is.
  The two ask different questions. Validity asks "would the vehicle hit something here", and must
  read unmapped space as free or the informed planners find no path to a goal beyond the frontier
  at all — so it always uses the **optimistic** search field, never the conservative one. Stamped
  frontier is a closed shell; validating against it would wall the search inside the mapped region.
  The cost asks "is this somewhere we would rather fly", and there the frontier is worth being
  repelled by. **The problem this fixes:** truncation cuts the committed prefix where *conservative*
  clearance drops below `FRONTIER_MARGIN`, but the cost only ever saw the *optimistic* field, so the
  search was optimising a metric blind to the criterion deciding how much of its path survived. It
  had no reason not to return paths running tangent to the frontier, which then truncated to almost
  nothing. Scoring both against the same field aligns them and the surviving prefix gets longer.
  Because the frontier is stamped as a **thin shell** of known-free voxels bordering unknown (not the
  whole unknown volume), the conservative field is `min(distance to real obstacle, distance to that
  shell)` — a strict refinement of the optimistic field, identical wherever the shell is not the
  nearest source, so no real-obstacle shaping is lost by scoring against it. The penalty saturates at
  `w·threshold` where clearance is 0, so the shell is expensive to approach but never a barrier. The
  cost-to-go heuristics stay admissible (the integrand is still ≥1). `setCostClearance` is
  order-independent with `setClearance`; passing an empty function restores single-field scoring, and
  with no distinct conservative view (frontier stamping off, or the legacy non-corridor mode where
  the search already runs on the conservative map) the override is skipped entirely.
  **Watch for:** with the goal at the frontier the cost is at its maximum exactly there, so the
  search approaches reluctantly and `BEST_EFFORT_GOAL` may settle short — `CLEARANCE_WEIGHT` is the
  knob if prefixes lengthen but the drone stops advancing.
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
  the junction into that intersection and shrinking can empty it. **The first region is the
  exception** — see the start relaxation below. `corridor_trajectory.{hpp,cpp}`: `CorridorTrajectoryOptimizer`
  solves degree-7 min-snap as an **OSQP QP** — monomial coefficients (same snap `Q`), C0–C4
  continuity, rest-to-rest ends, Bézier control points of position confined to the regions and of
  vel/acc/jerk within per-axis `VMAX/AMAX/JMAX` (hull property ⇒ the whole curve complies; solved in
  per-segment normalized time or OSQP stalls on conditioning), plus a feasibility-aware BOBYQA time
  search (infeasible ⇒ flat penalty; velocity-consistent seed grown until feasible).

  **Failure stages nothing — there is no min-snap fallback under `USE_CORRIDOR_QP`.** Any corridor
  failure (empty truncation, decomposition rejected, QP infeasible) makes `runTrajgen` return false,
  so the tracker rides out whatever it already has and latches hover-hold after `STALE_TIMEOUT`.
  There used to be a plain min-snap fallback on the truncated prefix; it was removed because min-snap
  ignores obstacles entirely, which makes it least defensible in exactly the situation that produces
  it — the corridor stage saying it cannot certify a safe trajectory. Standing still is the honest
  answer. The caller does not advance `last_trajgen_` on false, so this retries every cadence and
  recovers as soon as the map or the path allows a corridor. Each failure logs which stage failed and
  why — `truncated to N wp / L m`
  (pipeline refusing to commit toward unknown space), `decomposition FAILED (<which check>) …
  tightest conservative clearance C m vs required M m` — the parenthetical names the actual check
  that rejected it (two regions stopped overlapping, the goal fell outside the last region, or the
  drone is inside/touching an occupied or unknown cell), since the clearance figure is context, not
  the cause — or `QP INFEASIBLE over S regions` (corridor fine, no trajectory fits it within
  `VMAX/AMAX/JMAX`). These are distinct faults needing opposite fixes, hence distinct messages. A
  successful corridor logs one line only when `DEBUG_PLANNER_VIZ` is on.

  **The start relaxation — why the first region is special.** `truncatePath` ramps its requirement to
  *zero* at the drone (`ESCAPE_RAMP_DIST`) so a vehicle in a tight pocket can root a path at all. A
  convex region has no interior gradient — one plane holds over the whole region — so a uniform
  `CORRIDOR_MARGIN` next to the drone means truncation commits prefixes the corridor then refuses,
  every tick, for a drone that is merely *near* a wall. `buildCorridor` closes that gap by relaxing
  the first region, bounded in both available senses:
  - **In extent.** The first segment is split at `ESCAPE_RAMP_DIST` (`CorridorParams::start_relax_dist`,
    fed from the same ROS param), so reduced margin applies over that distance only and every later
    region carries the full `CORRIDOR_MARGIN`. The split is the *only* place an extent bound can come
    from, which is why it exists rather than just relaxing region 0 as-is (that would relax over a
    whole `MAX_SEGMENT_LEN`).
  - **In magnitude.** Region 0 is shrunk by the largest amount that still contains the drone —
    `min_r(b_r − n_r·p_drone)`, one pass over the faces, no bisection — clamped to the full shrink.
    Never more relaxation than the geometry forces, and it heals itself as the map fills in, with
    nothing to retune. Growth-window planes can never trigger it: the window is sized `+ pull_in`
    beyond what it wants to keep, so its slack always exceeds the shrink.

  The floor is `voxel_half_diagonal`, and that is geometry rather than taste: below it the region
  would contain points inside an occupied voxel's actual volume, not merely close to it. A drone
  that close fails the corridor outright with its own message. `runTrajgen` logs `start margin
  relaxed to X m` **unconditionally** (not behind `DEBUG_PLANNER_VIZ`) whenever it bites — it is the
  number that says how much protection the first stretch of the flown trajectory actually has.
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
- **Differential-flatness feed-forward (added on top; `PositionControl`'s own default is OFF, but the
  node now ships `ENABLE_FEEDFORWARD` defaulted `true`)**: `setReference()` accepts
  `vel_ff`/`acc_ff`; with feed-forward disabled the controller is byte-identical to the baseline
  (the `feedforward` unit test asserts this). Toggle live with the `ENABLE_FEEDFORWARD` parameter.
  Feed-forward is suppressed during the takeoff ramp. The accel→attitude/thrust map *is* the flatness
  output stage and already existed — that's why trajectory tracking didn't require a new controller.

  **Exactly two quantities are fed forward, at two structurally different points**, and the
  difference is the part worth remembering:
  - `vel_ff` is added to the velocity setpoint **before** the clamp and therefore **before** the
    velocity PID, so it does not bypass the loop — it raises the target the loop chases. The velocity
    error becomes `(Kp_pos·e_pos + vel_ff) − vel_meas`, which is what stops the controller needing a
    standing position error (and a wound-up integrator) just to hold speed.
  - `acc_ff` is added to the acceleration setpoint **after** the PID, so it genuinely bypasses
    feedback and goes straight into the attitude/thrust map. That is what tilts the vehicle into a
    turn when the trajectory asks, rather than after error has accumulated.

  **Not** fed forward: **jerk** (the trajectory has it and the QP pins it at the splice, but jerk maps
  to angular *rate* and the node publishes `VehicleAttitudeSetpoint` with only `q_d` + `thrust_body`,
  so there is no channel — the pinned jerk buys reference continuity, not a command) and **yaw rate**
  (`Reference` has no such field by design; `yaw_sp_move_rate` is left at zero). Three gates must all
  be open for any of it: `ENABLE_FEEDFORWARD`, tracker mode `kTracking` (the tracker forces it off in
  `kDirect`/`kHoverHold`), and `!_is_taking_off`.

`flatness_mapper` samples the trajectory for pos/vel/acc and derives **yaw from the velocity
heading** (camera leads the motion), holding the last yaw only when slow *and* the heading is
spinning. The polynomial evaluation itself lives in `common/trajectory_eval` (`evalTrajectory` /
`sampleMotion`), not in the mapper, because planning needs it too — a replan samples the trajectory
it is about to replace. `trajectory_tracker` composes the mapper + controller and runs the
**watchdog state machine**: `kDirect` (explicit setpoint — takeoff/manual hover), `kTracking`
(follow a fresh planner trajectory), `kHoverHold` (failsafe: latch current position when guidance
goes stale, i.e. the planner stalled/died). The stale fallback hovers; it does not land (PX4 rejects
offboard land).

**Mode precedence is strictly ordered, and `POS_SP` is NOT an override.** The branch order in
`TrajectoryTracker::update` is: fresh trajectory → `kTracking`; else *any* installed trajectory →
`kHoverHold`; else direct setpoint → `kDirect`. Two consequences that surprise operators:

- **Changing `POS_SP` while a goal is being tracked does nothing.** The node re-reads the parameter
  and pushes it in via `setSetpoint()` every tick, so it is accepted silently — stored, never used,
  no warning. A fresh trajectory always wins.
- **`POS_SP` is unreachable for as long as a trajectory is installed, even after guidance dies.**
  The stale case falls to `kHoverHold` (latch *current* position), not to `kDirect`. The only thing
  that restores `POS_SP` is `reset()`, i.e. a disarm or leaving offboard. (Before the splice work
  `reset()` did not clear trajectories either, so `POS_SP` was dead for the rest of the process once
  the first trajectory landed. It now clears them, so an interruption genuinely restores it.)

So `POS_SP` is a **pre-takeoff / no-goal setpoint only**. There is no "return to setpoint" or abort
path through it, and **there is no goal-cancel API at all**: `AutonomyCore::has_goal_` is set by
`setGoal` and never cleared, so once a goal is published the worker replans toward it forever. If an
in-flight abort is wanted it needs to be built — the honest options are a goal-clear that drops the
tracker back to `kDirect`, or reusing the node's existing latched `NAV_LAND` failsafe.

**Replans splice onto the outgoing trajectory; they are not rest-to-rest.** `solveQP`'s start
boundary pins position *and* velocity/acceleration/jerk to a `common::MotionState`; only the END is
at rest (which is what makes an un-replaced trajectory a safety stop). `AutonomyCore::spliceAnchor`
builds that state by sampling the **outgoing trajectory** at `t0 = now + lead` — deliberately not
from the estimator, since the reference is what the controller tracks and therefore what must be
continuous, and feeding VIO velocity / IMU accel back in would both inject estimator noise into the
boundary conditions and forgive the accumulated tracking error every replan. It falls back to rest
at the measured position when there is no outgoing trajectory (first plan) or the tracker has gone
stale into hover-hold. The committed path is re-rooted at the splice point, not the drone, so the
corridor is grown around where the trajectory actually starts (rooting at the measured position
would put the splice point outside region 0 under tracking error, making the start equality
infeasible).

The **lead is measured, not tuned**: a decaying max of observed `runTrajgen` wall times × 1.5,
clamped to [`kLeadMin` 0.04 s, `kLeadMax` 0.5 s], timed around the whole call including failures.
This works because `setTrajectory` **stages** rather than engages — the tracker holds a new
trajectory until wall-clock reaches its `t0` and keeps evaluating the old one until then. So
overshooting the lead costs nothing (bias it high), while overrunning it only starts the trajectory
slightly past its own beginning and logs a line saying so. Note the corollary: a trajectory does
**not** take effect on the tick that produced it, which is what `test_autonomy_core` and
`test_trajectory_tracker` pin.

**Two known gaps in this area — read before flying tracked trajectories:**

1. **No airborne gate: a trajectory can pre-empt the takeoff ramp.** The worker plans and stages
   while disarmed (state feed sits above the arm gate by design), and a fresh trajectory beats the
   direct setpoint. So if a goal is live when you arm, the first `stepControl` engages `kTracking`
   with a ground-level reference, and the open-loop ramp — which triggers on *reference* z > 0.5 —
   fires partway into the trajectory instead of cleanly off the ground, holds the vehicle flat and
   climbing while the reference runs away horizontally, then hands back to the PID at z > 1.0 with a
   large accumulated error. `reset()` clearing trajectories narrows this but does not close it: the
   worker re-stages within one `TRAJGEN_PERIOD`. **Needs an explicit "don't leave `kDirect` until
   takeoff completed" gate.**
2. **The plain min-snap path (`USE_CORRIDOR_QP=false`) is still rest-to-rest.** It inherits the
   splice *position* and the lead anchor, but `MinSnapTrajectory` has no start-derivative boundary,
   so engaging it while moving steps the velocity reference to zero — the exact stutter the corridor
   path was fixed for. Tolerable only because that mode is the obstacle-blind one anyway. Give
   `MinSnapTrajectory` a start-derivative boundary before relying on it.
3. **`_lim_tilt` is dead code — there is NO tilt limit.** It is assigned in the constructor (0.43 rad,
   ~25°) and by `setConstraints`, and then read nowhere. `_accelerationControl` builds `body_z`
   straight from the unbounded thrust vector with no saturation. This matters most for feed-forward:
   `acc_ff` is added to `_acc_sp` *after* the PID, so a trajectory's commanded acceleration converts
   directly into commanded tilt with nothing bounding it. Not dangerous at present limits (`AMAX`
   1.5 m/s² ⇒ atan(1.5/9.81) ≈ 8.7°), but the guard you appear to have does not exist.
4. **`setConstraints()` is never called by anything** — not the tracker, the core, or the node. So
   the limits are constructor defaults: `_lim_vel_horz` **10.0 m/s**, up 2.0, down 1.0. The comment
   at the `vel_ff` injection says it is added before the clamp "so the total commanded velocity stays
   within the configured limits", which is true but the configured limit is ~10× `VMAX`. The clamp is
   not a meaningful bound on the feed-forward as wired. Fixing 3 and 4 together means plumbing
   `VMAX`/`AMAX` into `setConstraints` from the same config the QP uses, and applying `_lim_tilt` in
   `_accelerationControl`. **Deliberately not done** — `position_control.cpp` is the flight-tested
   file and this should wait until after the first tracked flight.

**Corridor conservatism with a moving start (expect this on the bench).** The corridor bounds the
Bézier *control points*, and the low-index ones are built from the start derivatives scaled by
segment time: `b3 = 3Tv/7 + 3T²a/42 + T³j/210`. A 4 s segment multiplies start jerk by ~0.3, so an
aggressive splice state in a narrow region can push a control point outside and make the QP refuse a
curve that would actually have stayed inside. Replanning while manoeuvring hard in a tight corridor
is when this bites. Shorter `MAX_SEGMENT_LEN` weakens it; MINVO attacks it directly.

A moving start also adds one **genuinely physical** infeasibility: braking from v0 within distance d
needs `a >= v0²/(2d)` whatever the time allocation, so the seed-growth loop cannot rescue a committed
prefix shorter than the stopping distance. At `VMAX` 1.0 / `AMAX` 1.5 that is 0.33 m, so it should
not bite — but read it as "the prefix is too short to stop in", not as a tuning failure.

### autonomy_node (`ros2/autonomy_node/`)

Thin wrapper. Per 50 Hz tick: publishes the offboard heartbeat, checks arm/offboard (resets the
core on disarm or leaving offboard, re-priming takeoff), assembles an ENU `State` from the active
estimator (VIO in normal mode; PX4 odom in `USE_SIM_MODE`), feeds the core, reads back the `Command`,
applies the **yaw-drift correction** (toward PX4's yaw) and the ENU→NED/FRD conversion via
`core/common/frames`, and publishes the attitude setpoint + `ControllerDebug` telemetry. Goals
arrive on `/planner/goal` (geometry_msgs/PoseStamped, position only — yaw ignored); a `POS_SP`
parameter provides the default takeoff/hover setpoint. All PX4 message types and frame conversions
are confined to this file.

**BUG (open): `state.acc` is not rotated out of the body frame.** `onSensorCombined` takes
`SensorCombined.accelerometer_m_s2` — a **body-frame FRD** measurement — and passes it through
`frames::pxNedToEnu`, which is a static axis swap documented for *world* vectors, then subtracts
9.81 from z. There is no rotation by attitude, so it is only correct while the vehicle is level; at
20° of tilt roughly 3.4 m/s² of gravity is misattributed into the horizontal axes. It has stayed
invisible because the only consumer is the hover-thrust estimator, which uses z (nearly right at
small tilts). Fix it by rotating with the vehicle attitude before removing gravity. Relevant to more
than the estimator now: `state.acc` is an obvious-looking input for anything wanting a measured
acceleration, and it is wrong. (The trajectory splice deliberately does *not* use it — see the
control section.)

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
  map changes. **Follows the cost, not the collision check**: this samples the *conservative* field
  whenever one exists as a distinct view (see *Two fields, two questions*), so the stamped frontier
  shows up as a low-clearance shell. That is the point — it is the field the objective is scored
  against. The optimistic field the collision check uses is not published.
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
  stage the room it needs to grow regions. Since the cost is scored against the **conservative**
  field (see *Two fields, two questions*), this now also sets how hard the search is pushed off the
  *frontier*, and is therefore the main lever on how much of the path survives truncation — at the
  cost of longer detours, and of a goal at the frontier being approached more reluctantly.
- `CLEARANCE_THRESHOLD` (double, default `1.0` m) — clearance saturation / EDT maxdist.
- `UNKNOWN_WEIGHT` (double, default `10.0`) — flat extra cost charged per metre of path routed
  through never-observed space (see *The unknown-space surcharge*). Read it as "how many metres of
  detour through mapped space is one metre through unmapped space worth". **`CLEARANCE_WEIGHT`
  cannot substitute for this**: the distance field saturates at `CLEARANCE_THRESHOLD`, so unmapped
  space beyond that scores a proximity penalty of exactly zero and the weight multiplies a zero.
  Raise it if the planner still prefers leaving mapped space to detouring within it; `0` restores
  the old behaviour. Affects the cost only — unknown space stays enterable, so goals beyond the
  frontier still work. **Ignored when `TREAT_FRONTIER_AS_OBSTACLE` is false**, along with
  truncation's unobserved-space stop: that flag is the single switch for whether unmapped space is
  treated as a hazard at all.

  **Raising this defocuses the informed planners, and badly.** BIT*/AIT*/EIT* sample only where
  `‖start−x‖ + ‖x−goal‖ ≤ best_cost_so_far`, and both distances are straight-line estimates that
  know nothing about this term. Inflating the true cost without inflating the estimate inflates that
  sampling region, and once it exceeds the state-space diameter (~30 m here) the search degenerates
  to uniform sampling — the tree spreads behind the drone with almost nothing between start and
  goal. Measured on a mapped room with the goal 10 m beyond the frontier, as the fraction of tree
  nodes near the straight start→goal line: weight 0 → 100%, 0.5 → 91%, 1 → 49%, 2 → 17%, 5 → 8%,
  10 → 9%; the loss tracks `path_cost / straight_line_distance` exactly (1.00, 1.39, 1.77, 2.54,
  4.85, 8.71). **It only bites when the goal is beyond the frontier** — with the goal inside mapped
  space the optimal path pays no surcharge, cost stays near the straight-line distance, and the tree
  is unchanged even at weight 10. Which is to say it bites in the normal exploration case.
  **And it buys less than it looks like it should**: when the goal is beyond the frontier every
  candidate route ends in unknown space and pays a similar charge, so the term is close to a
  constant offset that shifts all costs without changing which route wins. It discriminates only
  between routes that differ in how many *metres* of unknown they traverse (flying out and around
  the outside of the map), which is what the `unknown_cost` test checks. Keep it low, and remember
  the exact defence against flying into unmapped space is truncation, not this.
- `DEBUG_PLANNER_VIZ` (bool, default `true`) — **single switch** for the debug planner
  visualisation (search tree, clearance field, corridor). Designed to be zero-cost when off: with it
  false the planner never extracts the OMPL tree (`getPlannerData`), the EDT is never sampled, the
  corridor snapshot is never taken, and the publish methods early-return. **Now defaulted true for
  bench validation** — turn it off for a real flight so the NUC pays nothing for it.
- `TREAT_FRONTIER_AS_OBSTACLE` (bool, **currently defaulted `false`** for corridor bench tests, so
  the decomposition is exercised against real obstacles alone; `true` is the intended flight
  setting) — stamp RTAB-Map's frontier cloud
  (`/rtabmap/octomap_global_frontier_space`, remapped in the launch) as occupied voxels into a deep
  **copy** of each incoming octomap, fed to the core as the **conservative** map view (the raw map
  stays the optimistic search view — dual-map; a small keep-out ball around the drone stays
  unstamped so it can root the search). With `USE_CORRIDOR_QP` off the search itself runs on the
  conservative view (legacy behavior); with it on, the conservative view drives truncation, corridor
  growth **and the search's cost objective** — but never its collision check, which stays optimistic
  (see *Two fields, two questions*). NOTE: on a fresh map almost everything is frontier — the drone
  is boxed in until it has scanned its surroundings.
  **This is the master switch for "is unmapped space a hazard".** It gates three things: the stamped
  shell, `UNKNOWN_WEIGHT`'s cost surcharge, and `truncatePath`'s stop at never-observed cells. Turn
  it off and the pipeline is the legacy one in this respect — unknown reads as ordinary free space
  everywhere. The last two are gated via `Config::treat_unknown_as_hazard`, set straight from this
  parameter, **not** via whether a conservative map view exists: that view needs a frontier cloud,
  and those two guards do not (they read the raw octree), so tying them to it made a missing
  `/octomap_frontier` disable them silently. The shell genuinely does need the cloud, so with the
  flag on but no cloud the node warns and you run without the shell's gradient but with both octree
  guards intact.
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
  **Also drives the corridor's start relaxation**: `buildCorridor` splits the first segment here, so
  this one number is where "the vehicle is a special case" ends for both stages. `<= 0` disables the
  relaxation too (uniform `CORRIDOR_MARGIN` everywhere, and a drone closer than that to anything
  mapped or unknown gets no corridor at all).
- `CORRIDOR_MARGIN` (double, default `0.4` m) — clearance the *corridor regions* keep from obstacles
  and unknown space. **This is a strictly harder test than the planner's `kCollisionMargin`**: the
  search validates only its centreline (and exempts a sphere at the start), while the corridor needs
  the margin over a whole 3D volume of the frontier-stamped map. Lowering it does not forfeit the
  safety guarantee — the trajectory stays provably confined to the regions, so it keeps exactly this
  clearance — but it is the clearance you actually fly with, so weigh it against the airframe's
  half-width. If `[trajgen] corridor: decomposition FAILED` reports a tightest conservative clearance
  below the required margin, this is the knob.
- `MAX_SEGMENT_LEN` (double, default `2.0` m) — corridor resample cap; one free region per piece.
  **Lowering this is the lever against convex over-conservatism**: a region spanning a long segment
  gets a plane cut across its *entire* length by one obstacle near the middle, so shorter segments
  recover free volume next to complex geometry. Costs `S`: the QP is `24·S` variables and BOBYQA
  searches in `S` dimensions. Do not lower it without pinning `CORRIDOR_BBOX` (below), or you give
  back in region width what you gained in region length. Note the first segment is additionally split
  at `ESCAPE_RAMP_DIST` for the start relaxation, so the true segment count can exceed
  `ceil(L / MAX_SEGMENT_LEN)`.
- `CORRIDOR_BBOX` (double[3], default `[1, 2, 2]`) — **minimum usable** half-extents of the
  region-growth window in the **segment-aligned** frame (component 0 = slack along the segment, 1/2 =
  lateral), not world axes. A floor, not a literal size: `buildCorridor` scales it with the longest
  segment (lateral ≥ segment length, along-track ≥ half of it, so consecutive regions overlap
  generously) and adds the margin pull-in on top, so the window planes can never eat into the volume
  the trajectory can actually use. The default is exactly what `MAX_SEGMENT_LEN = 2.0` derives, so it
  changes nothing at stock settings — it exists to **decouple the two knobs** the moment
  `MAX_SEGMENT_LEN` is lowered, since a purely derived window would narrow in proportion. Raise a
  component for wider regions, at the cost of more obstacle points per decomposition. All zeros
  restores the purely derived behaviour.

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
| true | true | false | raw | corridor QP | obstacles only — with the flag off nothing distinguishes unknown from free anywhere in the pipeline (no surcharge, no truncation stop, no stamped shell), so the corridor is collision-free against mapped obstacles but unknown space reads as free |
| true | true | true | **optimistic** (raw) validity, **conservative** cost + unknown surcharge | corridor QP | **yes** — full Stage 1: optimistic search, conservative truncation + corridor, unobserved space priced and never committed |

Notes: with `PLAN_TRAJECTORY=false` the corridor snapshot is never populated, so `/planner/corridor`
stays empty regardless of the other flags — trajgen is where truncation and corridor growth happen.
`BEST_EFFORT_GOAL` is orthogonal and matters whenever the goal is unreachable or unmapped: true
(default) commits to the closest reachable point and ratchets forward, false holds. The last row is
the only configuration that delivers the Stage 1 safety claim. Note the two `USE_CORRIDOR_QP=true`
rows never fall back to min-snap: a corridor failure stages nothing and the vehicle hovers, so an
unchecked polynomial is only reachable via the legacy `USE_CORRIDOR_QP=false` rows.

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
