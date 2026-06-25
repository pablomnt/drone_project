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
  library, not `octomap_msgs`), OMPL and NLopt. All guidance and control logic lives here. It carries
  a `COLCON_IGNORE` so colcon never tries to build it.
- **`ros2/` — thin ROS wrappers + vendored third-party packages.** The colcon/ament side. The one
  first-party node (`autonomy_node`) does only message↔core-type translation, frame conversions and
  PX4 I/O; the heavy third-party packages (`okvis`, `px4_msgs`) live under `ros2/third_party/` as
  in-tree copies of their upstream repos.

```
src/
├── core/                       # ROS-FREE autonomy core (plain CMake, COLCON_IGNORE)
│   ├── common/                 # types.hpp, frames.{hpp,cpp}, logging.hpp
│   ├── control/                # position_control, flatness_mapper, trajectory_tracker
│   ├── planning/               # rrt_star_planner, min_snap_trajectory
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
planning; finished trajectories are handed over by an atomic swap.

### `common/`

Shared plain types (`State`, `Reference`, `Command`, `Trajectory`, `Goal`, `MapHandle`) and
`frames`, which holds *all* ENU↔NED/FRD and OKVIS-yaw conversions — extracted out of the old node so
they are finally unit-testable. `logging.hpp` provides `DRONE_LOG_*` macros that replace ROS logging
inside the core.

### `planning/`

- `RrtStarPlanner` — OMPL SE(3) RRT* over an octree (X/Y ±15 m, Z 0.3–2.5 m, 0.4 m obstacle
  inflation, 3 s solve budget).
- `MinSnapTrajectory` / `MinSnapTimeOptimizer` — KKT minimum-snap solve plus an NLopt BOBYQA outer
  loop for per-segment time allocation ("mode A" of ETH Zurich's `mav_trajectory_generation`). The
  optimizer keeps the actual per-segment polynomial coefficients + segment times (rather than
  re-sampling to points) so the flatness mapper can differentiate it analytically. It currently
  ignores obstacles between waypoints; corridor/collision-aware trajectory checking is future work.

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
via `core/common/frames`, and publishes the attitude setpoint plus `ControllerDebug` telemetry. Goals
arrive on `/planner/goal`; a `POS_SP` parameter provides the default takeoff/hover setpoint. All PX4
message types and frame conversions are confined to this file. Launch files live in `launch/`:
`autonomy_vision_launch.py` (real flight — full camera/VIO/mapping stack + Foxglove + the
`body→camera_link` static TF), `autonomy_launch.py` (xterm-per-process variant) and
`autonomy_sim_launch.py` (SITL: `USE_SIM_MODE:=true`, `MicroXRCEAgent udp4`).

### `drone_interfaces/`

Custom message definitions (`ControllerDebug.msg`). Message generation has to stay ROS-side.

### `third_party/`

Vendored in-tree copies of upstream repos (kept as copies, not git submodules): `okvis2` (package
name `okvis`, a large VIO library with its own `external/` deps) and `px4_msgs` (matching the PX4
v1.17 topic set). Don't hand-edit these; they mirror upstream.

### `sim/`, `tools/`

`pc_publisher` is a sim-only static `.pcd` publisher; `system_monitor_pkg/cpu_monitor.py` publishes
`/telemetry/cpu_usage_total` and `/telemetry/ram_usage_total` at 2 Hz.

## Runtime pipeline

```
RealSense → OKVIS2 (VIO: /okvis/okvis_odometry) → RTAB-Map (/rtabmap/cloud_map)
         → octomap_server (/octomap_binary)
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

## Build

Two steps: build+install the standalone core, then colcon-build the ROS side pointing at it.

```bash
# 1. Build + install the ROS-free core (plain CMake). Do this with ROS NOT sourced to prove it.
cmake -S src/core -B build/core -DCMAKE_BUILD_TYPE=Release
cmake --build build/core
cmake --install build/core --prefix install_core    # any prefix; goes on CMAKE_PREFIX_PATH below

# 2. Build the ROS wrappers, pointing find_package(drone_core) at the core install.
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_NN=OFF \
  -DCMAKE_PREFIX_PATH=$PWD/install_core
source install/setup.bash
```

`autonomy_node` links the core's static libs, so there is no runtime dependency on the core install —
only a build-time one via `CMAKE_PREFIX_PATH`. `-DUSE_NN=OFF` is required (okvis2 defaults it on,
which needs LibTorch); `-DCMAKE_BUILD_TYPE=Release` is strongly wanted (VIO+SLAM are too slow on the
NUC un-optimized).

## Test

```bash
# Core unit tests, no ROS — the proof the core is ROS-free.
cmake -S src/core -B build/core -DDRONE_CORE_BUILD_TESTS=ON
cmake --build build/core
ctest --test-dir build/core --output-on-failure
```

Tests (plain CTest, no gtest): `frames`, `position_control`, `feedforward` (proves feed-forward
OFF ≡ baseline), `flatness_mapper`, `planner`, `autonomy_core` (plan→track→watchdog).

## Deployment notes

The onboard computer is an Intel NUC (i5, 16 GB RAM, no GPU), which is tight for VIO + SLAM +
planning at once — the camera is capped at 640×480@15fps, OKVIS runs CNN-less with ≤400 keypoints,
RTAB-Map is tuned light, and RRT* runs on its own thread (≤3 s) so it never stalls the 50 Hz control
loop. The operator drives the NUC over SSH/tmux with the laptop and NUC on a shared phone hotspot,
and uses Foxglove (`foxglove_bridge`, port 8765) for remote visualization. Before flying any planned
trajectory, verify map-frame consistency: the octomap/RTAB-Map "map" frame must share an origin with
the controller's VIO-rooted ENU (the TF tree is patched by the `body→camera_link` static transform
folded into `autonomy_vision_launch.py`).
