#!/usr/bin/env bash
#
# Flight recorder — run BY HAND when you decide to start capturing a flight.
#
# Curated, low-CPU topic set: controller telemetry, VIO estimate, the PX4
# command I/O, the committed plan, and the "what the drone sees" view
# (/okvis/cam0_matches/compressed, already JPEG-compressed so the CPU never
# re-encodes a raw frame). Raw images and pointclouds are deliberately left
# out here — see record_debug.sh for those.
#
# Usage:
#   ros2 ... (source your workspace first), then:
#   ./record_flight.sh
#   Ctrl-C to stop.
#
# Bags land in ~/flight_logs (override with FLIGHT_LOG_DIR=/some/path).
# Playback is the usual `ros2 bag play <bag_folder>` — no special flags.

set -euo pipefail

OUT_DIR="${FLIGHT_LOG_DIR:-$HOME/flight_logs}"
mkdir -p "$OUT_DIR"

# Prefer mcap (lower per-message write overhead) if the plugin is installed;
# otherwise fall back to the default sqlite3. Recording is the only thing that
# differs — `ros2 bag play` auto-detects the format either way.
# (Install mcap once with: sudo apt install ros-humble-rosbag2-storage-mcap)
STORAGE=sqlite3
PKGS="$(ros2 pkg list 2>/dev/null || true)"
if grep -qx 'rosbag2_storage_mcap' <<<"$PKGS"; then
  STORAGE=mcap
fi

TOPICS=(
  /debug/telemetry                        # 50 Hz controller log: state, setpoints, PID terms
  /okvis/cam0_matches/compressed          # what the drone sees: VIO feature-match overlay (JPEG)
  /okvis/okvis_odometry                   # VIO pose estimate
  /fmu/in/offboard_control_mode           # what we command PX4
  /fmu/in/vehicle_command                 #   "
  /fmu/in/vehicle_attitude_setpoint_v1    #   "
  /fmu/out/vehicle_status                 # arm / nav_state / offboard transitions
  /smooth_trajectory                      # committed trajectory (nav_msgs/Path)
  /planner/geometric_path                 # planner intent
  /planner/goal_marker                    #   "
  /rtabmap/octomap_binary                 # world model the planner actually saw
  /telemetry/cpu_usage_total              # onboard CPU load
  /rosout                                 # console log of every ROS node (rclcpp logs; see note below)
  /tf
  /tf_static
)

echo "Recording ${#TOPICS[@]} topics  (storage=$STORAGE)  ->  $OUT_DIR"
echo "Ctrl-C to stop."

# nice -n 10  : a disk flush can never steal cycles from the flight stack.
# --max-bag-duration 120 : split every 2 min, so a crash costs one segment.
#
# If `ros2 bag info` later shows /fmu/out/vehicle_status with 0 messages
# (best-effort QoS mismatch), add this flag to the command below:
#   --qos-profile-overrides-path "$(dirname "$(readlink -f "$0")")/qos_overrides.yaml"
cd "$OUT_DIR"
exec nice -n 10 ros2 bag record \
  --storage "$STORAGE" \
  --max-bag-duration 120 \
  "${TOPICS[@]}"
