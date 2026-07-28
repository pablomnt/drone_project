#!/usr/bin/env bash
#
# Deep-debug capture — run BY HAND, and ONLY when chasing a specific problem.
#
# High-bandwidth / higher-CPU streams: planner visualisation, the color scene,
# depth, and the IR frame OKVIS actually consumes. Do NOT leave this running
# for normal flights — it is the heavy tier. Keep captures short.
#
# Usage:  (source your workspace first)  ./record_debug.sh   — Ctrl-C to stop.
# Bags land in ~/flight_logs (override with FLIGHT_LOG_DIR=/some/path).

set -euo pipefail

OUT_DIR="${FLIGHT_LOG_DIR:-$HOME/flight_logs}"
mkdir -p "$OUT_DIR"

STORAGE=sqlite3
PKGS="$(ros2 pkg list 2>/dev/null || true)"
if grep -qx 'rosbag2_storage_mcap' <<<"$PKGS"; then
  STORAGE=mcap
fi

TOPICS=(
  # --- planner internals (DEBUG_PLANNER_VIZ must be enabled to populate these) ---
  /planner/clearance_field                                   # EDT clearance field (PointCloud2)
  /planner/occupancy_map                                     # occupancy cloud (PointCloud2)
  /planner/search_tree                                       # RRT* tree (MarkerArray)
  /planner/corridor                                          # safe-flight corridor (MarkerArray)
  # --- camera (compressed variants; depth is the big consumer) ---
  /camera/camera/color/image_raw/compressed                 # human-viewable scene (JPEG)
  /camera/camera/aligned_depth_to_color/image_raw/compressedDepth  # depth (PNG-compressed)
  /camera/camera/infra1/image_rect_raw/compressed           # the IR frame OKVIS sees
  /camera/camera/imu                                        # ~200 Hz camera IMU
  /okvis/okvis_points_matched                               # matched landmark cloud
)

echo "DEEP-DEBUG recording ${#TOPICS[@]} heavy topics  (storage=$STORAGE)  ->  $OUT_DIR"
echo "This is high-bandwidth. Ctrl-C to stop."

# Shorter splits here since segments are large. No extra compression: the image
# topics are already compressed; only raw depth would benefit and we take the
# /compressedDepth variant above.
cd "$OUT_DIR"
exec nice -n 10 ros2 bag record \
  --storage "$STORAGE" \
  --max-bag-duration 60 \
  "${TOPICS[@]}"
