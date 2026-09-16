import launch
import launch_ros.actions
import launch_ros.substitutions
import launch.actions
import launch.substitutions
import os

def generate_launch_description():

    config_file = os.path.expandvars('$HOME/ws_paramio/src/ros2/third_party/okvis2/config/realsense_D435i.yaml')
    cpu_script_path = os.path.expandvars('$HOME/ws_paramio/src/ros2/tools/system_monitor_pkg/system_monitor_pkg/cpu_monitor.py')

    return launch.LaunchDescription([

        # The only RViz in this stack is the one okvis's own launch file starts for
        # its VIO mesh/trajectory view; both okvis launch XMLs gate that node on an
        # `rviz` arg we simply never passed. Forward it so it can be turned off on
        # the NUC, where RViz is a meaningful share of the CPU budget and the same
        # data is already reachable over the Foxglove bridge from another machine.
        # Defaults true so the launch behaves exactly as before unless asked:
        #     ros2 launch autonomy_node autonomy_vision_launch.py rviz:=false
        launch.actions.DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description="Start okvis's RViz view. Set false to save CPU on the NUC."
        ),

        # Start the autonomy node (wraps the drone_core stack).
        # The map now comes from RTAB-Map's own ray-traced octomap (see the rtabmap
        # block below) rather than a standalone octomap_server, so remap the node's
        # /octomap_binary subscription onto /rtabmap/octomap_binary. The frontier
        # (known-free/unknown boundary) subscription is remapped onto RTAB-Map's
        # octomap_global_frontier_space, which the node burns into the map as
        # obstacles when TREAT_FRONTIER_AS_OBSTACLE is on.
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run autonomy_node autonomy_node '
                '--ros-args -r /octomap_binary:=/rtabmap/octomap_binary '
                '-r /octomap_frontier:=/rtabmap/octomap_global_frontier_space'
            ],
            output='screen'
        ),

        # Static transform from the body frame to the camera (corrects the TF tree).
        launch.actions.ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '-1.5708', '--roll', '1.5708',
                '--frame-id', 'body', '--child-frame-id', 'camera_link'
            ],
            output='screen'
        ),

        # Start the MicroXRCE agent for Pixhawk communication
        launch.actions.ExecuteProcess(
            cmd=['bash', '-c', 'MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'],
            output='screen'
        ),

        # Launch the RealSense camera driver
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 launch realsense2_camera rs_launch.py '
                'enable_sync:=true '
                'unite_imu_method:=2 '
                'depth_module.depth_profile:=640x480x15 '
                'depth_module.infra_profile:=640x480x15 '
                'rgb_camera.color_profile:=640x480x15 '
                'depth_module.enable_depth:=true '
                'enable_infra1:=true '
                'enable_infra2:=true '
                'enable_accel:=true '
                'enable_gyro:=true '
                'global_time_enabled:=true '
                'align_depth.enable:=true'
            ],
            output='screen'
        ),

        # Disable the RealSense emitter dynamically after booting
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch.actions.ExecuteProcess(
                    cmd=[
                        'ros2', 'param', 'set', '/camera/camera', 
                        'depth_module.emitter_enabled', '0'
                    ],
                    output='screen'
                )
            ]
        ),

        # Launch OKVIS node in subscriber mode.
        # OKVIS logs through glog (the 'I0629 ...' lines: pose-init, RANSAC, large
        # reprojection error), which floods the terminal. GLOG_minloglevel=1 drops
        # its INFO chatter while keeping warnings/errors, so the planning logs stay
        # readable. This launch also owns the stack's only rviz2 (for the VIO
        # mesh/trajectory view), gated on the `rviz` launch argument above.
        #
        # The command is built as a LIST of substitution fragments rather than an
        # f-string: `rviz` is a LaunchConfiguration resolved at launch time, not a
        # Python value, so it cannot be interpolated into a string here.
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                [
                    'GLOG_minloglevel=2 ros2 launch okvis okvis_node_subscriber.launch.xml ',
                    f'config_filename:={config_file} ',
                    'rviz:=', launch.substitutions.LaunchConfiguration('rviz'),
                ]
            ],
            output='screen'
        ),

        # Launch RTAB map optimized for background execution
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 launch rtabmap_launch rtabmap.launch.py '
                'visual_odometry:=false '
                'rtabmap_viz:=false '
                # Quiet rtabmap's per-update INFO stats so the terminal can focus on
                # planning logs; warnings/errors still print.
                'log_level:=warn '
                'rgb_topic:=/camera/camera/color/image_raw '
                'depth_topic:=/camera/camera/aligned_depth_to_color/image_raw '
                'camera_info_topic:=/camera/camera/color/camera_info '
                'frame_id:=camera_link '
                'approx_sync:=true '
                'publish_tf_odom:=false '
                'odom_topic:=/okvis/okvis_odometry '
                # Block on TF lookups for up to 3s instead of the 0.2s default. /tf_static is
                # latched, so once the body->camera_link static publisher is up the transform is
                # available for all timestamps; waiting removes the startup race that otherwise
                # placed the first clouds ~90 deg off (no map reset needed).
                'wait_for_transform:=3.0 '
                # RTAB-Map builds its own 3D occupancy octomap and publishes
                # /rtabmap/octomap_binary (it links liboctomap). RayTracing is the key
                # flip: each scan carves free space along the rays from the sensor
                # origin, so voxels seen empty decay back to free instead of
                # accumulating forever (the old octomap_server-on-cloud_map setup could
                # never clear). The density knobs restore something close to that old
                # dense map (rtabmap's grid is otherwise decimated + ground-segmented,
                # so it looked sparse):
                #   DepthDecimation 1          - full-res depth (default 4 was sparse)
                #   NormalsSegmentation false  - keep ALL points as obstacles instead of
                #       discarding "ground"; for a flying drone the floor IS an obstacle
                # NoiseFiltering drops isolated speckle: a voxel needs >=5 neighbours
                #   within 0.10 m to survive, so lone specks vanish but surfaces (many
                #   neighbours) stay. The RADIUS is what keeps this gentle - an earlier
                #   0.05 m radius with the same neighbour count was too strict in the
                #   smaller sphere and stripped legitimate voxels, making the map sparse.
                # RangeMax trades look-ahead vs. far D435i depth noise (8 m is well into
                #   the noisy range; lower it toward ~3 m for less speckle at the source).
                #   CellSize matches the old 0.05 m octomap_server resolution.
                'args:=" -d --Vis/MinInliers 12 --Rtabmap/DetectionRate 1 --Rtabmap/ImagesBufferSize 10 --Rtabmap/TimeThr 0 --Rtabmap/MemoryThr 0 --RGBD/OptimizeMaxError 5.0 '
                '--Grid/3D true --Grid/RayTracing true --Grid/CellSize 0.05 --Grid/RangeMax 8.0 '
                '--Grid/DepthDecimation 1 --Grid/NormalsSegmentation false '
                '--Grid/NoiseFilteringRadius 0.1 --Grid/NoiseFilteringMinNeighbors 5"'
            ],
            output='screen'
        ),

        # (Removed) The standalone octomap_server that voxelized /rtabmap/cloud_map.
        # RTAB-Map now publishes the occupancy octomap directly (Grid/* args above), so
        # this process is redundant; it also could never clear stale voxels because an
        # assembled cloud carries no sensor origin to ray-trace from. Point Foxglove at
        # /rtabmap/octomap_binary for the voxel view.



        # Start Foxglove WebSocket bridge for remote visualization.
        # NOTE: do NOT append `--log-level WARN` here — `ros2 launch` (Humble) has
        # no such option, so it exits with "unrecognized arguments" and the bridge
        # never comes up. --log-level is a node arg (--ros-args --log-level), which
        # this launch XML does not forward, so the bridge's per-topic 'Advertising
        # new channel' INFO chatter is accepted as the cost of using the launch file.
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765'
            ],
            output='screen'
        ),

        # Start the joystick node
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.15'
            ],
            output='screen'
        ),

        # Start the CPU and memory resource monitor
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                f'{cpu_script_path}'
            ],
            output='screen'
        )

    ])
    