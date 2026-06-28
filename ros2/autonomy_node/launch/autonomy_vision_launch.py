import launch
import launch_ros.actions
import launch_ros.substitutions
import launch.actions
import os

def generate_launch_description():
    
    config_file = os.path.expandvars('$HOME/ws_paramio/src/ros2/third_party/okvis2/config/realsense_D435i.yaml')
    cpu_script_path = os.path.expandvars('$HOME/ws_paramio/src/ros2/tools/system_monitor_pkg/system_monitor_pkg/cpu_monitor.py')

    return launch.LaunchDescription([

        # Start the autonomy node (wraps the drone_core stack)
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run autonomy_node autonomy_node'
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

        # Launch OKVIS node in subscriber mode
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                f"ros2 launch okvis okvis_node_subscriber.launch.xml config_filename:={config_file}"
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
                'args:=" -d --Vis/MinInliers 12 --Rtabmap/DetectionRate 1 --Rtabmap/ImagesBufferSize 10 --Rtabmap/TimeThr 0 --Rtabmap/MemoryThr 0"'
            ],
            output='screen'
        ),

        # Launch octomap server to generate multi resolution voxel markers for foxglove
        launch.actions.ExecuteProcess(
            cmd=[
                'bash', '-c',
                'ros2 run octomap_server octomap_server_node --ros-args -p resolution:=0.05 -p frame_id:=map -r cloud_in:=/rtabmap/cloud_map'
            ],
            output='screen'
        ),

        

        # Start Foxglove WebSocket bridge for remote visualization
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
    