import launch
import launch_ros.actions
import launch_ros.substitutions
import launch.actions
import os

def generate_launch_description():
    # Rutas dinámicas basadas en las variables de entorno del NUC
    config_file = os.path.expandvars('$HOME/ws_paramio/src/okvis2/config/realsense_D435i.yaml')
    cpu_script_path = os.path.expandvars('$HOME/ws_paramio/src/system_monitor_pkg/system_monitor_pkg/cpu_monitor.py')
    
    return launch.LaunchDescription([
        # 1. Controlador de posición (Terminal propia)
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                'ros2 run position_controller position_controller_node'
            ],
            output='screen'
        ),

        # 2. Agente MicroXRCE para comunicación con la Pixhawk (Terminal propia)
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-hold', '-e', 'bash', '-lc', 'MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'],
            output='screen'
        ),

        # 3. Driver de la Cámara RealSense D435i configurado (Terminal propia)
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
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
                'align_depth.enable:=true '
                'depth_module.emitter_enabled:=0'
            ],
            output='screen'
        ),

        # 4. OKVIS en Modo Suscriptor (Sustituye al nodo Realsense directo anterior)
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                f"ros2 launch okvis okvis_node_subscriber.launch.xml config_filename:={config_file}"
            ],
            output='screen'
        ),

        # 7. RTAB-Map optimizado para ejecución en segundo plano sin GUI (Terminal propia)
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                'ros2 launch rtabmap_launch rtabmap.launch.py '
                'visual_odometry:=false '
                'rtabmap_viz:=false '
                'rgb_topic:=/camera/camera/color/image_raw '
                'depth_topic:=/camera/camera/aligned_depth_to_color/image_raw '
                'camera_info_topic:=/camera/camera/color/camera_info '
                'frame_id:=camera_link '
                'approx_sync:=true '
                'publish_tf_odom:=false '
                'odom_topic:=/okvis/okvis_odometry '
                'args:="-d --Vis/MinInliers 12 --Rtabmap/DetectionRate 1 --Rtabmap/ImagesBufferSize 10 --Rtabmap/TimeThr 1.5"'
            ],
            output='screen'
        ),

        # 6. Transformada Estática para alineación de ejes (Terminal propia)
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                'ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch -1.5708 --roll 1.5708 --frame-id body --child-frame-id camera_link'
            ],
            output='screen'
        ),

        # 7. Nodo del Joystick / Mando (Terminal propia)
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                'ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.15'
            ],
            output='screen'
        ),

        # 8. Monitor de Recursos de CPU/RAM del NUC (Terminal propia)
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                f'{cpu_script_path}'
            ],
            output='screen'
        )
        
    ])