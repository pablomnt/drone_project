import launch
import launch_ros.actions
import launch_ros.substitutions
import launch.actions
import os

def generate_launch_description():
    # Get the config file path
    config_file = os.path.expandvars('$HOME/ws_paramio/src/ros2/third_party/okvis2/config/realsense_D435i.yaml')
    cpu_script_path = os.path.expandvars('$HOME/ws_paramio/src/ros2/tools/system_monitor_pkg/system_monitor_pkg/cpu_monitor.py')
    return launch.LaunchDescription([
        # Launch the autonomy node in a new terminal
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                'ros2 run autonomy_node autonomy_node'
            ],
            output='screen'
        ),

        # Launch MicroXRCEAgent in a new terminal
        launch.actions.ExecuteProcess(
            cmd=['xterm', '-hold', '-e', 'bash', '-lc', 'MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600'],
            output='screen'
        ),

        # Launch the okvis node in a new terminal
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                f"ros2 launch okvis okvis_node_realsense.launch.xml config_filename:={config_file}"
            ],
            output='screen'
        ),

        # Launch the joystick node in a new terminal
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                'ros2 run joy joy_node --ros-args -p dev:=/dev/input/js0 -p deadzone:=0.15'
            ],
            output='screen'
        ),

        # Launch the CPU/RAM Monitor in a new terminal
        launch.actions.ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e', 'bash', '-lc',
                f'{cpu_script_path}'
            ],
            output='screen'
        )
        
    ])
