import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Expand the $HOME variable to get the absolute path to the config file
    config_file = os.path.expandvars('$HOME/ws_paramio/src/okvis2/config/realsense_D435i.yaml')

    return LaunchDescription([
        # 1. MicroXRCEAgent (External Process)
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyUSB0', '-b', '921600'],
            output='screen'
        ),

        # 2. Joy Node (Native ROS 2 Node with parameters)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.15
            }]
        ),

        # 3. Position Controller Node (Native ROS 2 Node)
        Node(
            package='position_controller',
            executable='position_controller_node',
            name='position_controller_node',
            output='screen'
        ),

        # 4. OKVIS Launch File (Included from the 'okvis' package)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('okvis'),
                    'launch', # Assuming the xml file is installed in the package's 'launch' directory
                    'okvis_node_realsense.launch.xml'
                )
            ),
            # Pass the configuration file path as a launch argument
            launch_arguments={'config_filename': config_file}.items()
        )
    ])