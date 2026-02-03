from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    quad_acc_control = Node(
        package='px4ctrl_pkg',
        executable='quad_velocity_py_node',
        name='quad_acc_control'
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.15
        }]
    )

    uxrce_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/ttyUSB0', '-b', '921600'],
        output='screen'
    )

    return LaunchDescription([
        quad_acc_control,
        #joy_node,
        uxrce_agent
    ])