from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='way_finder',
            executable='joystick_teleop_node',
            name='joystick_teleop',
            output='screen'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
    ])