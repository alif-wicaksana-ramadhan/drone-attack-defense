from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_controller',
            executable='attacker',
            name='sim',
            arguments=['attacker1'],
            output='screen',
        ),
        # Node(
        #     package='drone_controller',
        #     executable='attacker',
        #     name='sim',
        #     arguments=['attacker2'],
        #     output='screen',
        # ),
    ])