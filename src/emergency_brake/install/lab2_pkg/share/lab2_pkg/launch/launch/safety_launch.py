from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'lab2_pkg',
            executable = 'safety_node.py',
            name = 'safety_node',
            output='screen',
            parameters=[
                {'reaction_time': 0.5},
                {'safety_distance':1.0}
            ]
        )
    ])