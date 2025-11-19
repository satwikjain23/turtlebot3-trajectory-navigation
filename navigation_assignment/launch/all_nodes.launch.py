from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package="navigation_assignment",
            executable="path_smoothing_node",
            output="screen"
        ),
        Node(
            package="navigation_assignment",
            executable="trajectory_generator_node",
            output="screen"
        ),
        Node(
            package="navigation_assignment",
            executable="controller_node",
            output="screen"
        ),
        Node(
            package="navigation_assignment",
            executable="trajectory_visualizer",
            output="screen"
        )
    ])

