from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='som_cws_ajrp',
            executable='coordinate_publisher',
            output='screen'
        ),
        Node(
            package='som_cws_ajrp',
            executable='distance_calculator',
            output='screen'
        ),
        Node(
            package='som_cws_ajrp',
            executable='tsp_solver',
            output='screen',
        )
    ])