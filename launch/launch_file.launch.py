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
            executable='weight_calculator',
            output='screen'
        )
        # Node(
        #     package='som_cws_ajrp',
        #     executable='simple_sub_node',
        #     output='screen',
        # ),

        # first comment
    ])