from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fish_cam',
            namespace='front_cam',
            executable='DWE_exploreHD_pub',
            name='DWE_exploreHD_pub'
        ),
    ])