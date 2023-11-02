
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Launch the example.launch.py launch file."""
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('fish_cam'), 'launch'),
            '/DWEhd.launch.py'])
    )

    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('controller_node'), 'launch'),
            '/joy.launch.py'])
    )

    ''' 
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('launch_tutorial'), 'launch'),
            '/turtlesim_world_1_launch.py'])
    )
    '''
    
    return LaunchDescription([
        joy_launch,
        #imu_launch,
        camera_launch
    ])