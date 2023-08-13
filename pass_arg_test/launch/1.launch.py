import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    test_name = LaunchConfiguration('test_name')
    
    declare_test_name_cmd = DeclareLaunchArgument(
        'test_name',
        default_value='A',
        description='')
    
    bringup_dir = get_package_share_directory('pass_arg_test')
    
    launch_2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', '2.py')),
            launch_arguments={'test_name': test_name}.items())
    
    return LaunchDescription([
        declare_test_name_cmd,
        LogInfo(msg=['test_name in 1.launch.py is ', LaunchConfiguration('test_name')]),
        launch_2
    ])