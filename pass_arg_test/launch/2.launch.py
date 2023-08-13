from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    test_name = LaunchConfiguration('test_name')
    test_name_file = LaunchConfiguration('test_name_file')
    
    declare_test_name_cmd = DeclareLaunchArgument(
    'test_name',
    default_value='B',
    description='')
    
    declare_test_name_file = DeclareLaunchArgument('test_name_file', default_value=[test_name, '.yaml'])
    
    return LaunchDescription([
        declare_test_name_cmd,
        declare_test_name_file,
        LogInfo(msg=['test_name in 2.launch.py is ', LaunchConfiguration('test_name')]),
        LogInfo(msg=['test_name_file in 2.launch.py is ', LaunchConfiguration('test_name_file')]),
        ])
    