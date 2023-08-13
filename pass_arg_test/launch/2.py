from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    test_name = LaunchConfiguration('test_name')
    test_name_file = LaunchConfiguration('test_name_file')
    
    declare_test_name_file = DeclareLaunchArgument('test_name_file', default_value=[test_name, '.yaml'])
    
    return LaunchDescription([
        declare_test_name_file,
        LogInfo(msg=LaunchConfiguration('test_name_file'))
        ])
    