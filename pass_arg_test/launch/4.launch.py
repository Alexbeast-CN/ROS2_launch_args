from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get command line arguments 'add_param' and 't_min' at launch time
    add_param = LaunchConfiguration('add_param') 
    declare_add_param = DeclareLaunchArgument(
        'add_param',
        default_value='False',
        description='Whether to start the talker node')
    
    params = [1, 2]

    def append_params(context: LaunchContext):
        if context.launch_configurations['add_param'] == 'True':
            params.append(3)
        print('params: ', params)
        return [LogInfo(msg=[str(params)])]  # Print the list params

    append_params_action = OpaqueFunction(function=append_params)

    return LaunchDescription([
        declare_add_param,
        append_params_action,
    ])