from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get command line arguments 'use_node' and 't_min' at launch time
    use_node = LaunchConfiguration('use_node') 
    t_min = LaunchConfiguration('t_min')
    
    declare_use_node = DeclareLaunchArgument(
        'use_node',
        default_value='False',
        description='Whether to start the talker node')
    declare_t_min = DeclareLaunchArgument(
        't_min',
        default_value='0.1',
        description='How long the talker node will run')
    
    def run_node_action(context: LaunchContext):
        if context.launch_configurations['use_node'] == 'True':
            talker = Node(
                package='demo_nodes_cpp',
                executable='talker',
                output='screen'
            )
            log1 = LogInfo(msg=["The talker will stop in ", t_min, " min"])
            # Convert t_min to seconds and use it as a timer
            t_min_in_seconds = PythonExpression(['float(', t_min, ') * 60'])
            timer_action = TimerAction(period=t_min_in_seconds, actions=[Shutdown()])
            return [log1, talker, timer_action]
        
        return [] # Return an empty list if use_node is False

    run_node_action_func = OpaqueFunction(function=run_node_action)

    return LaunchDescription([
        declare_use_node,
        declare_t_min,
        run_node_action_func,
    ])