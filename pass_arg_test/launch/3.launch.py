from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 在启动时获取命令行参数 'use_node' and 't_min'
    use_node = LaunchConfiguration('use_node') 
    t_min = LaunchConfiguration('t_min')  
    
    declear_use_node = DeclareLaunchArgument(
        'use_node',
        default_value='True',
        description='Whether to start the talker node')
    
    declear_t_min = DeclareLaunchArgument(
        't_min',
        default_value='1',
        description='How long the talker node will run')
    
    talker = Node(
                package='demo_nodes_cpp',
                executable='talker',
                output='screen',
                condition=IfCondition(use_node))

    # Convert t_min to seconds and use it as a timer
    t_min_in_seconds = PythonExpression(['float(', t_min, ') * 60'])
    timer_action = TimerAction(
        period=t_min_in_seconds, 
        actions=[Shutdown()],
        condition=IfCondition(use_node))
    
    return LaunchDescription([
        declear_use_node,
        declear_t_min,
        LogInfo(msg=[LaunchConfiguration('use_node')]),
        LogInfo(
            msg=["The talker will stop in ", t_min, " min"],
            condition=IfCondition(use_node)),
        talker,
        timer_action  # Add the timer action to the launch description
    ])