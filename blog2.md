#! https://zhuanlan.zhihu.com/p/671323402
# ROS2 launch.py 中 LaunchConfiguration 的使用方法

## 1. 传参

在 ROS2 的 launch 中，我们经常需要用到传参的形式来确定一些节点或者逻辑是否应该执行。在 ROS2 中，传参往往需要用到 `LaunchConfiguration()`，比如下面的代码中，

```py
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 在启动时获取命令行参数 'use_node'
    use_node = LaunchConfiguration('use_node') 
    
    return LaunchDescription([
        LogInfo(msg=[LaunchConfiguration('use_node')]),
    ])
```

我们可以使用 `ros2 launch xxx.launch.py use_node:=True` 来将 `True`` 这个值传到 'use_node' 这个配置参数中去。 执行这个文件，我们就可以得到打印

```
[INFO] [launch.user]: True 
``` 

在使用 `LaunchConfiguration()` 的时候，我们往往会与 `DeclareLaunchArgument()` 一起使用，以给定相关参数一个默认值，防止用户在执行 launch 文件的时候没有指定参数而报错。比如：

```py
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 在启动时获取命令行参数 'use_node'
    use_node = LaunchConfiguration('use_node') 
    declare_use_node = DeclareLaunchArgument(
        'use_node',
        default_value='True',
        description='Whether to start the talker node')

    return LaunchDescription([
        declare_use_node,
        LogInfo(msg=[LaunchConfiguration('use_node')]),
    ])
```

## 2. Ifcondition 的使用方法

但像 `use_node` 这样的 `LaunchConfiguration()` 参数与普通的 python 变量不同，它不能直接参与 python 的逻辑运算中，因为在执行 `return LaunchDescription()` 之前，`use_node` 变量都只是一个占位符，一个空指针。比如我们试着打印：`print(LaunchConfiguration('use_node'))`  得到的是像这样的结果：

```
<launch.substitutions.launch_configuration.LaunchConfiguration object at 0x7f12a380c130>  
```

为了能够使用 LaunchConfiguration() 的变量作逻辑判断以确定能否要启动某个节点，我们必须在 Node 中使用 Ifcondition()，比如：

```py
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # 在启动时获取命令行参数 'test_name'
    use_node = LaunchConfiguration('use_node') 
    declare_use_node = DeclareLaunchArgument(
        'use_node',
        default_value='True',
        description='Whether to start the talker node')

    talker = Node(
                package='demo_nodes_cpp',
                executable='talker',
                output='screen',
                condition=IfCondition(use_node))
    
    return LaunchDescription([
        declare_use_node,
        LogInfo(msg=[LaunchConfiguration('use_node')]),
        talker
    ])
```

启动程序 `ros2 launch xxx.launch.py use_node:=True` 就会看到类似以下的输出结果了：

```
[INFO] [launch]: All log files can be found below /home/tim/.ros/log/2023-12-08-23-58-41-827081-TIMs-4090i9-3077
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: True
[INFO] [talker-1]: process started with pid [3078]
[talker-1] [INFO] [1702051122.901982148] [talker]: Publishing: 'Hello World: 1'
[talker-1] [INFO] [1702051123.901958012] [talker]: Publishing: 'Hello World: 2'
```

反之，如果你使用 use_node:=True 则没有 talk节点启动 。

## 3. 将 launch_configurations 转为常规的 python 变量

如果你所传递的参数需要用到 python 来进行更加复杂的计算，或者逻辑判断。那么，你需要利用 launch.actions 中的 `PythonExpression` 来实现了。比如，下面的例子，我们希望用户输入 launch 文件的执行时间（分钟），然后在设置定时器将程序在给定时间后杀死。由于杀死程序的指令需要的输入单位为秒，所以我们需要使用 python 进行一个简单的换算：

```py
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
```

这里我们将常规的 python 语句 `t_min_in_seconds=float(t_min)*60` 用 `str` 的形式传给 `PythonExpression`，就实现了将 launch_configurations 转为常规的 python 变量的功能。执行这个 launch 文件，并传入参数 `t_min:=0.1` 我们可以看到类似下面的输出结果，程序在 6s 后自动停止：

```
[INFO] [launch]: All log files can be found below /home/tim/.ros/log/2023-12-09-15-16-27-626107-TIMs-4090i9-3658
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: True
[INFO] [launch.user]: The talker will stop in 0.1 min
[INFO] [talker-1]: process started with pid [3659]
[talker-1] [INFO] [1702106188.685729711] [talker]: Publishing: 'Hello World: 1'
[talker-1] [INFO] [1702106189.685910887] [talker]: Publishing: 'Hello World: 2'
[talker-1] [INFO] [1702106190.685919767] [talker]: Publishing: 'Hello World: 3'
[talker-1] [INFO] [1702106191.685829454] [talker]: Publishing: 'Hello World: 4'
[talker-1] [INFO] [1702106192.685931620] [talker]: Publishing: 'Hello World: 5'
[INFO] [talker-1]: sending signal 'SIGINT' to process[talker-1]
[talker-1] [INFO] [1702106193.684573551] [rclcpp]: signal_handler(signum=2)
[INFO] [talker-1]: process has finished cleanly [pid 3659]
```

## 4. OpaqueFunction 的使用

如果你不想在每一个操作后面都添加一个 `condition=IfCondition(use_node)`，那么你可以使用 `OpaqueFunction` 来实现。比如下面的例子，我们将原本写在 return 里的 actions 都放在 `run_node_action()` 中，并且只有当 `use_node:=True` 的时候才会执行：

```py
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
```


相关文章推荐：

[ROS2 在多个 launch 文件之间传递参数](https://zhuanlan.zhihu.com/p/649741715)