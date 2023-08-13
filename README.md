# ROS2 在多个 launch 文件之间传递参数

本文主要解决 ROS2 在启动 `bring_up.launch.py` 时给其 include 的其他 `launch` 文件传递参数，以及如何将参数与字符串拼接的问题。本文所使用的代码位于 [github](https://github.com/Alexbeast-CN/ROS2_launch_args)

首先我们创建一个基于 `ament_cmake` 构建的功能包 `pass_arg_test`

```
ros2 pkg create --build-type ament_cmake pass_arg_test
```

打开 `CMakeLists.txt` 文件，加入如下内容：

```cmake
install(
  DIRECTORY launch src include
  DESTINATION share/${PROJECT_NAME}
)
```

然后在其中创建 `launch` 文件夹，并新建 `1.launch.py` 文件，内容如下：

```py
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 在启动时获取命令行参数 'test_name'
    test_name = LaunchConfiguration('test_name') 
    
    # 在启动时声明命令行参数 'test_name'，其默认值为 'A'
    declare_test_name_cmd = DeclareLaunchArgument(
        'test_name',
        default_value='A',
        description='')

    # 获取'pass_arg_test'包的共享目录，
    bringup_dir = get_package_share_directory('pass_arg_test')

    # 将命令行参数 'test_name' 传递给 '2.launch.py'
    launch_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', '2.launch.py')),
        launch_arguments={'test_name': test_name}.items())
    
    return LaunchDescription([
        # 读取命令行参数 'test_name'
        declare_test_name_cmd,
        # 打印命令行参数 'test_name'
        LogInfo(msg=['test_name in 1.launch.py is ', LaunchConfiguration('test_name')]),
        # 运行 2.launch.py
        launch_2
    ])
```

随后，我们再创建一个 `2.launch.py` 文件，内容如下：

```py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # 在启动时获取命令行参数 'test_name'
    test_name = LaunchConfiguration('test_name')
    # 在启动时获取命令行参数 'test_name_file'
    test_name_file = LaunchConfiguration('test_name_file')

    # 在启动时声明命令行参数 'test_name'，其默认值为 'B'
    declare_test_name_cmd = DeclareLaunchArgument(
    'test_name',
    default_value='A',
    description='')

    # 在启动时声明命令行参数 'test_name_file'，其默认值为 test_name+'.yaml'
    declare_test_name_file = DeclareLaunchArgument('test_name_file', default_value=[test_name, '.yaml'])
    
    return LaunchDescription([
        # 读取命令行参数 'test_name'
        declare_test_name_cmd,
        # 读取命令行参数 'test_name_file'
        declare_test_name_file,
        # 打印命令行参数 'test_name'
        LogInfo(msg=['test_name in 2.launch.py is ', LaunchConfiguration('test_name')]),
        # 打印命令行参数 'test_name_file'
        LogInfo(msg=['test_name_file in 2.launch.py is ', LaunchConfiguration('test_name_file')]),
        ])
    
```

然后使用 `colcon build` 进行编译，` . install/setup.bash` 来 `source`，然后就可以运行 `launch` 文件了。

- 首先我们测试不传参数，单独运行 `2.launch.py` 的情况：
```
ros2 launch pass_arg_test 2.launch.py
```

得到如下结果：

```
[INFO] [launch]: All log files can be found below /home/tim/.ros/log/2023-08-13-15-03-56-861257-TIMs-4090i9-2703
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: test_name in 2.launch.py is B
[INFO] [launch.user]: test_name_file in 2.launch.py is B.yaml
```

我们可以看到 `2.launch.py` 中 `test_name` 的值为 `B`，而 `test_name_file` 参数，通过 `DeclareLaunchArgument` 中 `default_value = [test_name, '.yaml']` 的方式将 `test_name` 与字符串 `'.yaml'` 拼接了起来。最终得到了 `A.yaml`。

- 然后我们测试不传参数，单独运行 `1.launch.py` 的情况：

```
ros2 launch pass_arg_test 1.launch.py
```

得到如下结果：

```
[INFO] [launch]: All log files can be found below /home/tim/.ros/log/2023-08-13-14-51-37-415986-TIMs-4090i9-1307
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: test_name in 1.launch.py is A
[INFO] [launch.user]: test_name in 2.launch.py is A
[INFO] [launch.user]: test_name_file in 2.launch.py is A.yaml
```

可以看到，`1.launch.py` 中的 `test_name` 参数被传递给了 `2.launch.py`，因此 `1.launch.py` 中 `test_name` 的默认值 `B` 被覆盖为了 `A`。

- 接下来我们测试传递参数的情况：

```
ros2 launch pass_arg_test 1.launch.py test_name:=B
```

得到如下结果：

```
[INFO] [launch]: All log files can be found below /home/tim/.ros/log/2023-08-13-14-56-22-908586-TIMs-4090i9-1805
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: test_name in 1.launch.py is B
[INFO] [launch.user]: test_name in 2.launch.py is B
[INFO] [launch.user]: test_name_file in 2.launch.py is B.yaml
```

`1.launch.py` 接收到了来自命令行的 `test_name` 参数，将默认的 `test_name = A`  修改为了 `test_name = B`

- 最后我们再测试分别给 `test_name` 和 `test_name_file` 传递参数的情况：

```
ros2 launch pass_arg_test 1.launch.py test_name:=B test_name_file:=C.yaml
```

得到如下结果：

```
[INFO] [launch]: All log files can be found below /home/tim/.ros/log/2023-08-13-15-07-44-524786-TIMs-4090i9-3069
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: test_name in 1.launch.py is B
[INFO] [launch.user]: test_name in 2.launch.py is B
[INFO] [launch.user]: test_name_file in 2.launch.py is C.yaml
```

虽然 `2.launch.py` 中的 `test_name` 是 `B`，但 `test_name_file` 依然被命令行修改为了 `C.yaml`

> 本文参考 ROS Answer 中关于 [如果将 LaunchConfiguration 与字符串拼接](https://answers.ros.org/question/384712/ros2-launch-how-to-concatenate-launchconfiguration-with-string/) 的答案。