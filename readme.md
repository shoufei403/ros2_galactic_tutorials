## 仓库内容

demos 文件夹是官方的demo程序。目前的版本是galactic。

examples 文件夹是官方的示例代码。优先参考这个文件夹来写代码。目前的版本是galactic。

test_pkg 是使用下面的命令生成的

```
ros2 pkg create test_pkg --node-name test_cpp --dependencies rclcpp std_msgs
```

turtlesim 是小乌龟的实现代码。目前的版本是galactic。

tutorial_interfaces 是接口定义的示例代码。

turtlesim_controller 是作业代码。后面会随着课程不断完善。大家有实现新的功能可以提PR。

## 编译

在用户工作空间的src目录下克隆代码：

```
mkdir -p galactic_ws/src
cd galactic_ws/src
```

```
git clone https://gitee.com/shoufei403/tn_ros2_tutorials.git
```
解决依赖  
在galactic_ws目录下运行下面命令
```
rosdep install --from-paths src --ignore-src --rosdistro galactic -yr
```

编译代码(需在src的上级目录下编译)  
先编译tutorial_interfaces包，因为有些包对它有依赖。

```
colcon build --symlink-install --packages-select tutorial_interfaces
source install/setup.bash
```

再编译其他包

```
colcon build --symlink-install
source install/setup.bash
```

注意：turtlesim 包目前不会编译，因为放了COLCON_IGNORE文件在这个包里。这样做是因为系统里已经安装了这个包。这里只是用来做代码参考。

## 运行turtlesim_controller示例
请参考turtlesim_controller目录下的[readme.md](turtlesim_controller/readme.md)文件。

## 官方示例代码仓库的地址

主要方便后续的更新。

examples

```
git clone https://ghproxy.com/https://github.com/ros2/examples -b galactic
```

demos

```
https://ghproxy.com/https://github.com/ros2/demos.git
```

