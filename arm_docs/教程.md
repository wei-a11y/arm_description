# 基于 ROS 2 与 MoveIt 2 的机械臂控制项目 2026

## 一. 环境配置

本教程的复现环境如下。不同电脑型号通常不会影响步骤，但 ROS 2、Ubuntu 和 MoveIt 2 版本建议保持一致；如果版本不同，依赖包名称、MoveIt Setup Assistant 界面和控制器配置可能会有细微差异。

电脑：联想小新 Air 15（无独显）

系统：Ubuntu 22.04（外接硬盘双系统）

ROS 2 版本：Humble

MoveIt 2 版本：Humble 版本

工具：VS Code（包括 ROS、Python、CMake 等扩展）；Terminator（可通过 `sudo apt install terminator` 安装，用于分屏终端，`Ctrl+Shift+O` 上下分屏，`Ctrl+Shift+E` 左右分屏）

<img src="基于ROS2与Moveit2的机械臂控制项目.assets/image-20260612163858504.png" alt="image-20260612163858504" style="zoom:80%;" />

开始编写时间：2026.6.16

结束编写时间：2026.6.22

参考视频：Edouard 老师的相关课程

按照下面的命令、代码及顺序应该是可以一步一步完整跑完流程的



## 二. 项目内容简介

本项目基于 Ubuntu 22.04、ROS 2 Humble 和 MoveIt 2，搭建了一套六自由度机械臂及夹爪的运动规划与控制仿真系统。当前项目完成的是基于 `FakeSystem` 的仿真控制验证：控制器能够接收 MoveIt 规划出的轨迹，仿真硬件能够返回关节状态，并在 RViz 中展示机械臂运动效果；本文不会把它描述为已经完成真实机械臂实机控制。

教程会按真实学习顺序逐步推进：先用 URDF（Unified Robot Description Format，统一机器人描述格式）和 Xacro（XML Macro，XML 宏）描述机械臂结构，再通过 `robot_state_publisher`、TF（Transform，坐标变换）和 RViz 完成模型可视化；随后使用 MoveIt Setup Assistant 配置规划组、命名姿态、末端执行器、碰撞矩阵和控制器映射；再引入 `ros2_control`、`joint_state_broadcaster`、`arm_controller`、`gripper_controller` 与 `FakeSystem`，让规划结果能够被控制器执行。

上层控制部分使用 MoveIt C++ API 编写 `Commander` 节点，把机械臂和夹爪控制封装为命名姿态、关节目标、末端位姿、笛卡尔路径和夹爪开闭等功能，并通过 ROS 2 话题对外提供入口。其中，`joint_command` 用于接收六个关节目标角度，`open_gripper` 用于控制夹爪开闭，`pose_command` 通过自定义消息接收末端位置、姿态和笛卡尔路径标志。

从系统链路看，本项目要建立的是：机器人模型 URDF/Xacro → `joint_states` 与 TF → RViz 可视化 → MoveIt 规划 → ros2_control 控制器 → FakeSystem 仿真硬件 → 状态反馈 → Commander 节点 → 外部 ROS 2 节点通过话题发送关节、位姿和夹爪任务。

后续如果接入真实机械臂，需要额外实现或替换硬件接口（HW Interface）和硬件驱动（HW Driver），由它们把 ROS 2 中的通用关节命令转换为具体厂商控制柜或驱动器能够识别的协议。

## 三. 项目具体内容、操作过程与问题记录

### 1. 六自由度机械臂的 URDF 模型

#### 1.1 创建工作空间

工作空间（workspace）是 ROS 2 项目的顶层目录。后续所有功能包都会放在 `ros2_ws/src` 下，`colcon build` 会把源码编译到 `build`、`install` 和 `log` 目录中。

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面$ mkdir ros2_ws

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面$ cd ros2_ws/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ mkdir src

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ls
src

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build                     
Summary: 0 packages finished [0.70s]

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ls
build  install  log  src

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ cd install

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/install$ ls
COLCON_IGNORE     local_setup.sh            local_setup.zsh  setup.sh
local_setup.bash  _local_setup_util_ps1.py  setup.bash       setup.zsh
local_setup.ps1   _local_setup_util_sh.py   setup.ps1

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/install$ source setup.bash 

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/install$ gedit ~/.bashrc

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/install$ source ~/.bashrc

```

#### 1.2 创建 URDF 功能包

功能包（package）是 ROS 2 组织代码和资源的基本单位。这里创建的 `my_robot_description` 专门保存机器人描述文件、RViz 配置和启动文件。

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面$ cd ros2_ws/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ cd src/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ls
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ros2 pkg create my_robot_description
going to create a new package
package name: my_robot_description
destination directory: /home/rachel/桌面/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['rachel <3246689874@qq.com>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
creating folder ./my_robot_description
creating ./my_robot_description/package.xml
creating source and include folder
creating folder ./my_robot_description/src
creating folder ./my_robot_description/include/my_robot_description
creating ./my_robot_description/CMakeLists.txt

[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identitifers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ls
my_robot_description

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ cd my_robot_description/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on$ ls
CMakeLists.txt  include  package.xml  src

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on$ rm -r include/ src/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on$ mkdir urdf launch rviz

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on$ ls
CMakeLists.txt  launch  package.xml  rviz  urdf

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on$ cd ..

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/$ code .
```

更改CMakeLists.txt文件

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ cd ..

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [1.19s]                

Summary: 1 package finished [1.47s]

#专门构建某个包的命令行，用这个也可以
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build --packages-select my_robot_description
Starting >>> my_robot_description
Finished <<< my_robot_description [0.13s]                  

Summary: 1 package finished [0.41s]

```

#### 1.3 编写 URDF 文件

URDF 用 XML 描述机器人由哪些连杆（link）和关节（joint）组成。连杆描述几何外形，关节描述父子连杆之间怎样连接、沿哪个轴运动、运动范围是多少。

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ cd src/my_robot_description/urdf/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on/urdf$ ls
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on/urdf$ touch arm.urdf
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on/urdf$ cd ../..
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ code .
```

arm.URDF   :

```xml
<?xml version="1.0"?>
<robot name="my_robot">

    <!-- Materials -->
    <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
    </material>

    <material name="blue">
        <color rgba="0.0 0.0 0.5 1.0"/>
    </material>


    <!-- base_link: box size 0.4 0.4 0.1 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.4 0.4 0.1"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- shoulder_link: cylinder length 0.5 radius 0.1 -->
    <link name="shoulder_link">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.1"/>
            </geometry>
            <origin xyz="0 0 0.25" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <!-- arm_link: cylinder length 0.6 radius 0.05 -->
    <link name="arm_link">
        <visual>
            <geometry>
                <cylinder length="0.6" radius="0.05"/>
            </geometry>
            <!-- cylinder default along z, rotate to x direction -->
            <origin xyz="0 0 0.3" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- elbow_link: cylinder length 0.1 radius 0.05 -->
    <link name="elbow_link">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.05"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <!-- forearm_link: cylinder length 0.5 radius 0.05 -->
    <link name="forearm_link">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.05"/>
            </geometry>
            <!-- cylinder default along z, rotate to x direction -->
            <origin xyz="0 0 0.25" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- wrist_link: box size 0.1 0.1 0.05 -->
    <link name="wrist_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.05"/>
            </geometry>
            <origin xyz="0 0 0.025" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <!-- hand_link: box size 0.1 0.1 0.02 -->
    <link name="hand_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.02"/>
            </geometry>
            <origin xyz="0 0 0.01" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- tool_link: no visual -->
    <link name="tool_link"/>

    <!-- joint1: revolute, axis z, min -3.14, max 3.14 -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="shoulder_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint2: revolute, axis y, min 0, max 2.5 -->
    <joint name="joint2" type="revolute">
        <parent link="shoulder_link"/>
        <child link="arm_link"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint3: revolute, axis y, min 0, max 2.5 -->
    <joint name="joint3" type="revolute">
        <parent link="arm_link"/>
        <child link="elbow_link"/>
        <origin xyz="0 0 0.6" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint4: revolute, axis z, min -3.14, max 3.14 -->
    <joint name="joint4" type="revolute">
        <parent link="elbow_link"/>
        <child link="forearm_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint5: revolute, axis y, min -1.57, max 1.57 -->
    <joint name="joint5" type="revolute">
        <parent link="forearm_link"/>
        <child link="wrist_link"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint6: continuous, axis z -->
    <joint name="joint6" type="continuous">
        <parent link="wrist_link"/>
        <child link="hand_link"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <!-- hand_tool_joint: fixed -->
    <joint name="hand_tool_joint" type="fixed">
        <parent link="hand_link"/>
        <child link="tool_link"/>
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
    </joint>

</robot>
```

下个包看可视化效果：

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ sudo apt install ros-humble-urdf-tutorial
[sudo] rachel 的密码： 
正在读取软件包列表... 完成
正在分析软件包的依赖关系树... 完成
正在读取状态信息... 完成                 
下列软件包是自动安装的并且现在不需要了：
  libfwupd2 libfwupdplugin5 libgcab-1.0-0 libsmbios-c2 python3-pyside2.qthelp
  python3-pyside2.qtnetwork python3-pyside2.qtprintsupport
  python3-pyside2.qttest python3-pyside2.qtxml
使用'sudo apt autoremove'来卸载它(它们)。
将会同时安装下列软件：
  ros-humble-urdf-launch
下列【新】软件包将被安装：
  ros-humble-urdf-launch ros-humble-urdf-tutorial
升级了 0 个软件包，新安装了 2 个软件包，要卸载 0 个软件包，有 1 个软件包未被升级。
需要下载 742 kB 的归档。
解压缩后会消耗 1,085 kB 的额外空间。
您希望继续执行吗？ [Y/n] y
获取:1 https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy/main amd64 ros-humble-urdf-launch amd64 0.1.2-1jammy.20260422.111151 [6,424 B]
获取:2 https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy/main amd64 ros-humble-urdf-tutorial amd64 1.1.0-1jammy.20260422.111705 [735 kB]
已下载 742 kB，耗时 1秒 (537 kB/s)                
正在选中未选择的软件包 ros-humble-urdf-launch。
(正在读取数据库 ... 系统当前共安装有 408939 个文件和目录。)
准备解压 .../ros-humble-urdf-launch_0.1.2-1jammy.20260422.111151_amd64.deb  ...
正在解压 ros-humble-urdf-launch (0.1.2-1jammy.20260422.111151) ...
正在选中未选择的软件包 ros-humble-urdf-tutorial。
准备解压 .../ros-humble-urdf-tutorial_1.1.0-1jammy.20260422.111705_amd64.deb  ..
.
正在解压 ros-humble-urdf-tutorial (1.1.0-1jammy.20260422.111705) ...
正在设置 ros-humble-urdf-launch (0.1.2-1jammy.20260422.111151) ...
正在设置 ros-humble-urdf-tutorial (1.1.0-1jammy.20260422.111705) ...

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ source ~/.bashrc
#下包之后一般要source一下

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ cd my_robot_description/urdf/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on/urdf$ ls
arm.urdf

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_descripti
on/urdf$ ros2 launch urdf_tutorial display.launch.py model:=/home/rachel/桌面/ros2_ws/src/my_robot_description/urdf/arm.urdf

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_description/urdf$ ros2 run tf2_tools view_frames
#看tf树
```



#### 1.4 改写 Xacro 文件

Xacro 可以把较长的 URDF 拆成多个文件，并复用材料、尺寸、宏等公共内容。这样后面加入夹爪、控制器标签和 MoveIt 配置时，文件结构会更清晰。

1.arm.Xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- base_link: box size 0.4 0.4 0.1 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.4 0.4 0.1"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- shoulder_link: cylinder length 0.5 radius 0.1 -->
    <link name="shoulder_link">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.1"/>
            </geometry>
            <origin xyz="0 0 0.25" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <!-- arm_link: cylinder length 0.6 radius 0.05 -->
    <link name="arm_link">
        <visual>
            <geometry>
                <cylinder length="0.6" radius="0.05"/>
            </geometry>
            <!-- cylinder default along z, rotate to x direction -->
            <origin xyz="0 0 0.3" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- elbow_link: cylinder length 0.1 radius 0.05 -->
    <link name="elbow_link">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.05"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <!-- forearm_link: cylinder length 0.5 radius 0.05 -->
    <link name="forearm_link">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.05"/>
            </geometry>
            <!-- cylinder default along z, rotate to x direction -->
            <origin xyz="0 0 0.25" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- wrist_link: box size 0.1 0.1 0.05 -->
    <link name="wrist_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.05"/>
            </geometry>
            <origin xyz="0 0 0.025" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <!-- hand_link: box size 0.1 0.1 0.02 -->
    <link name="hand_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.02"/>
            </geometry>
            <origin xyz="0 0 0.01" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
    </link>

    <!-- tool_link: no visual -->
    <link name="tool_link"/>

    <!-- joint1: revolute, axis z, min -3.14, max 3.14 -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="shoulder_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint2: revolute, axis y, min 0, max 2.5 -->
    <joint name="joint2" type="revolute">
        <parent link="shoulder_link"/>
        <child link="arm_link"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint3: revolute, axis y, min 0, max 2.5 -->
    <joint name="joint3" type="revolute">
        <parent link="arm_link"/>
        <child link="elbow_link"/>
        <origin xyz="0 0 0.6" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint4: revolute, axis z, min -3.14, max 3.14 -->
    <joint name="joint4" type="revolute">
        <parent link="elbow_link"/>
        <child link="forearm_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint5: revolute, axis y, min -1.57, max 1.57 -->
    <joint name="joint5" type="revolute">
        <parent link="forearm_link"/>
        <child link="wrist_link"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint6: continuous, axis z -->
    <joint name="joint6" type="continuous">
        <parent link="wrist_link"/>
        <child link="hand_link"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <!-- hand_tool_joint: fixed -->
    <joint name="hand_tool_joint" type="fixed">
        <parent link="hand_link"/>
        <child link="tool_link"/>
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
    </joint>

</robot>
```

2.common_properties.Xacro

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
    </material>

    <material name="blue">
        <color rgba="0 0 0.5 1" />
    </material>

</robot>
```

3.my_robot.URDF.Xacro

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="common_properties.xacro" />
    <xacro:include filename="arm.xacro" />

</robot>
```



#### 1.5 创建启动文件

Launch 文件用于一次启动多个 ROS 2 节点。这里的目标是同时启动 `robot_state_publisher` 和 RViz，使 URDF/Xacro 模型能够被加载并显示出来。

查看节点话题等

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ros2 node list
/joint_state_publisher
/robot_state_publisher
/rviz
/transform_listener_impl_5572d420d030

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ rqt_graph 
```

![image-20260616155554538](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616155554538.png)

在description包的launch文件夹里编写launch启动文件

display.launch.xml

```xml
<launch>
    <let name="urdf_path"
         value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro"/>

    <node pkg="robot_state_publisher" exec="robot_state_publisher" output="screen">
        <param name="robot_description"
               value="$(command 'xacro $(var urdf_path)')"
               type="str"/>
    </node>

    <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui"/>

    <node pkg="rviz2" exec="rviz2" output="screen"/>
</launch>
```

编译看效果：

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.13s]                  

Summary: 1 package finished [0.41s]

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch my_robot_description display.launch.xml 
```

![image-20260616162706433](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616162706433.png)

1.改Fixed Frame；2.点Add添加RobotModel；3.改Description Topic添加机器人状态描述即可看见可视化机器人

TF坐标系也是Add里加

保存RViz配置：File——save config as  保存到description功能包的RViz目录下，可以避免后面重复配置

![image-20260616163424339](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616163424339.png)

更改launch启动文件成下面内容，直接用配置好的RViz文件避免重复配置

```xml
<launch>
    <let name="urdf_path"
         value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro"/>

    <let name="rviz_config_path"
         value="$(find-pkg-share my_robot_description)/rviz/urdf_config.rviz"/>

    <node pkg="robot_state_publisher" exec="robot_state_publisher" output="screen">
        <param name="robot_description"
               value="$(command 'xacro $(var urdf_path)')"
               type="str"/>
    </node>

    <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui"/>

    <node pkg="rviz2" exec="rviz2" output="screen"
          args="-d $(var rviz_config_path)"/>
</launch>
```

编译看效果（每次改文件都养成习惯，编译+source）：

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.13s]                  

Summary: 1 package finished [0.41s]

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch my_robot_description display.launch.xml 
```

无需从新配置

![image-20260616164158876](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616164158876.png)



#### 1.6本节小结

**本节只是帮助大家快速了解怎么编写URDF文件，其实URDF文件以及Xacro文件都很简单，一般工程里也不会自己去做这么粗糙的机械臂，一般路径是在solidwork里配置好，使用sw2URDF插件生成description功能包，不过我之前看这个插件还只适配ROS并不能直接给ROS 2用（不知道现在更新没有），但是这样能得到URDF文件以及工业设计的模型就行，自己改改就能用，很方便！（大家需要知道这个路径方法）**



### 2. 机械臂的 MoveIt 2 相关配置

MoveIt 2 是 ROS 2 中常用的运动规划框架。它负责根据机器人模型、关节限制、碰撞信息和目标状态生成运动轨迹，但不直接控制电机。

#### 2.1 添加碰撞属性标签

本项目为了简化碰撞计算，把圆柱体的碰撞模型都搞成了立方体（其实平时工程问题不用改，生成的碰撞模型就能用）

arm.Xacro文件：

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- base_link: box size 0.4 0.4 0.1 -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.4 0.4 0.1"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.4 0.4 0.1"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>        
        </collision>
    </link>

    <!-- shoulder_link: cylinder length 0.5 radius 0.1 -->
    <link name="shoulder_link">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.1"/>
            </geometry>
            <origin xyz="0 0 0.25" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.2 0.2 0.5"/>
            </geometry>
            <origin xyz="0 0 0.25" rpy="0 0 0"/>     
        </collision>
    </link>

    <!-- arm_link: cylinder length 0.6 radius 0.05 -->
    <link name="arm_link">
        <visual>
            <geometry>
                <cylinder length="0.6" radius="0.05"/>
            </geometry>
            <!-- cylinder default along z, rotate to x direction -->
            <origin xyz="0 0 0.3" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.6"/>
            </geometry>
            <origin xyz="0 0 0.3" rpy="0 0 0"/>
        </collision>
    </link>

    <!-- elbow_link: cylinder length 0.1 radius 0.05 -->
    <link name="elbow_link">
        <visual>
            <geometry>
                <cylinder length="0.1" radius="0.05"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.1"/>
            </geometry>
            <origin xyz="0 0 0.05" rpy="0 0 0"/>
        </collision>
    </link>

    <!-- forearm_link: cylinder length 0.5 radius 0.05 -->
    <link name="forearm_link">
        <visual>
            <geometry>
                <cylinder length="0.5" radius="0.05"/>
            </geometry>
            <!-- cylinder default along z, rotate to x direction -->
            <origin xyz="0 0 0.25" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.05"/>
            </geometry>
            <origin xyz="0 0 0.25" rpy="0 0 0"/> 
        </collision>
    </link>

    <!-- wrist_link: box size 0.1 0.1 0.05 -->
    <link name="wrist_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.05"/>
            </geometry>
            <origin xyz="0 0 0.025" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.05"/>
            </geometry>
            <origin xyz="0 0 0.025" rpy="0 0 0"/>
        </collision>
    </link>

    <!-- hand_link: box size 0.1 0.1 0.02 -->
    <link name="hand_link">
        <visual>
            <geometry>
                <box size="0.1 0.1 0.02"/>
            </geometry>
            <origin xyz="0 0 0.01" rpy="0 0 0"/>
            <material name="grey"/>
        </visual>
        <collision>
            <geometry>
                <box size="0.1 0.1 0.02"/>
            </geometry>
            <origin xyz="0 0 0.01" rpy="0 0 0"/>
        </collision>
    </link>

    <!-- tool_link: no visual -->
    <link name="tool_link"/>

    <!-- joint1: revolute, axis z, min -3.14, max 3.14 -->
    <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="shoulder_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint2: revolute, axis y, min 0, max 2.5 -->
    <joint name="joint2" type="revolute">
        <parent link="shoulder_link"/>
        <child link="arm_link"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint3: revolute, axis y, min 0, max 2.5 -->
    <joint name="joint3" type="revolute">
        <parent link="arm_link"/>
        <child link="elbow_link"/>
        <origin xyz="0 0 0.6" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="0" upper="2.5" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint4: revolute, axis z, min -3.14, max 3.14 -->
    <joint name="joint4" type="revolute">
        <parent link="elbow_link"/>
        <child link="forearm_link"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint5: revolute, axis y, min -1.57, max 1.57 -->
    <joint name="joint5" type="revolute">
        <parent link="forearm_link"/>
        <child link="wrist_link"/>
        <origin xyz="0 0 0.5" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit lower="-1.57" upper="1.57" effort="1000.0" velocity="1.0"/>
    </joint>

    <!-- joint6: continuous, axis z -->
    <joint name="joint6" type="continuous">
        <parent link="wrist_link"/>
        <child link="hand_link"/>
        <origin xyz="0 0 0.05" rpy="0 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>

    <!-- hand_tool_joint: fixed -->
    <joint name="hand_tool_joint" type="fixed">
        <parent link="hand_link"/>
        <child link="tool_link"/>
        <origin xyz="0 0 0.02" rpy="0 0 0"/>
    </joint>

</robot>
```

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面$ cd ros2_ws/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.14s]                  

Summary: 1 package finished [0.42s]
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch my_robot_description display.launch.xml 
```

可视化：

![image-20260616171943429](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616171943429.png)



#### 2.2 使用 MoveIt Setup Assistant 可视化配置机械臂

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch moveit_setup_assistant setup_assistant.launch.py 
```

按照下面步骤一步一步配置即可

![image-20260616172432685](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616172432685.png)

1.导入文件URDF

![image-20260616172558339](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616172558339.png)

2.碰撞矩阵（点一下就行，会允许相邻关节之间有碰撞，不同关节之间不行）

![image-20260616172810413](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616172810413.png)

3.设置虚拟关节

![image-20260616173245328](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616173245328.png)

4.设置规划组（很重要）

![image-20260616173343628](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616173343628.png)

![image-20260616173436260](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616173436260.png)

![image-20260616173523994](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616173523994.png)

![image-20260616173626337](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616173626337.png)

5.创建零位或其他特定姿态

![image-20260616173734225](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616173734225.png)

![image-20260616173806736](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616173806736.png)

6.添加末端执行器

暂无先不做

7.设置被动关节

没有也不用设置

8.ros2_control URDF——软件硬件桥梁

![image-20260616175031381](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616175031381.png)

9.ROS 2 Controllers 控制器

![image-20260616175316073](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616175316073.png)

10.MoveIt controllers 

![image-20260616175404431](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616175404431.png)

11.感知模块 传感器有没有？

没有也不管

12.启动文件

一般不管默认

![image-20260616175533893](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616175533893.png)

13.作者信息

想咋填咋填但是要填一个不能空着

![image-20260616175658747](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616175658747.png)

14.生成配置文件

![image-20260616175915234](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260616175915234.png)



#### 2.3配置生成的文件介绍

![image-20260617090146136](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617090146136.png)

config是配置，可以改有的参数；launch是启动文件，一般只需要关注demo.launch.py和move_group.launch.py文件，其他可以不咋用

| 文件/目录                            | 主要作用                                                     | 新手最常改什么                                         |
| ------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------ |
| `config/`                            | 存放 MoveIt、控制器和机器人约束相关配置                      | 大多数配置修改都在这里完成                             |
| `initial_positions.yaml`             | 设置机器人启动时的初始关节角度                               | 修改机械臂启动后的默认姿态                             |
| `joint_limits.yaml`                  | 设置各关节的位置、速度和加速度限制                           | 调整运动速度、加速度和关节约束                         |
| `kinematics.yaml`                    | 设置逆运动学求解器参数                                       | 末端位姿规划失败或较慢时重点检查                       |
| `moveit_controllers.yaml`            | 告诉 MoveIt 应该把轨迹发送给哪个控制器                       | 添加机械臂、夹爪控制器及对应关节                       |
| `ros2_controllers.yaml`              | 配置 `ros2_control` 中真正执行轨迹的控制器                   | 修改 `arm_controller`、`gripper_controller` 和关节列表 |
| `my_robot.ros2_control.xacro`        | 定义机器人提供哪些控制接口和状态接口                         | 后续接入真实硬件时，通常需要重点修改                   |
| `my_robot.srdf`                      | MoveIt 的语义模型，包含规划组、末端执行器、命名姿态和碰撞规则 | 一般由 Setup Assistant 生成，不建议手动大量修改        |
| `my_robot.urdf.xacro`                | 加载机器人本体 URDF/Xacro 的入口                             | 机器人本体模型变化时需要同步修改                       |
| `pilz_cartesian_limits.yaml`         | 设置 Pilz 笛卡尔规划器的速度、加速度等限制                   | 使用 Pilz 直线、圆弧等工业轨迹规划时再关注             |
| `moveit.rviz`                        | RViz 的显示布局与插件配置                                    | 调整固定坐标系、显示内容和界面布局                     |
| `launch/`                            | 启动 MoveIt、RViz、控制器等节点                              | 根据需要选择启动文件或组合启动流程                     |
| `demo.launch.py`                     | 启动完整演示环境，通常包含 RViz、MoveIt 和 FakeSystem        | 新手最常使用，用于验证整体配置是否正常                 |
| `move_group.launch.py`               | 只启动核心 `move_group` 规划节点                             | 编写自己的 bringup 或 Commander 节点时常用             |
| `moveit_rviz.launch.py`              | 单独启动带 MoveIt 插件的 RViz                                | 只需要可视化和手动规划时使用                           |
| `rsp.launch.py`                      | 启动 `robot_state_publisher`，发布 TF 坐标关系               | 机器人模型或 TF 不显示时重点检查                       |
| `spawn_controllers.launch.py`        | 启动并加载 ros2_control 控制器                               | 控制器没有运行、无法执行轨迹时重点检查                 |
| `static_virtual_joint_tfs.launch.py` | 发布虚拟关节对应的静态 TF                                    | 固定基座与 `world` 坐标系关系异常时检查                |
| `warehouse_db.launch.py`             | 启动 MoveIt 的数据库服务                                     | 初学阶段一般不用                                       |
| `.setup_assistant`                   | 保存 Setup Assistant 的工程信息                              | 通常不需要手动修改                                     |
| `CMakeLists.txt`、`package.xml`      | ROS 2 功能包构建与依赖声明                                   | 增加配置文件、Launch 文件或依赖时修改                  |

初学阶段最常需要关注的是 `joint_limits.yaml`、`kinematics.yaml`、`moveit_controllers.yaml`、`ros2_controllers.yaml` 和 `demo.launch.py`。其中，前四个决定“机器人如何规划和执行”，`demo.launch.py` 则负责把整套演示环境启动起来。MoveIt 配置包通常由 Setup Assistant 生成，包含机器人描述、运动学、关节限制、规划和控制器等参数。



#### 2.4 演示配置可视化效果

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面$ cd ros2_ws/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.16s]                
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [1.13s]                

Summary: 2 packages finished [1.71s]
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash 
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch my_robot_moveit_config demo.launch.py 
```

直接这样运行会报错，RViz里也没有内容，这里其实是MoveIt配置的一个小bug，自己改一下配置文件就行

![image-20260617091020401](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617091020401.png)

![image-20260617091219748](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617091219748.png)

需要改成双精度浮点数，就能正常运行，我也不知道为什么需要这么做，但是这么做了就行

<img src="基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617091712577.png" alt="image-20260617091712577" style="zoom:80%;" />

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.12s]                  
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.11s]                

Summary: 2 packages finished [0.51s]
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash 
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch my_robot_moveit_config demo.launch.py 
```

这里我还出现了一个问题，就是还是不显示模型，之前可以的，现在不行了，我找了找原因，可能是我.bashrc文件里source的工作空间太多了，建议以后大家做的时候注释掉其他不用的工作空间避免环境问题

还有一个问题就是我是下载的MoveIt源码编译后使用，不是二进制使用，所以不知道为什么OMPL这个是有问题的，要解决这个问题才能看见MoveIt正常启动机械臂。如果你没有问题正常启动跳过这里。

**原因：我的 MoveIt 是按 `libompl.so.17` 编译出来的（一年前编译的），但我电脑现在只有 `libompl.so.18`、`libompl.so.16`，没有 `.17`。**

所以 `move_group` 一启动，OMPL 插件就找不到旧库，直接崩掉。MoveIt 源码安装本来要求用 `rosdep install` 安装依赖后再编译；现在这种情况通常是 **OMPL 版本变了，但 `ws_MoveIt2` 没重新编译**。MoveIt 官方源码流程也是先安装依赖，再 `colcon build`。

不要手动做软链接，比如不要把 `.18` 链接成 `.17`。这可能会造成 ABI 不匹配。

现在先做下一步：**重新编译 MoveIt 里的 OMPL 相关包**。

执行：

```bash
cd ~/ws_moveit2

source /opt/ros/humble/setup.bash

colcon build --packages-select moveit_planners_ompl moveit_ros_move_group --cmake-clean-cache
```

现在做下一步检查，看 `libompl.so.17` 有没有变成 `libompl.so.18`。

执行：

```bash
cd ~/ws_moveit2

source /opt/ros/humble/setup.bash
source install/local_setup.bash

ldd ~/ws_moveit2/install/moveit_planners_ompl/lib/libmoveit_ompl_planner_plugin.so | grep ompl
```

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ws_moveit2$ cd ~/ws_moveit2

source /opt/ros/humble/setup.bash
source install/local_setup.bash

ldd ~/ws_moveit2/install/moveit_planners_ompl/lib/libmoveit_ompl_planner_plugin.so | grep ompl
	libmoveit_ompl_interface.so.2.5.8 => /home/rachel/ws_moveit2/install/moveit_planners_ompl/lib/libmoveit_ompl_interface.so.2.5.8 (0x00007ca3b1e8a000)
	libompl.so.18 => /opt/ros/humble/lib/x86_64-linux-gnu/libompl.so.18 (0x00007ca3b1800000)
```

**OMPL 这个问题已经修好了**：

```text
libompl.so.18 => /opt/ros/humble/lib/x86_64-linux-gnu/libompl.so.18
```

这说明源码版 MoveIt 现在已经重新链接到我系统里实际存在的 OMPL 库。ROS 2 的 overlay 会优先于 underlay，所以现在要用干净顺序启动：先 Humble，再 `ws_MoveIt2`，最后你的 `ros2_ws`。ROS 官方文档也是这个 underlay 到 overlay 的逻辑。

**新开一个终端**，执行：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit2/install/local_setup.bash

cd ~/桌面/ros2_ws
source install/setup.bash

ros2 launch my_robot_moveit_config demo.launch.py
```

可视化运行正常

![image-20260617095733156](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617095733156.png)



现在执行试试：我们发现plan正常，但是execute执行会失败，下面我们看看是什么问题，怎么解决！

![image-20260617100501746](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617100501746.png)

修改配置文件，增加两行代码，保存从新编译即可execute了

![image-20260617100821969](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617100821969.png)

![image-20260617102112419](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617102112419.png)



#### 2.5本节小结

**本节主要是学习配置MoveIt，修正了一些问题后进行可视化机械臂展示，MoveIt非常强大，也是做路径规划的好包，后面可以探索一下把自己的算法加进MoveIt里看看能不能跑，做一些扩展**



#### 2.6 拓展知识

你可以先这样理解：

**ROS 2 controller 是真正执行关节运动的底层控制器。
MoveIt controller 是 MoveIt 用来“找到并调用 ROS 2 controller”的配置接口。**

不是两套电机控制器。

##### 1. ROS 2 controller 是什么

ROS 2 controller 属于 `ros2_control`。

它负责：

```text
接收关节目标
→ 控制关节位置/速度/力矩
→ 和真实硬件或仿真硬件交互
→ 发布关节状态
```

`ros2_control` 里有一个 `controller_manager`，它连接 **controllers** 和 **hardware abstraction**，也就是把控制器和硬件抽象层接起来。([ROS控制器](https://control.ros.org/humble/doc/getting_started/getting_started.html?utm_source=chatgpt.com))

在我的日志里这些就是 ROS 2 controller 相关内容：

```text
ros2_control_node
joint_state_broadcaster
arm_controller
FakeSystem
```

含义是：

```text
FakeSystem              假硬件
joint_state_broadcaster 发布关节状态
arm_controller          接收轨迹并控制机械臂关节
```

我现在用的是 `FakeSystem`，所以不是接真实机械臂电机，而是在模拟一个硬件接口。

##### 2. MoveIt controller 是什么

MoveIt controller 不是直接控制电机。

MoveIt 的核心任务是：

```text
规划机械臂怎么动
```

比如它算出一条轨迹：

```text
joint1 从 0 到 0.5
joint2 从 0 到 -0.3
joint3 从 0 到 1.0
...
```

但是 MoveIt 自己不负责真正让关节动。它要把轨迹发给底层控制器。MoveIt 文档里也说，MoveIt 通常把机械臂运动命令发送给 `JointTrajectoryController`。([MoveIt](https://MoveIt.picknik.ai/humble/doc/examples/controller_configuration/controller_configuration_tutorial.html?utm_source=chatgpt.com))

所以 MoveIt controller 配置的作用是告诉 MoveIt：

```text
你要执行轨迹时，去找哪个 ROS2 controller
```

比如：

```text
MoveIt 规划出来轨迹 → 发送给 arm_controller → arm_controller 控制 FakeSystem 或真实硬件
```

##### 3. 两者关系

最重要的一条链路是：

```text
RViz / MoveIt
    ↓ 规划轨迹
move_group
    ↓ 发送 FollowJointTrajectory action
MoveIt controller 配置
    ↓ 找到 arm_controller
ROS2 controller: arm_controller
    ↓ 控制关节接口
FakeSystem / 真实机械臂硬件
```

所以区别是：

| 名称              | 属于哪里     | 作用                                           |
| ----------------- | ------------ | ---------------------------------------------- |
| ROS 2 controller   | ros2_control | 真正接收轨迹并控制关节                         |
| MoveIt controller | MoveIt       | 告诉 MoveIt 应该把轨迹发给哪个 ROS 2 controller |
| hardware          | ros2_control | 真实电机驱动或 FakeSystem 假硬件               |
| move_group        | MoveIt       | 负责规划，不直接控制电机                       |

##### 4. 用我的情况说

我现在的情况应该是：

```text
MoveIt 里面的规划组：arm
ROS2 controller 里面的控制器：arm_controller
硬件：FakeSystem
```

也就是说：

```text
arm 是 MoveIt 规划对象
arm_controller 是 ros2_control 执行轨迹的控制器
FakeSystem 是假硬件
```

一句话总结：

```text
MoveIt controller 负责“找人干活”；
ROS2 controller 负责“真正干活”；
hardware 负责“被控制的关节或电机”。
```



### 3. 添加夹爪（末端执行器）

末端执行器（end effector）是安装在机械臂末端、用于执行具体任务的机构。本项目中的末端执行器是一个简单夹爪，用于演示 MoveIt 中机械臂规划组和夹爪规划组的组合配置。

通常来说不会只做机械臂的规划，我们会加一个末端执行器来完成任务

#### 3.1 为夹爪创建一个 URDF

/home/rachel/桌面/ros2_ws/src/my_robot_description/URDF/gripper.Xacro

```xml
<?xml version="1.0"?>
<robot name="temp" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- 材质 -->
  <material name="light_green">
    <color rgba="0.4 1.0 0.4 1.0"/>
  </material>

  <material name="dark_green">
    <color rgba="0.0 0.4 0.0 1.0"/>
  </material>

  <!-- 夹爪底座 -->
  <link name="gripper_base_link">
    <visual>
      <geometry>
        <box size="0.20 0.06 0.02"/>
      </geometry>
      <origin xyz="0 0 0.01" rpy="0 0 0"/>
      <material name="dark_green"/>
    </visual>

    <collision>
      <geometry>
        <box size="0.20 0.06 0.02"/>
      </geometry>
      <origin xyz="0 0 0.01" rpy="0 0 0"/>
    </collision>
  </link>

  <!-- 左夹爪手指 -->
  <link name="gripper_left_finger_link">
    <visual>
      <geometry>
        <box size="0.02 0.06 0.08"/>
      </geometry>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
      <material name="light_green"/>
    </visual>

    <collision>
      <geometry>
        <box size="0.02 0.06 0.08"/>
      </geometry>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
    </collision>
  </link>

  <!-- 右夹爪手指 -->
  <link name="gripper_right_finger_link">
    <visual>
      <geometry>
        <box size="0.02 0.06 0.08"/>
      </geometry>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
      <material name="light_green"/>
    </visual>

    <collision>
      <geometry>
        <box size="0.02 0.06 0.08"/>
      </geometry>
      <origin xyz="0 0 0.04" rpy="0 0 0"/>
    </collision>
  </link>

  <!-- 左手指连接到底座 -->
  <joint name="gripper_left_finger_joint" type="prismatic">
    <parent link="gripper_base_link"/>
    <child link="gripper_left_finger_link"/>
    <origin xyz="-0.07 0 0.02" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit effort="1000.0" velocity="1.0" lower="0.0" upper="0.06"/>
  </joint>

  <!-- 右手指连接到底座 -->
  <joint name="gripper_right_finger_joint" type="prismatic">
    <parent link="gripper_base_link"/>
    <child link="gripper_right_finger_link"/>
    <origin xyz="0.07 0 0.02" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <mimic joint="gripper_left_finger_joint" multiplier="-1.0"/>
    <limit effort="1000.0" velocity="1.0" lower="-0.06" upper="0.0"/>
  </joint>

</robot>
```

可视化正常：ros2 launch URDF_tutorial display.launch.py model:=/home/rachel/桌面/ros2_ws/src/my_robot_description/URDF/gripper.Xacro 

![image-20260617114111900](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617114111900.png)



#### 3.2 把夹爪加到机械臂上

改/home/rachel/桌面/ros2_ws/src/my_robot_description/URDF/my_robot.URDF.Xacro文件；

1.把gripper.Xacro文件的材质信息放到my_robot_description/URDF/common_properties.Xacro文件里，后面编写更好，这只是一个操作，管理URDF文件属性等的习惯，可以不做但是做了更好，这里不是很影响

2.删掉gripper.Xacro文件里的robot name=temp这个，因为要放到my_robot.URDF.Xacro文件里只有一个名字就行，相当于刚刚是一个文件，现在把文件要拆解成身体不是要整个

3.添加关节把夹爪安装到机械臂末端

my_robot.URDF.Xacro文件：

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="common_properties.xacro" />
  <xacro:include filename="arm.xacro" />
  <xacro:include filename="gripper.xacro" />

  <joint name="gripper_base_joint" type="fixed">
    <parent link="tool_link" />
    <child link="gripper_base_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>

</robot>
```

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.13s]                  
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.11s]                

Summary: 2 packages finished [0.54s]
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash 
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch urdf_tutorial display.launch.py model:=/home/rachel/桌面/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro 
```

可视化界面：

![image-20260617115729061](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617115729061.png)



#### 3.3 使用 MoveIt 配置夹爪

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

1.导入文件

![image-20260617151411598](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617151411598.png)

2.生成碰撞

![image-20260617151509365](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617151509365.png)

3.设置夹爪的规划组

![image-20260617151726635](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617151726635.png)

![image-20260617151623914](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617151623914.png)

![image-20260617151825083](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617151825083.png)

4.设置夹爪的一些特殊位置

![image-20260617152009841](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152009841.png)

![image-20260617151947304](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617151947304.png)

![image-20260617152142031](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152142031.png)

5.设置末端执行器

![image-20260617152229190](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152229190.png)

![image-20260617152311614](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152311614.png)

6.设置ros2_control

![image-20260617152417598](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152417598.png)

7.ROS 2 Controllers

![image-20260617152459394](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152459394.png)

8.MoveIt controllers

![image-20260617152604107](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152604107.png)

9.保存配置（保存之前可以对之前的先进行备份）

![image-20260617152834000](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617152834000.png)

10.修复之前出现过的问题，配置之后要自觉改，改了才是正确的配置包

​       1.my_robot_MoveIt_config/config/joint_limits.yaml文件里要改成浮点数；限制改为true，不对的关节也要修改一下

```yaml
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 1.0
default_acceleration_scaling_factor: 1.0

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  gripper_left_finger_joint:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  gripper_right_finger_joint:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint1:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint2:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint3:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint4:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint5:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
  joint6:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
```

​     2.my_robot_MoveIt_config/config/MoveIt_controllers.yaml补上一点内容，让夹爪也可以执行

```yaml
# MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_controller
    - gripper_controller

  arm_controller:
    type: FollowJointTrajectory
    joints:
      - joint1
      - joint2
      - joint3
      - joint4
      - joint5
      - joint6
    action_ns: follow_joint_trajectory
    default: true

  gripper_controller:
    type: FollowJointTrajectory
    joints:
      - gripper_left_finger_joint
    action_ns: follow_joint_trajectory
    default: true
```



可视化：

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.13s]                  
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.36s]                

Summary: 2 packages finished [0.78s]
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash 
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch my_robot_moveit_config demo.launch.py 
```

![image-20260617154057333](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617154057333.png)



#### 3.4本节小结

这一节主要是创建夹爪URDF以及配置夹爪的MoveIt，这个跟前两章是一样的，也很简单，我之前本科毕业设计也做过，不过比这个模型要复杂的多，我做的双臂的，模型很复杂，但是原理是一样的。按照我上面的来就能做，复杂的也是一样的！



### 4. 创建启动专用功能包

`bringup` 功能包通常用于统一启动一个机器人系统需要的节点和配置。这样读者不用分别启动机器人描述、控制器、MoveIt 和 RViz，后续测试也更接近工程项目的组织方式。

在一个工程里，如果启动的文件到处都是，一是不好找，二是配置老要从新配，三是容易导致环境依赖冲突，所以，一般做一个大的工程项目的时候，我们需要做一个专用的方启动文件的功能包，使我们能集中管理应用程序的启动部分，这是培养项目管理能力的好办法。这是ROS社区的好习惯，也会让其他开发者更容易加入项目。

本章主要做my_robot_bringup包



#### 4.1创建my_robot_bringup功能包

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ cd src
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ls
my_robot_description  my_robot_moveit_config
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ros2 pkg create my_robot_bringup
going to create a new package
package name: my_robot_bringup
destination directory: /home/rachel/桌面/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['rachel <3246689874@qq.com>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
creating folder ./my_robot_bringup
creating ./my_robot_bringup/package.xml
creating source and include folder
creating folder ./my_robot_bringup/src
creating folder ./my_robot_bringup/include/my_robot_bringup
creating ./my_robot_bringup/CMakeLists.txt

[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identitifers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ls
my_robot_bringup  my_robot_description  my_robot_moveit_config
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ cd my_robot_bringup/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_bringup$ ls
CMakeLists.txt  include  package.xml  src
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_bringup$ rm -r include/ src/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_bringup$ mkdir launch config
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_bringup$ ls
CMakeLists.txt  config  launch  package.xml
```

修改my_robot_bringup/CMakeLists.txt：

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

```

下面做的目的是可以不需要启动时加载MoveIt config包，导致依赖错综复杂或者从新配置了出问题。还是那句话：可以不做，但是这是培养的工程管理思维，最好是能把细节处理好，不然一个项目越来越大就更难搞了

复制一份my_robot_MoveIt_config/config/ros2_controllers.yaml到my_robot_bringup/config/ros2_controllers.yaml

复制一份my_robot_MoveIt_config/config/my_robot.ros2_control.Xacro到my_robot_description/URDF/my_robot.ros2_control.Xacro

修改my_robot_description/URDF/my_robot.URDF.Xacro：

```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:include filename="common_properties.xacro" />
  <xacro:include filename="arm.xacro" />
  <xacro:include filename="gripper.xacro" />
  <xacro:include filename="my_robot.ros2_control.xacro" />

  <joint name="gripper_base_joint" type="fixed">
    <parent link="tool_link" />
    <child link="gripper_base_link" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>

</robot>
```

修改my_robot_description/URDF/my_robot.ros2_control.Xacro：删去不要的宏和属性，修改一些小细节

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <ros2_control name="Arm" type="system">
        <hardware>
            <!-- By default, set up controllers for simulation. This won't work on real hardware -->
            <plugin>mock_components/GenericSystem</plugin>
        </hardware>
        <joint name="joint1">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
        </joint>
        <joint name="joint2">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
        </joint>
        <joint name="joint3">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
        </joint>
        <joint name="joint4">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
        </joint>
        <joint name="joint5">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
        </joint>
        <joint name="joint6">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
        </joint>
        <joint name="gripper_left_finger_joint">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.0</param>
            </state_interface>
        </joint>

    </ros2_control>

</robot>

```



#### 4.2编写启动文件

my_robot_bringup/launch/my_robot.launch.xml

```xml
<launch>

  <let name="urdf_path"
       value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro" />

  <let name="rviz_config_path"
       value="$(find-pkg-share my_robot_description)/rviz/urdf_config.rviz" />

  <node pkg="robot_state_publisher" exec="robot_state_publisher" output="screen">
       <param name="robot_description"
              value="$(command 'xacro $(var urdf_path)')"
              type="str"/>
  </node>

  <node pkg="controller_manager" exec="ros2_control_node">
    <param from="$(find-pkg-share my_robot_bringup)/config/ros2_controllers.yaml" />
  </node>

  <node pkg="controller_manager" exec="spawner" args="joint_state_broadcaster" />

  <node pkg="controller_manager" exec="spawner" args="arm_controller" />

  <node pkg="controller_manager" exec="spawner" args="gripper_controller" />

  <include file="$(find-pkg-share my_robot_moveit_config)/launch/move_group.launch.py" />

  <node pkg="rviz2" exec="rviz2" output="screen"
        args="-d $(var rviz_config_path)" />

</launch>
```

my_robot_bringup/package.xml添加依赖

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_bringup</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="3246689874@qq.com">rachel</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>my_robot_description</exec_depend>
  <exec_depend>my_robot_moveit_config</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>rviz2</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

可视化

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Finished <<< my_robot_description [0.13s]                  
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.11s]                
Starting >>> my_robot_bringup
Finished <<< my_robot_bringup [1.06s]                  

Summary: 3 packages finished [1.59s]
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source install/setup.bash 
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ ros2 launch my_robot_bringup my_robot.launch.xml 
```

![image-20260617174915652](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617174915652.png)

添加MoveIt，保存配置

![image-20260617175332514](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617175332514.png)

![image-20260617175400167](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260617175400167.png)

改启动文件my_robot_bringup/launch/my_robot.launch.xml的配置文件

```text
  <let name="rviz_config_path"
       value="$(find-pkg-share my_robot_bringup)/config/my_robot_moveit.rviz" />
```

#### 4.3本节小结

这一节主要是编写启动文件，便于后面直接打开直接用，这一节学习的是如何编写启动文件，如何确定启动要什么节点话题等，以后做工程项目的时候也是要编写launch文件来管理项目的。



### 5. 使用 MoveIt C++ API

MoveGroupInterface 是 MoveIt C++ API 中最常用的高层接口。它可以设置命名姿态、关节目标、末端位姿目标，也可以请求 MoveIt 规划并执行轨迹。

MoveIt C++ API 可以理解成：**你在 C++ 代码里给机械臂下命令的工具箱**，比如设定目标姿态、规划路径、执行运动、添加障碍物，而真正复杂的运动规划由 MoveIt 后台的 `move_group` 节点完成。官方文档也说，`MoveGroupInterface` 是 MoveIt 中最简单的用户接口，可用于设置关节或位姿目标、生成运动规划、移动机器人、添加环境物体等，并通过 ROS topic、service、action 与 `move_group` 通信。

再压缩成一句话：**MoveIt C++ API 就是让你用 C++ 告诉机械臂“去哪、怎么避障、开始动”的接口。**

#### 5.1 区分 RViz、C++、Python 控制 MoveIt（概念区分）

**MoveIt 不是“自己在控制机械臂”，而是一个运动规划大脑；RViz、C++代码、Python代码只是三种不同的“下命令方式”。**

##### 1. “MoveIt自己控制”通常指什么？

我现在说的“MoveIt自己控制”，是指 **RViz 里拖动机械臂末端，然后点 Plan / Execute**。

它的逻辑是：**人用鼠标给目标点 → MoveIt 规划路径 → ros2_control / 控制器执行轨迹**

MoveIt 官方 RViz 教程也说明，RViz 插件可以交互式设置起点、目标点、规划场景、测试规划器并可视化结果。

所以它适合：**调试模型、检查URDF、检查SRDF、检查规划组、看机械臂能不能动。**

但它不适合真正写项目逻辑，因为每次都要人手动拖。

------

##### 2. C++ API 是什么？

C++ API 就是你不再用鼠标拖，而是写代码：

```text
move_group.setPoseTarget(target_pose);
move_group.plan(plan);
move_group.execute(plan);
```

意思是：**程序告诉 MoveIt：末端去这个位置，帮我规划并执行。**

MoveIt 官方教程里，ROS 2 C++ 项目的基本流程就是创建 ROS 节点，然后用 `MoveGroupInterface` 进行规划和执行。
MoveIt 的 C++ `MoveGroupInterface` 也是官方常用的高层接口，用来完成大多数常见操作，例如设置目标、规划、执行、处理规划场景等。

所以 C++ API 适合：**正式项目、机械臂自动流程、和夹爪/相机/传感器联动。**

比如你以后要写：“机械臂先到物体上方 → 张开夹爪 → 下移 → 闭合夹爪 → 抬起 → 放到目标位置”

这个就不能一直靠 RViz 手拖，而应该写 C++ 或 Python 程序。

------

##### 3. Python代码和C++代码有什么区别？

本质上它们做的事很像：**都是给 MoveIt 发命令，让 MoveIt 规划轨迹。**

区别主要是：

| 方式          | 你怎么操作                      | 适合干什么                     |
| ------------- | ------------------------------- | ------------------------------ |
| RViz 手动控制 | 鼠标拖目标点，点 Plan / Execute | 调试、验证模型能不能动         |
| C++ API       | 写 C++ 节点控制 MoveIt          | 正式工程、机器人项目、复杂流程 |
| Python API    | 写 Python 脚本控制 MoveIt       | 快速测试、教学、简单实验       |

ROS 1 里常见的是 `MoveIt_commander`，它提供 `MoveGroupCommander`、`PlanningSceneInterface`、`RobotCommander` 等 Python 接口。
ROS 2 里也有 Python 接口发展路线，例如 MoveIt 官方介绍过新的 Python library，它是把 MoveIt 的 C++ 核心组件绑定后暴露给 Python 使用。

简单说：**C++ 更像正式工程版，Python 更像快速实验版。**

------

##### 4. 最关键的一句话

**RViz 是你手动点按钮控制 MoveIt；C++ / Python 是你写程序自动控制 MoveIt；真正规划路径的核心还是 MoveIt。**

再用一个类比：**MoveIt 像导航软件，RViz 像你手动在地图上点目的地，C++ / Python 像你写程序自动输入目的地；导航路线还是 MoveIt 算出来的。**

小白阶段的学习顺序建议是：**先用 RViz 确认机械臂能规划 → 再写最简单的 C++ API 控制一个目标点 → 最后再加入夹爪、多个点位和完整任务流程。**



#### 5.2 编写一个简单的 C++ 测试文件调用 MoveIt API 接口控制机械臂

使用的是之前MoveIt配置的一些pose，主要学代码怎么写，这小节实际动作非常简单，就是从一个位置到另外一个位置，且是规定好的位置

创建C++功能包

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面$ cd ros2_ws/src/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ls
my_robot_bringup  my_robot_description  my_robot_moveit_config

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ros2 pkg create my_robot_commander_cpp --build-type ament_cmake --dependencies rclcpp
going to create a new package
package name: my_robot_commander_cpp
destination directory: /home/rachel/桌面/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['rachel <3246689874@qq.com>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: ['rclcpp']
creating folder ./my_robot_commander_cpp
creating ./my_robot_commander_cpp/package.xml
creating source and include folder
creating folder ./my_robot_commander_cpp/src
creating folder ./my_robot_commander_cpp/include/my_robot_commander_cpp
creating ./my_robot_commander_cpp/CMakeLists.txt

[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identitifers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ls
my_robot_bringup        my_robot_description
my_robot_commander_cpp  my_robot_moveit_config
```

创建my_robot_commander_cpp/src/test_MoveIt.cpp文件：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    // Named goal
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("pose_1");

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    bool success1 = static_cast<bool>(arm.plan(plan1));

    if (success1)
    {
        arm.execute(plan1);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
```

改my_robot_commander_cpp/CMakeLists.txt文件：

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)

ament_target_dependencies(
  test_moveit
  rclcpp
  moveit_ros_planning_interface
)

install(
  TARGETS test_moveit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

修改my_robot_commander_cpp/package.xml文件：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_commander_cpp</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="3246689874@qq.com">rachel</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>moveit_ros_planning_interface</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

```

运行测试

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ colcon build
Starting >>> my_robot_description
Starting >>> my_robot_commander_cpp
Finished <<< my_robot_description [0.14s]                                  
Starting >>> my_robot_moveit_config
Finished <<< my_robot_moveit_config [0.12s]                                    
Starting >>> my_robot_bringup
Finished <<< my_robot_bringup [0.11s]                                          
Finished <<< my_robot_commander_cpp [7.27s]                     

Summary: 4 packages finished [7.57s]
```

```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

```bash
ros2 run my_robot_commander_cpp test_moveit
```

上面两个命令行在两个终端里依次运行，可视化效果如下图，执行第二个命令后机械臂自己运动到pose_1,无需在RViz里拖拽

![image-20260618134705228](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260618134705228.png)

<video src="基于ROS2与Moveit2的机械臂控制项目.assets/moveit_clip_3_9s.mp4"></video>

```text
你可以再在cpp文件里添加其他动作，比如
    // Named goal
    arm.setStartStateToCurrentState();
    arm.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan plan2;

    bool success2 = static_cast<bool>(arm.plan(plan2));

    if (success2)
    {
        arm.execute(plan2);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    }
    
以这种形式可以规划一连串的动作，实现c++以moveit API的形式实现机械臂的规划控制
```

添加夹爪也是一样的

![image-20260618141110640](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260618141110640.png)





#### 5.3 发送关节目标和姿态目标

修改my_robot_commander_cpp/src/test_MoveIt.cpp文件，发送**关节目标指令**，学习这种写法，初始化和结束不用管都是一样的

修改保存构建source记得每次都要做防止没变化还是之前的

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");


    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // bool success1 = static_cast<bool>(arm.plan(plan1));

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;

    // bool success2 = static_cast<bool>(arm.plan(plan2));

    // if (success2)
    // {
    //     arm.execute(plan2);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    //--------------------------------------------------------------------------------

    //Joint Goal
    std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    arm.setStartStateToCurrentState();
    arm.setJointValueTarget(joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    bool success1 =
        (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        arm.execute(plan1);
    }

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
```

可视化：

<video src="基于ROS2与Moveit2的机械臂控制项目.assets/moveit_5_3_JointGoal.mp4"></video>

修改my_robot_commander_cpp/src/test_MoveIt.cpp文件，发送**姿态目标指令**，学习这种写法，初始化和结束不用管都是一样的

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");


    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // bool success1 = static_cast<bool>(arm.plan(plan1));

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;

    // bool success2 = static_cast<bool>(arm.plan(plan2));

    // if (success2)
    // {
    //     arm.execute(plan2);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    //--------------------------------------------------------------------------------

    // //Joint Goal
    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // bool success1 =
    //     (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }


    //---------------------------------------------------------------------------------

    // Pose Goal
    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    bool success1 =
        (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        arm.execute(plan1);
    }


    rclcpp::shutdown();
    spinner.join();

    return 0;
}
```

可视化不演示了跟上面差不多，学习写法最重要

![image-20260618155857206](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260618155857206.png)



#### 5.4 笛卡尔路径

**笛卡尔路径就是让机械臂末端按你规定的直线或折线轨迹移动，例如“垂直向下 10 cm”，而不是只管起点和终点、中间怎么走由规划器自己决定。** MoveIt 中通常通过多个空间路径点让末端依次经过这些点来生成这类轨迹。

修改my_robot_commander_cpp/src/test_MoveIt.cpp文件，发送笛卡尔坐标，学习这种写法，初始化和结束不用管都是一样的

修改保存构建source记得每次都要做防止没变化还是之前的

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_moveit");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() {
        executor.spin();
    });

    auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");

    arm.setMaxVelocityScalingFactor(1.0);
    arm.setMaxAccelerationScalingFactor(1.0);

    auto gripper = moveit::planning_interface::MoveGroupInterface(node, "gripper");


    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("pose_1");

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // bool success1 = static_cast<bool>(arm.plan(plan1));

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    // // Named goal
    // arm.setStartStateToCurrentState();
    // arm.setNamedTarget("home");

    // moveit::planning_interface::MoveGroupInterface::Plan plan2;

    // bool success2 = static_cast<bool>(arm.plan(plan2));

    // if (success2)
    // {
    //     arm.execute(plan2);
    // }
    // else
    // {
    //     RCLCPP_ERROR(node->get_logger(), "Planning failed!");
    // }

    //--------------------------------------------------------------------------------

    // //Joint Goal
    // std::vector<double> joints = { 1.5, 0.5, 0.0, 1.5, 0.0, -0.7 };

    // arm.setStartStateToCurrentState();
    // arm.setJointValueTarget(joints);

    // moveit::planning_interface::MoveGroupInterface::Plan plan1;

    // bool success1 =
    //     (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (success1)
    // {
    //     arm.execute(plan1);
    // }


    //---------------------------------------------------------------------------------

    // Pose Goal
    tf2::Quaternion q;
    q.setRPY(3.14, 0.0, 0.0);
    q = q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.pose.position.x = 0.0;
    target_pose.pose.position.y = -0.7;
    target_pose.pose.position.z = 0.4;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm.setStartStateToCurrentState();
    arm.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan1;

    bool success1 =
        (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success1)
    {
        arm.execute(plan1);
    }

    // Cartesian Path

    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose pose1 = arm.getCurrentPose().pose;
    pose1.position.z += 0.2;
    waypoints.push_back(pose1);

    moveit_msgs::msg::RobotTrajectory trajectory;

    double fraction = arm.computeCartesianPath(
        waypoints,
        0.01,
        0.0,
        trajectory,
        true,
        nullptr
    );

    if (fraction == 1.0)
    {
        arm.execute(trajectory);
    }

    rclcpp::shutdown();
    spinner.join();

    return 0;
}
```

![image-20260618162239838](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260618162239838.png)



#### 5.5 更符合 ROS 2 和 MoveIt 的 C++ 工程写法

上面的写法如果内容很多会很难看，也很难管理，所以下面我们来看更工程的写法，创建类对象的写法，构建ROS 2节点等，这样可以更好的连接ROS 2和MoveIt。更工程，封装的更好

编写my_robot_commander_cpp/src/commander_template.cpp

```cpp
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

class Commander
{
public:
    Commander(const std::shared_ptr<rclcpp::Node>& node)
    {
        node_ = node;

        // 创建机械臂控制接口
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        // 创建夹爪控制接口
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");
    }

    // 1. 前往 SRDF 中保存的命名姿态，例如 pose_1、home
    void goToNamedTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);

        planAndExecute(arm_);
    }

    // 2. 前往指定关节角度
    void goToJointTarget(const std::vector<double>& joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);

        planAndExecute(arm_);
    }

    // 3. 前往指定末端位姿
    // cartesian_path = false：普通规划，只保证到达终点
    // cartesian_path = true ：笛卡尔路径，末端尽量沿直线运动
    void goToPoseTarget(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;

        // 使用机械臂当前的规划参考坐标系，通常就是 base_link
        const std::string reference_frame = arm_->getPlanningFrame();
        arm_->setPoseReferenceFrame(reference_frame);

        target_pose.header.frame_id = reference_frame;

        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        // 普通位姿规划
        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);

            planAndExecute(arm_);

            // 清除本次位姿目标，避免影响后续规划
            arm_->clearPoseTargets();
        }
        // 笛卡尔路径规划
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;

            // 从当前末端位置，直线移动到 target_pose
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;

            // 你的 MoveIt 版本需要 6 个参数
            double fraction = arm_->computeCartesianPath(
                waypoints,
                0.01,        // 每一步最大间隔：1 cm
                0.0,         // 关节跳变阈值
                trajectory,
                true,        // 是否避障
                &error_code  // 错误码输出
            );

            // fraction 接近 1，说明整条路径都规划成功
            if (fraction >= 0.999)
            {
                auto result = arm_->execute(trajectory);

                if (result != moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Cartesian path execution failed!"
                    );
                }
            }
            else
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Cartesian path only completed %.1f%%",
                    fraction * 100.0
                );
            }
        }
    }

private:
    // 所有普通规划共用：规划成功才执行
    void planAndExecute(
        const std::shared_ptr<MoveGroupInterface>& interface)
    {
        MoveGroupInterface::Plan plan;

        bool success =
            (interface->plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            auto result = interface->execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Trajectory execution failed!"
                );
            }
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Motion planning failed!"
            );
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    {
        Commander commander(node);

        // 给 MoveIt 一点时间接收当前机械臂状态
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        // 一次只保留一种测试方式，其他先注释掉

        // 方式 1：前往命名姿态
        commander.goToNamedTarget("pose_1");

        // 方式 2：前往指定关节角度
        // commander.goToJointTarget({1.5, 0.5, 0.0, 1.5, 0.0, -0.7});

        // 方式 3：普通末端位姿规划
        // commander.goToPoseTarget(
        //     0.0, -0.7, 0.4,
        //     3.14, 0.0, 0.0,
        //     false
        // );

        // 方式 4：笛卡尔路径，末端尽量直线到目标位置
        // commander.goToPoseTarget(
        //     0.0, -0.7, 0.4,
        //     3.14, 0.0, 0.0,
        //     true
        // );
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
```

 `CMakeLists.txt` 中建议再补上这几个依赖：

```cmake
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
```

并把原来的依赖改为：

```cmake
ament_target_dependencies(
  test_moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
)
```

`package.xml` 里也补：

```text
<depend>geometry_msgs</depend>
<depend>moveit_msgs</depend>
<depend>tf2</depend>
```

这个程序的作用一句话：**它把控制机械臂的多种 MoveIt 操作封装成一个 `Commander` 类，使程序可以用统一方法让机械臂到达命名姿态、关节角度、空间位姿或沿直线运动。**

相比之前直接在 `main()` 里逐段写的方式，这种写法的好处一句话：**公共的规划与执行逻辑只写一次，后续增加抓取、放置、夹爪开合等任务时只需调用方法，代码更清晰、更不容易重复出错。**

MoveIt 的 `MoveGroupInterface` 本来就是面向规划组的高层 C++ 接口，可设置命名目标、关节目标、位姿目标并执行轨迹；笛卡尔路径会按路径点和 `eef_step` 生成末端路径，`jump_threshold` 用于限制逆解时异常的关节跳变。



#### 5.6 向程序中添加一个话题订阅者（给夹爪创建通信接口）

继续编写my_robot_commander_cpp/src/commander_template.cpp

```cpp
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <example_interfaces/msg/bool.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

using Bool = example_interfaces::msg::Bool;

class Commander
{
public:
    Commander(const std::shared_ptr<rclcpp::Node>& node)
    {
        node_ = node;

        // 创建机械臂控制组
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        // 创建夹爪控制组
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        // 订阅 open_gripper 话题
        // true：打开夹爪
        // false：关闭夹爪
        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper",
            10,
            std::bind(
                &Commander::openGripperCallback,
                this,
                std::placeholders::_1));
    }

    // 1. 前往 MoveIt 中保存的命名姿态，例如 pose_1、home
    void goToNamedTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);

        planAndExecute(arm_);
    }

    // 2. 前往指定关节角度
    void goToJointTarget(const std::vector<double>& joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);

        planAndExecute(arm_);
    }

    // 3. 前往指定末端位姿
    // cartesian_path = false：普通路径规划，只保证最终到达目标
    // cartesian_path = true ：笛卡尔路径，末端尽量沿直线到目标
    void goToPoseTarget(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;

        // 使用机械臂实际规划坐标系，通常为 base_link
        target_pose.header.frame_id = arm_->getPlanningFrame();

        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        // 普通位姿规划
        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);

            planAndExecute(arm_);

            arm_->clearPoseTargets();
        }
        // 笛卡尔路径规划
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;

            double fraction = arm_->computeCartesianPath(
                waypoints,
                0.01,        // 末端每一步最大移动距离：1 cm
                0.0,         // 关节跳变阈值
                trajectory,
                true,        // 是否避障
                &error_code  // 接收错误码
            );

            if (fraction >= 0.999)
            {
                auto result = arm_->execute(trajectory);

                if (result != moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Cartesian path execution failed!");
                }
            }
            else
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Cartesian path only completed %.1f%%",
                    fraction * 100.0);
            }
        }
    }

    // 打开夹爪
    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");

        planAndExecute(gripper_);
    }

    // 关闭夹爪
    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");

        planAndExecute(gripper_);
    }

private:
    // 所有普通规划共用：规划成功后再执行
    void planAndExecute(
        const std::shared_ptr<MoveGroupInterface>& interface)
    {
        MoveGroupInterface::Plan plan;

        bool success =
            (interface->plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            auto result = interface->execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Trajectory execution failed!");
            }
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Motion planning failed!");
        }
    }

    // 收到 open_gripper 话题消息时触发
    void openGripperCallback(const Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(node_->get_logger(), "Opening gripper...");
            openGripper();
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Closing gripper...");
            closeGripper();
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    {
        Commander commander(node);

        RCLCPP_INFO(
            node->get_logger(),
            "Commander is ready. Waiting for open_gripper topic...");

        // 下面这些是之前的方法测试示例。
        // 一次只打开一个，其他保持注释。

        // commander.goToNamedTarget("pose_1");

        // commander.goToJointTarget(
        //     {1.5, 0.5, 0.0, 1.5, 0.0, -0.7}
        // );

        // commander.goToPoseTarget(
        //     0.0, -0.7, 0.4,
        //     3.14, 0.0, 0.0,
        //     false
        // );

        // commander.goToPoseTarget(
        //     0.0, -0.7, 0.4,
        //     3.14, 0.0, 0.0,
        //     true
        // );

        while (rclcpp::ok())
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));
        }
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
```

`CMakeLists.txt` 不能只保留原来的 `test_MoveIt`，还要额外加入这个可执行文件：

```cmake
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(commander_template src/commander_template.cpp)

ament_target_dependencies(
  commander_template
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  geometry_msgs
  moveit_msgs
  tf2
)

install(
  TARGETS
    test_moveit
    commander_template
  DESTINATION lib/${PROJECT_NAME}
)
```

`package.xml` 也补这几行：

```text
<depend>example_interfaces</depend>
<depend>geometry_msgs</depend>
<depend>moveit_msgs</depend>
<depend>tf2</depend>
```

编译并运行：

```bash
cd ~/桌面/ros2_ws
colcon build --packages-select my_robot_commander_cpp
source install/setup.bash

ros2 run my_robot_commander_cpp commander_template
```

打开夹爪：

```bash
ros2 topic pub --once open_gripper example_interfaces/msg/Bool "{data: true}"
```

关闭夹爪：

```bash
ros2 topic pub --once open_gripper example_interfaces/msg/Bool "{data: false}"
```

注意：`"arm"`、`"gripper"`、`"gripper_open"`、`"gripper_closed"` 都必须和你自己的 MoveIt 配置中的 planning group 与 Named Target 名字完全一致。`create_subscription()` 会在收到对应类型和话题名的消息时调用回调函数；MoveGroupInterface 则负责设置目标、规划与执行轨迹。

**这个代码是干什么的：**它把机械臂和夹爪的 MoveIt 控制封装进 `Commander` 类，并通过 `open_gripper` 话题让外部程序发送 `true` 或 `false` 来自动打开或关闭夹爪。

**相比之前添加了什么：**新增了夹爪开闭函数和一个 ROS 2 订阅者，因此控制夹爪不再只能在代码里直接调用，而可以由其他节点或终端话题消息触发。

![image-20260618175146411](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260618175146411.png)



#### 5.7 创建两个新的话题订阅者（给机械臂创建通信接口——正向和逆向运动学）

关节订阅者是用的ROS 2的接口定义的，姿态订阅者是自定义接口，这些都可以变，只是演示一些可行的定义方法，代码可以有多种写法，所以你需要知道的不是代码怎么写，而是干了什么为什么要这么干

##### 5.7.1 正向运动学话题订阅控制（joint）

继续编写my_robot_commander_cpp/src/commander_template.cpp，先添加关节订阅者

原来的 `goToJointTarget()` 函数只是一个“内部能力”，相当于机械臂节点里有“根据六个关节角运动”的功能。ROS 2 不会自动知道：收到 `/joint_command` 后要调用这个函数。必须自己写出这条连接：

```text
/joint_command 话题
        ↓
joint_cmd_sub_ 订阅者
        ↓
jointCmdCallback 回调函数
        ↓
goToJointTarget()
        ↓
MoveIt 规划并执行
```

ROS 2 中，节点必须通过 `create_subscription()` 显式注册订阅者，并绑定回调函数；收到对应话题消息后，才会执行回调。

```cpp
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;

using namespace std::placeholders;

class Commander
{
public:
    Commander(const std::shared_ptr<rclcpp::Node>& node)
    {
        node_ = node;

        // 创建机械臂规划组
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        // 创建夹爪规划组
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        // 订阅夹爪控制话题
        // true：打开夹爪
        // false：关闭夹爪
        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper",
            10,
            std::bind(
                &Commander::openGripperCallback,
                this,
                _1));

        // 订阅机械臂关节角控制话题
        // 消息中必须包含 6 个关节角度
        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command",
            10,
            std::bind(
                &Commander::jointCmdCallback,
                this,
                _1));
    }

    // 前往 MoveIt 中保存的命名姿态，例如 pose_1、home
    void goToNamedTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);

        planAndExecute(arm_);
    }

    // 前往指定的六个关节角度
    void goToJointTarget(const std::vector<double>& joints)
    {
        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);

        planAndExecute(arm_);
    }

    // 前往指定末端位姿
    // false：普通规划
    // true：笛卡尔路径规划，末端尽量沿直线运动
    void goToPoseTarget(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        bool cartesian_path = false)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;

        target_pose.header.frame_id = arm_->getPlanningFrame();

        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        // 普通位姿规划
        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);

            planAndExecute(arm_);

            arm_->clearPoseTargets();
        }
        // 笛卡尔路径规划
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;

            double fraction = arm_->computeCartesianPath(
                waypoints,
                0.01,
                0.0,
                trajectory,
                true,
                &error_code);

            if (fraction >= 0.999)
            {
                auto result = arm_->execute(trajectory);

                if (result != moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Cartesian path execution failed!");
                }
            }
            else
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Cartesian path only completed %.1f%%",
                    fraction * 100.0);
            }
        }
    }

    // 打开夹爪
    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");

        planAndExecute(gripper_);
    }

    // 关闭夹爪
    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");

        planAndExecute(gripper_);
    }

private:
    // 普通规划与执行的公共函数
    void planAndExecute(
        const std::shared_ptr<MoveGroupInterface>& interface)
    {
        MoveGroupInterface::Plan plan;

        bool success =
            (interface->plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            auto result = interface->execute(plan);

            if (result != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "Trajectory execution failed!");
            }
        }
        else
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Motion planning failed!");
        }
    }

    // 收到 open_gripper 消息时触发
    void openGripperCallback(const Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Opening gripper...");

            openGripper();
        }
        else
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Closing gripper...");

            closeGripper();
        }
    }

    // 收到 joint_command 消息时触发
    void jointCmdCallback(const FloatArray::SharedPtr msg)
    {
        std::vector<double> joints(
            msg->data.begin(),
            msg->data.end());

        // 你的机械臂是 6 自由度，因此必须给出 6 个角度
        if (joints.size() != 6)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "joint_command needs 6 values, but received %zu values.",
                joints.size());

            return;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Received 6 joint values. Planning motion...");

        goToJointTarget(joints);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    // 保存订阅对象，保证订阅关系持续存在
    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");

    // 让 ROS 通信和 MoveIt 过程能够同时处理
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    auto commander = std::make_shared<Commander>(node);

    RCLCPP_INFO(
        node->get_logger(),
        "Commander is ready.");

    RCLCPP_INFO(
        node->get_logger(),
        "Waiting for topics: open_gripper and joint_command.");

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
```

```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

```bash
ros2 run my_robot_commander_cpp commander_template
```

运行节点后，可以在另一个终端发送 6 个关节目标：

```bash
ros2 topic pub --once /joint_command \
example_interfaces/msg/Float64MultiArray \
"{data: [1.5, 0.5, 0.0, 1.5, 0.0, -0.7]}"
```

![image-20260622102357268](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260622102357268.png)

夹爪仍然这样控制：

```bash
ros2 topic pub --once /open_gripper \
example_interfaces/msg/Bool \
"{data: true}"
ros2 topic pub --once /open_gripper \
example_interfaces/msg/Bool \
"{data: false}"
```

`Float64MultiArray` 在 ROS 2 Humble 的 `example_interfaces` 中包含一个 `data` 数组，因此这里将它的 6 个数依次作为六个关节的目标角度；这种通用数组消息适合当前练习和原型验证。

**这个代码是干什么的：**它是一个 MoveIt 控制节点，可接收 ROS 2 话题消息来控制机械臂运动和夹爪开闭。

**相比之前添加了什么：**新增了 `joint_command` 订阅者，使外部节点或终端能发送六个关节角度，触发 MoveIt 规划并执行机械臂运动。



##### 5.7.2 逆向运动学话题订阅控制（末端 pose）

自定义接口配置姿态订阅者（逆向运动学）：主要就是学习怎么自定义消息接口（通俗说就是自己定义数据类型我理解的是）

很多时候系统自带的消息接口不能满足我们的需求，所以我们要自定义消息接口来发送我们特定的信息结构，这节主要就是来完成一次自定义消息接口，通过话题来订阅这个接口信息来控制机械臂。当然上面使用系统的接口是为了演示，工程项目里建议全部创建自定义接口，避免混用或者接口占用等情况，要有系统性，下面一起来学习创建接口吧

创建接口包

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ cd src
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ ros2 pkg create my_robot_interfaces
going to create a new package
package name: my_robot_interfaces
destination directory: /home/rachel/桌面/ros2_ws/src
package format: 3
version: 0.0.0
description: TODO: Package description
maintainer: ['rachel <3246689874@qq.com>']
licenses: ['TODO: License declaration']
build type: ament_cmake
dependencies: []
creating folder ./my_robot_interfaces
creating ./my_robot_interfaces/package.xml
creating source and include folder
creating folder ./my_robot_interfaces/src
creating folder ./my_robot_interfaces/include/my_robot_interfaces
creating ./my_robot_interfaces/CMakeLists.txt

[WARNING]: Unknown license 'TODO: License declaration'.  This has been set in the package.xml, but no LICENSE file has been created.
It is recommended to use one of the ament license identitifers:
Apache-2.0
BSL-1.0
BSD-2.0
BSD-2-Clause
BSD-3-Clause
GPL-3.0-only
LGPL-3.0-only
MIT
MIT-0
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src$ cd my_robot_interfaces/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_interface
s$ ls
CMakeLists.txt  include  package.xml  src
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_interface
s$ rm -r include/ src/
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_interface
s$ mkdir msg
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws/src/my_robot_interface
s$ ls
CMakeLists.txt  msg  package.xml
```

增加依赖my_robot_interfaces/package.xml：

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_interfaces</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="3246689874@qq.com">rachel</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

修改my_robot_interfaces/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PoseCommand.msg"
 )

ament_export_dependencies(rosidl_default_runtime)

ament_package()
```

定义消息my_robot_interfaces/msg/PoseCommand.msg:对应goToPoseTarget函数的参数

```text
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
bool cartesian_path
```

```bash
colcon build --packages-select my_robot_interfaces
source install/setup.bash
ros2 interface show my_robot_interfaces/msg/PoseCommand
```



###### 小插曲（没有就跳过）

小插曲：构建失败的直接原因是你的工作空间路径含有中文“桌面”，ROS 2 的 `rosidl` 在生成 `PoseCommand.msg` 的中间 `.idl` 文件时把这段 Unicode 路径解析坏了，导致后续去错误的 `/ros2_ws/...` 路径寻找文件。

建议后面不要把工作空间放在有中文的路径下，我后面将移动工作空间到主目录下，并软连接到桌面相当于是个快捷方式

可以保留“桌面上的入口”，但**实际工作空间要移到纯英文路径**。我的报错和已公开的同类 Humble `rosidl` 报错一致：接口生成器把 `.msg` 路径处理为“绝对路径:相对路径”形式，而我的绝对路径中含 `桌面`，最终导致生成的 `.idl` 文件路径错乱。

最稳妥的做法：

```bash
# 先确认 home 下没有同名 ros2_ws
ls -ld ~/ros2_ws ~/桌面/ros2_ws
```

若提示 `~/ros2_ws` 不存在，再执行：

```bash
mv ~/桌面/ros2_ws ~/ros2_ws
```

这样真实路径变为：

```text
/home/rachel/ros2_ws
```

然后给桌面建立一个快捷入口，桌面上仍然能看到并双击打开：

```text
ln -s ~/ros2_ws ~/桌面/ros2_ws
```

以后有一条原则：

```text
桌面上的 ros2_ws：只是快捷方式
终端构建、运行、source：一律使用 ~/ros2_ws
```

接着修改 `.bashrc` 中原来的这行：

```bash
source "$HOME/桌面/ros2_ws/install/setup.bash"
```

改为：

```bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
```

最后重新构建：

```bash
source /opt/ros/humble/setup.bash

cd ~/ros2_ws

rm -rf build install log

colcon build --packages-select my_robot_interfaces
```

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/桌面/ros2_ws$ source /opt/ros/humble/setup.bash

cd ~/ros2_ws

rm -rf build install log

colcon build --packages-select my_robot_interfaces
Starting >>> my_robot_interfaces
Finished <<< my_robot_interfaces [4.07s]                     

Summary: 1 package finished [4.36s]
```

成功后验证：

```bash
source install/setup.bash

ros2 interface show my_robot_interfaces/msg/PoseCommand
```

应输出：

```text
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
bool cartesian_path
```

不要再从 `~/桌面/ros2_ws` 进入终端执行 `colcon build`；即使它看起来是同一个文件夹，也可能再次把中文路径带入构建过程。

![image-20260622113617297](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260622113617297.png)



找路径添加到.vscode文件夹下的下面文件位置，避免后续的编译运行错误

```bash
rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws$ cd src/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws/src$ code .

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws/src$ cd ..

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws$ cd install/my_robot_interfaces/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws/install/my_robot_interfaces$ ls
include  lib  local  share

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws/install/my_robot_interfaces$ cd include/

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws/install/my_robot_interfaces/include$ ls
my_robot_interfaces

rachel@rachel-Lenovo-XiaoXinAir-15ALC-2021:~/ros2_ws/install/my_robot_interfaces/include$ pwd
/home/rachel/ros2_ws/install/my_robot_interfaces/include
```

在.vscode/c_cpp_properties.json添加路径

![image-20260622114733443](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260622114733443.png)



继续修改my_robot_commander_cpp/src/commander_template.cpp，增加位姿订阅者

```cpp
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>

#include <my_robot_interfaces/msg/pose_command.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using MoveGroupInterface =
    moveit::planning_interface::MoveGroupInterface;

using Bool = example_interfaces::msg::Bool;
using FloatArray = example_interfaces::msg::Float64MultiArray;
using PoseCmd = my_robot_interfaces::msg::PoseCommand;

using namespace std::placeholders;

class Commander
{
public:
    Commander(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node)
    {
        // 创建机械臂规划组
        arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
        arm_->setMaxVelocityScalingFactor(1.0);
        arm_->setMaxAccelerationScalingFactor(1.0);

        // 创建夹爪规划组
        gripper_ = std::make_shared<MoveGroupInterface>(node_, "gripper");

        // 订阅夹爪控制话题
        // true：打开夹爪
        // false：关闭夹爪
        open_gripper_sub_ = node_->create_subscription<Bool>(
            "open_gripper",
            10,
            std::bind(
                &Commander::openGripperCallback,
                this,
                _1));

        // 订阅关节角控制话题
        // 消息中必须包含 6 个关节角度
        joint_cmd_sub_ = node_->create_subscription<FloatArray>(
            "joint_command",
            10,
            std::bind(
                &Commander::jointCmdCallback,
                this,
                _1));

        // 订阅末端位姿控制话题
        // 消息类型：my_robot_interfaces/msg/PoseCommand
        pose_cmd_sub_ = node_->create_subscription<PoseCmd>(
            "pose_command",
            10,
            std::bind(
                &Commander::poseCmdCallback,
                this,
                _1));
    }

    // 前往 MoveIt 中保存的命名姿态，例如 home、pose_1
    void goToNamedTarget(const std::string& name)
    {
        arm_->setStartStateToCurrentState();
        arm_->setNamedTarget(name);

        planAndExecute(arm_);
    }

    // 前往指定的六个关节角度
    void goToJointTarget(const std::vector<double>& joints)
    {
        if (joints.size() != 6)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Joint target requires 6 values, but received %zu.",
                joints.size());
            return;
        }

        arm_->setStartStateToCurrentState();
        arm_->setJointValueTarget(joints);

        planAndExecute(arm_);
    }

    // 前往指定末端位姿
    // cartesian_path = false：普通路径规划
    // cartesian_path = true ：笛卡尔路径规划，末端尽量直线运动
    void goToPoseTarget(
        double x,
        double y,
        double z,
        double roll,
        double pitch,
        double yaw,
        bool cartesian_path = false)
    {
        // 将 RPY 欧拉角转换为四元数
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();

        geometry_msgs::msg::PoseStamped target_pose;

        // 通常是 base_link
        target_pose.header.frame_id = arm_->getPlanningFrame();

        target_pose.pose.position.x = x;
        target_pose.pose.position.y = y;
        target_pose.pose.position.z = z;

        target_pose.pose.orientation.x = q.getX();
        target_pose.pose.orientation.y = q.getY();
        target_pose.pose.orientation.z = q.getZ();
        target_pose.pose.orientation.w = q.getW();

        arm_->setStartStateToCurrentState();

        // 普通位姿规划
        if (!cartesian_path)
        {
            arm_->setPoseTarget(target_pose);

            planAndExecute(arm_);

            arm_->clearPoseTargets();
        }
        // 笛卡尔路径规划
        else
        {
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(target_pose.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            moveit_msgs::msg::MoveItErrorCodes error_code;

            double fraction = arm_->computeCartesianPath(
                waypoints,
                0.01,
                0.0,
                trajectory,
                true,
                &error_code);

            if (fraction >= 0.999)
            {
                auto result = arm_->execute(trajectory);

                if (result != moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "Cartesian path execution failed!");
                }
            }
            else
            {
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Cartesian path only completed %.1f%%.",
                    fraction * 100.0);
            }
        }
    }

    // 打开夹爪
    void openGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_open");

        planAndExecute(gripper_);
    }

    // 关闭夹爪
    void closeGripper()
    {
        gripper_->setStartStateToCurrentState();
        gripper_->setNamedTarget("gripper_closed");

        planAndExecute(gripper_);
    }

private:
    // 规划成功后再执行
    void planAndExecute(
        const std::shared_ptr<MoveGroupInterface>& move_group)
    {
        MoveGroupInterface::Plan plan;

        bool success =
            (move_group->plan(plan) ==
             moveit::core::MoveItErrorCode::SUCCESS);

        if (!success)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Motion planning failed!");
            return;
        }

        auto result = move_group->execute(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Trajectory execution failed!");
        }
    }

    // 收到 open_gripper 消息时触发
    void openGripperCallback(const Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Opening gripper...");

            openGripper();
        }
        else
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "Closing gripper...");

            closeGripper();
        }
    }

    // 收到 joint_command 消息时触发
    void jointCmdCallback(const FloatArray::SharedPtr msg)
    {
        std::vector<double> joints(
            msg->data.begin(),
            msg->data.end());

        if (joints.size() != 6)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "joint_command requires 6 values, but received %zu.",
                joints.size());
            return;
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "Received joint command.");

        goToJointTarget(joints);
    }

    // 收到 pose_command 消息时触发
    void poseCmdCallback(const PoseCmd::SharedPtr msg)
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "Received pose command: x=%.3f, y=%.3f, z=%.3f, "
            "roll=%.3f, pitch=%.3f, yaw=%.3f, cartesian=%s",
            msg->x,
            msg->y,
            msg->z,
            msg->roll,
            msg->pitch,
            msg->yaw,
            msg->cartesian_path ? "true" : "false");

        goToPoseTarget(
            msg->x,
            msg->y,
            msg->z,
            msg->roll,
            msg->pitch,
            msg->yaw,
            msg->cartesian_path);
    }

private:
    std::shared_ptr<rclcpp::Node> node_;

    std::shared_ptr<MoveGroupInterface> arm_;
    std::shared_ptr<MoveGroupInterface> gripper_;

    rclcpp::Subscription<Bool>::SharedPtr open_gripper_sub_;
    rclcpp::Subscription<FloatArray>::SharedPtr joint_cmd_sub_;
    rclcpp::Subscription<PoseCmd>::SharedPtr pose_cmd_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("commander");

    // MoveIt 执行、话题回调可同时处理
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    auto commander = std::make_shared<Commander>(node);

    RCLCPP_INFO(
        node->get_logger(),
        "Commander is ready. Waiting for open_gripper, "
        "joint_command and pose_command topics...");

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(100));
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}
```

还必须确认 `my_robot_commander_cpp` 能找到你的自定义消息包。否则会报：

```text
my_robot_interfaces/msg/pose_command.hpp: No such file or directory
```

在 `my_robot_commander_cpp/CMakeLists.txt` 中加入：

```cmake
find_package(my_robot_interfaces REQUIRED)
```

并在 `ament_target_dependencies(...)` 中加入：

```text
my_robot_interfaces
```

在 `my_robot_commander_cpp/package.xml` 中加入：

```text
<depend>my_robot_interfaces</depend>
```

自定义消息包需先成功构建，之后才能被控制节点包含和使用。开三个终端：

```bash
ros2 launch my_robot_bringup my_robot.launch.xml
```

```bash
ros2 run my_robot_commander_cpp commander_template
```

构建成功后，位姿话题可这样测试：

```bash
ros2 topic pub --once /pose_command \
my_robot_interfaces/msg/PoseCommand \
"{x: 0.0, y: -0.7, z: 0.4, roll: 3.14, pitch: 0.0, yaw: 0.0, cartesian_path: false}"
```

其中 `cartesian_path: false` 是普通 MoveIt 位姿规划；改为 `true` 时会调用笛卡尔路径规划。

![image-20260622160440219](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260622160440219.png)



#### 5.8 以上述例子拓展补充 ROS 2 的基础理解

把 ROS 2 想成一个“很多小程序一起协作的机器人系统”。

```text
传感器程序、机械臂程序、MoveIt、夹爪程序、相机程序
不是直接互相调用
而是通过 ROS 的通信机制交换信息
```

##### 先记住这 5 个词

| 名词                        | 你可以先理解成                   | 你的项目里的例子                                   |
| --------------------------- | -------------------------------- | -------------------------------------------------- |
| 节点 Node（节点）           | 一个独立的小程序，负责一件事     | `commander`、`move_group`、`robot_state_publisher` |
| 话题 Topic（话题）          | 一个公开的信息频道               | `/joint_command`、`/open_gripper`、`/joint_states` |
| 发布者 Publisher（发布者）  | 往频道里发消息的人               | 终端中的 `ros2 topic pub`                          |
| 订阅者 Subscriber（订阅者） | 监听某个频道、收到消息后做事的人 | `eg：Commander` 中的 `joint_cmd_sub_`              |
| 消息 Message（消息）        | 频道里传输的具体数据格式         | 六个关节角数组、`true/false` 等                    |

最常见的关系就是：

```text
发布者
   ↓ 发消息
话题 /joint_command
   ↓ 传递消息
订阅者
   ↓ 收到后执行回调函数
Commander
   ↓
MoveIt 规划机械臂运动
```

ROS 2 的话题采用“发布—订阅”模式：发布者和订阅者不需要彼此直接认识，只要话题名称和消息类型匹配即可通信。

------

##### 话题、服务、动作怎么区分

###### 1. Topic（话题）：持续发消息

适合不断出现的数据或简单指令。

```text
相机不断发布图像
机械臂不断发布关节状态
你发一次 /joint_command 关节目标
```

特点是：**只负责发送数据，不保证对方一定完成任务，也不专门返回结果。**

例子：

```bash
ros2 topic pub --once /joint_command ...
```

这里终端是发布者，`Commander` 是订阅者。

------

###### 2. Service（服务）：问一次，答一次

适合“立刻能完成”的请求。

```text
请求：机械臂当前是否使能？
回应：是 / 否

请求：读取当前关节角？
回应：六个角度
```

逻辑是：

```text
客户端 Client：提问
        ↓
服务端 Server：立即处理
        ↓
客户端：拿到回答
```

可以把它理解成“打电话问一个明确的问题，立刻得到答复”。服务适用于短时、同步的请求—响应任务。

------

###### 3. Action（动作）：任务较久，有进度，可取消

适合机械臂执行轨迹、导航到目标点、抓取等需要时间的任务。

```text
目标 Goal：机械臂移动到某个位置
反馈 Feedback：已完成 30%、60%、90%
结果 Result：成功 / 失败
取消 Cancel：中途停止
```

可以把 Action 理解成：

```text
不是“发一句命令就结束”
而是“下达一个任务，并持续跟踪它”
```

MoveIt 执行轨迹时，底层常用 `FollowJointTrajectory` Action。MoveIt 发送“执行这一条轨迹”的目标，控制器持续执行，最后返回成功或失败。ROS 2 的 Action 就是专门用于这类长时间、需要反馈和可取消的任务。

------





#### 5.9 本节小结

本章主要学习如何使用 MoveIt C++ API 创建 `Commander` 控制节点。控制方式可以先分成两类理解：一类是基于关节角的控制，也就是直接给出六个关节的目标值；另一类是基于末端位姿的控制，也就是给出末端执行器的位置和姿态，由 MoveIt 进行逆运动学（IK）求解。

第 5.2 到 5.4 节中的代码更适合作为小范围测试：它们能帮助我们确认 MoveIt API、命名姿态、关节目标、末端位姿和笛卡尔路径是否可用。第 5.5 到 5.7 节则逐步把这些能力封装成更接近工程使用的 `Commander` 类，并通过话题订阅者给外部节点提供控制入口。

对于初学者来说，重点不只是记住每一行 C++，而是理解工程结构：哪些功能应该放在 MoveIt 配置中，哪些逻辑应该封装在控制节点里，外部任务又应该通过什么接口把目标发送进来。我觉得不会写复杂代码现在不是大问题，但是我们要培养工程思维，知道对于一个项目应该怎么做，把握好大方向，代码这些问题完全可以交给AI来写的。





### 6 使用 MoveIt Python API（Humble 环境暂不展开）

MoveIt Python API 主要指 `MoveIt_py`，用于在 Python 中调用 MoveIt 的规划能力，例如设置关节目标或末端位姿目标、进行碰撞约束下的路径规划，并执行生成的机械臂轨迹。

`MoveIt_py` 本质上是对 MoveIt C++ 核心接口的 Python 封装，适合快速验证、脚本编写和高层任务逻辑开发；但其接口覆盖、示例资料、底层调试与复杂扩展能力通常仍弱于 C++ API。因此，在复杂工程、控制链路和深度定制场景中，C++ 仍更常用。

对于 ROS 2 Humble，官方 `MoveIt_py` 未提供可直接通过 apt 安装的二进制包，且 Humble 分支未完整包含该功能，因此本项目暂不使用。ROS 2 Jazzy 对应的 MoveIt 二进制软件包已包含 `ros-jazzy-MoveIt-py`；后续切换到 Ubuntu 24.04 与 ROS 2 Jazzy 时，可再学习该接口。

注意使用`sudo install ros-humble-MoveIt-py`是安装不上的， 因为humble就没有这个接口

补充一个直观理解：

```text
C++ MoveGroupInterface：
更完整、更稳定，适合现在写的 commander 节点。

moveit_py：
把部分 MoveIt C++ 能力包装成 Python，
写起来更快，但不是所有 C++ 功能都能一一对应。
```



有个疑问：那可不可以说，Python其实不适合来编写ROS 2节点，还是C++适合做底层？然后Python可以用来编写脚本简单，但是ROS 2还是适合用C++来写？

解答：

**Python 完全可以编写 ROS 2 节点，适合算法验证、视觉处理、数据处理和任务脚本；但涉及机械臂底层控制、实时性、MoveIt 深度调用、复杂并发和工程部署时，通常更适合用 C++。**

原因是 ROS 2 同时官方支持 `rclpy` 和 `rclcpp`，两者都能创建节点、话题、服务、动作，不是 Python “不能写 ROS 2 节点”。

可以先按这个分工记：

```text
Python：
视觉算法、AI 模型、数据处理、快速验证、高层任务逻辑

C++：
机械臂控制节点、MoveIt 规划调用、控制器交互、实时性要求高的核心模块
```

对我们现在这个项目：

```text
Commander 节点、MoveIt、轨迹执行：优先 C++
后续视觉识别、抓取目标坐标计算：可以 Python
Python 算出目标位姿 → 发布 pose_command → C++ Commander 执行机械臂
```

这才是比较常见的工程组合。MoveIt 官方也明确把 Python API定位为快速原型和实验用途，并说明它只绑定了部分 C++ API。



### 7 MoveIt 与硬件连接

本节是对控制链路的理解补充，不表示本项目已经接入真实机械臂。当前工程使用的是 `FakeSystem`，它用于模拟硬件接口的命令接收和状态反馈。

![image-20260622170713881](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260622170713881.png)

这张图表达的是：**上层 ROS 2 节点负责提出任务，MoveIt 负责规划，ros2_control 负责把轨迹真正下发到硬件，同时硬件状态再反馈回来。**

可以配下面这段说明放进文档：

> 在实际机械臂系统中，其他 ROS 2 节点可以通过话题、服务或动作向 Commander 节点发送关节目标、末端位姿或抓取任务。Commander 节点通过 MoveIt API 调用 MoveIt 2，完成运动学计算、碰撞检测和轨迹规划。规划得到的关节轨迹随后通过 MoveIt 控制器发送给 ros2_control 中的轨迹控制器，由其进一步转换为对真实机械臂或仿真硬件的控制指令。硬件执行过程中持续返回关节位置、速度等状态信息，ros2_control 将这些状态发布为关节状态，robot_state_publisher 再结合 URDF 模型发布 TF 坐标变换，供 MoveIt 和 RViz 获取机器人当前姿态。

简化记成两条链路：

```text
控制链路：
其他节点 → Commander → MoveIt → ros2_control → Hardware

状态反馈链路：
Hardware → ros2_control → joint_states → robot_state_publisher → TF → MoveIt / RViz
```

补充说明：
 **MoveIt 并不是直接控制硬件，而是通常通过 `FollowJointTrajectory` 动作把轨迹交给 ros2_control 中的 JointTrajectoryController。** `robot_state_publisher` 也不参与控制，它只根据 `/joint_states` 和 URDF 发布机器人各连杆的 TF 坐标。

硬件接口可以理解为 ROS 2 与真实机械臂之间的翻译层：它负责将上层控制指令转换为硬件能够执行的命令，并将机械臂传回的关节状态转换为 ROS 2 可读取的数据。

![image-20260622170726972](基于ROS2与Moveit2的机械臂控制项目.assets/image-20260622170726972.png)

从左到右看：

```text
MoveIt → Controller → HW interface → HW driver → Hardware
```

1. MoveIt：负责想好“怎么动”

MoveIt 算出一条轨迹，例如：

```text
joint1 从 0 转到 1.0 rad
joint2 从 0 转到 0.5 rad
```

它只负责规划，不直接控制电机。

2. Controller：负责按轨迹下命令

这里的 Controller 就是 `arm_controller`、`gripper_controller` 这类控制器。

它收到 MoveIt 的轨迹后，会不断给每个关节发目标，例如：

```text
现在 joint1 应该到 0.2
下一时刻应该到 0.25
再下一时刻应该到 0.3
```

Controller 和硬件接口都属于 `ros2_control` 的范围。`controller_manager` 的作用就是把控制器和硬件抽象部分连接起来。

3. HW interface：ROS 与厂家驱动之间的翻译层

它把 Controller 给出的通用 ROS 指令：

```text
joint1 目标位置 = 1.0 rad
```

转换成硬件驱动需要的数据；同时把硬件返回的关节角度、速度等数据转换回 ROS 能理解的格式。

现在使用的 `FakeSystem`，本质上就是这个位置的“假硬件接口”。

```text
Controller → FakeSystem → RViz 中的模型运动
```

真实硬件接口则会真正读取编码器状态、发送关节控制命令。ros2_control 将这类硬件部分抽象为硬件组件，用于表示真实执行器、传感器或完整机器人系统。

4. HW driver：厂家提供的底层驱动

这是更靠近真实机械臂的一层，通常由厂家提供。

它负责真正通过：

```text
网口、串口、CAN 总线、厂商 SDK
```

与机械臂控制柜或电机驱动器通信。

例如：

```text
HW interface：joint1 转到 1.0 rad
↓
HW driver：转换成厂家协议的数据包
↓
真实控制柜、电机驱动器
```

5. Hardware：真实机械臂

包括机械臂本体、电机、减速器、编码器、控制柜等。

> MoveIt 负责生成机械臂运动轨迹，但不直接向电机发送指令。轨迹首先交由 ros2_control 中的 Controller 进行执行管理，再通过硬件接口转换为 ROS 2 与具体机械臂之间通用的关节命令和状态数据。随后，硬件驱动按照不同厂商的通信协议，将命令发送至真实机械臂控制柜，并读取编码器等硬件反馈信息。通过这种分层结构，上层 MoveIt 规划逻辑通常不需要因机械臂型号变化而大幅修改，只需替换底层硬件接口和驱动部分。

最简单地记：

```text
MoveIt：想怎么动
Controller：按轨迹安排每一步
HW interface：ROS 语言和硬件语言之间翻译
HW driver：按厂商协议发数据
Hardware：真实机械臂执行
```



本节先理解到分层关系即可。真实硬件控制需要具体机械臂、驱动器、控制柜和通信协议支持，不能仅凭本项目中的 `FakeSystem` 推断已经完成实机控制。后续接入实机时，应重点核查厂商协议、硬件接口实现、安全限位、急停逻辑和上电调试流程。后续我会继续做连接仿真器里的机械臂，可以期待一下。





## 四.项目总结

## 项目总结

本项目围绕“**如何让一个 ROS 2 机械臂从模型描述走到可规划、可执行、可通信控制”的完整过程**展开，搭建了六自由度机械臂与夹爪的 MoveIt 2 仿真控制系统。项目的核心不是单独学习某一个软件包，而是理解机器人控制中建模、状态发布、运动规划、轨迹执行和外部任务指令之间的连接关系。

首先，使用 URDF 和 Xacro 描述机械臂及夹爪的连杆、关节、运动范围、碰撞模型和安装关系，形成机器人的**数字化结构模型**。该模型回答的是“机器人由什么组成、每个关节能怎样运动、末端执行器位于哪里”等问题。随后，robot_state_publisher 根据 URDF 模型和关节状态发布 TF 坐标变换，RViz 据此显示机械臂当前姿态。因此，URDF 负责定义结构，joint_states 反映当前关节状态，TF 和 RViz 则负责将状态可视化。

在**运动规划层**，利用 MoveIt Setup Assistant 配置机械臂规划组、夹爪规划组、末端执行器、命名姿态、碰撞矩阵、关节约束和控制器映射。MoveIt 的作用是根据机器人模型、目标状态和碰撞约束，计算一条可行的关节运动轨迹。它并不直接驱动电机，而是负责回答“机械臂应该以什么路径运动，才能安全到达目标”。

在**轨迹执行层**，项目使用 ros2_control 建立控制器与硬件之间的标准连接。controller_manager 负责管理控制器，joint_state_broadcaster 负责发布关节状态，arm_controller 和 gripper_controller 负责接收轨迹并执行关节控制。当前系统采用 FakeSystem 作为仿真硬件，它不连接真实电机，而是模拟硬件接收命令和返回状态的过程。因此，MoveIt 规划出的轨迹能够在 RViz 中执行并更新机器人姿态。后续接入真实机械臂时，主要需要将 FakeSystem 替换为对应厂商的硬件接口和驱动，上层的规划与 Commander 控制逻辑可以尽量保持不变。

在**上层任务控制**部分，项目编写了基于 MoveIt C++ API 的 Commander 节点。该节点将机械臂控制封装为命名姿态控制、关节角控制、末端位姿控制、笛卡尔路径控制以及夹爪开闭控制等功能，并通过 ROS 2 话题接收外部指令。外部节点可以发布关节目标、夹爪开闭指令或末端位姿目标；Commander 接收到消息后调用 MoveGroupInterface 设置目标、请求 MoveIt 规划并执行轨迹。这样，上层视觉、任务规划或人工交互模块不必直接操作控制器，只需向 Commander 发送任务目标即可。

整个项目的控制链路可以概括为：

```text
外部任务节点或终端
→ 发布关节、位姿或夹爪控制指令
→ Commander 节点接收并调用 MoveIt C++ API
→ MoveIt 根据模型、约束和碰撞信息规划轨迹
→ MoveIt 将轨迹发送给 ros2_control 中的控制器
→ arm_controller 或 gripper_controller 控制 FakeSystem
→ FakeSystem 返回关节状态
→ joint_state_broadcaster 发布 joint_states
→ robot_state_publisher 根据 URDF 发布 TF
→ RViz、MoveIt 获取机器人当前姿态并完成可视化。
```

通过该项目，主要学习了 ROS 2 功能包组织、URDF/Xacro 建模、TF 坐标关系、RViz 可视化、MoveIt 规划配置、ros2_control 控制器机制、FakeSystem 仿真硬件、Launch 文件统一启动、ROS 2 话题通信、自定义消息接口以及 MoveIt C++ API 的使用方法。**更重要的是，建立了对机器人控制系统分层结构的整体认识：URDF 定义机器人，MoveIt 负责规划，ros2_control 负责执行，硬件接口连接真实设备，ROS 2 节点负责产生和传递任务指令。后续可在该框架上接入视觉识别、抓取任务规划、真实机械臂驱动和传感器反馈模块，逐步扩展为完整的智能抓取系统。**

本项目最重要的收获是建立机器人控制系统的分层认识：算法、规划、控制器、硬件接口和真实设备分别处在不同位置。后续无论继续做抓取算法、视觉识别、任务规划，还是接入真实机械臂，都应先明确新模块要接在哪一层、输入输出是什么、会影响哪条链路。

最重要的是框架的认识，这是一个控制机器人的底层框架，后续你要知道做论文也好，做创新也好，做的是哪部分，加在哪个位置，这才是最重要的。上层的是算法，但是你不能不清楚算法是在哪个环节起作用，不能只知道创新算法但是不知道为什么创新！

本教程中的机械臂模型相对简化，适合用于理解 ROS 2、MoveIt 2 和 ros2_control 的基本工作流。工业级机械臂还会涉及更完整的动力学参数、厂商驱动、标定、安全策略和实机调试流程，这些内容需要结合具体硬件继续扩展。后续有机会再分享更多复杂的机械臂怎么玩，我也才刚起步，希望与大家一起探索更多有趣的知识！











