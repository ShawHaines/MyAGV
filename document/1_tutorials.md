# ROS Tutorial

## 0.Prepare
* bash

```
ls
cd
rm
mv
export
echo
whereis/which
-----------
help/--help
man
-----------
mkdir
touch
cat
```



## 1.Installing and Configuring Your ROS Environment

* 依照[官方文档](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)，我选择的是中科大的源
* 安装[catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html) : `apt install python-catkin-tools`
  * 更友好的编译输出
  * `catkin build`与`catkin_make_isolated`相似，允许并行编译

---

> 如果你使用了虚拟机，[这里](http://www.rawinfopages.com/tips/2017/07/speed-up-virtualbox-in-windows/)有一些tips可以尽量的加速你的虚拟机。以及：
>
> * 尽量打开3D加速
> * 如果没有打开3D加速的话，gazebo尽量缩小窗口大小，这样可以减少render带来了cpu占用。

---

* 可以参考的文件管理结构：
```
└── 'ROOT_DIR' or '~'
    ├── catkin_ws          # catkin workspace
    │   ├── build          #
    │   ├── devel          #
    │   │   └── setup.bash #
    │   └── src            # src -> ../catkin_ws_backup/XXX_ws_src
    └── catkin_ws_backup   # store packages | multi-workspace backup
```
* 可以简化你的常用命令` alias rs="source devel/setup.bash"`
* `$ROS_PACKAGE_PATH`

## 2.Navigating the ROS Filesystem

* package是ROS构建代码或者工程的基本单位(区分node)
* rospack, roscd, rosls
* Tab Completion

## 3.Creating & Building a ROS Package
* `catkin_create_pkg <package_name>`
* 修改pakcage.xml里面的配置
* CMakeLists (for c++ programming)
* TASK1：使用rosrun运行一个python脚本：将脚本放在`<your_package>/scripts`文件夹
```bash
catkin build
source devel/setup.bash
roscore 
# open another console
rosrun <package_name> <script_name.py>
```

## 4.Understanding ROS Nodes & Topic
* 进程通信
* Nodes: A node is an executable that uses ROS to communicate with other nodes.
* Messages: ROS data type used when subscribing or publishing to a topic.
* Topics: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages.
* Master: Name service for ROS (i.e. helps nodes find each other)
* rosout: ROS equivalent of stdout/stderr
* roscore: Master + rosout + parameter server 

```bash
rostopic [echo|pub|list|hz|help|...]
rosnode [list|info|help|...]
rqt_graph
------------
rqt_plot
rosmsg [show]
```



## 5.Understanding ROS Services and Parameters
```
rosservice [list|call|type|find|uri|...]
rosparam [list|set|get|load|dump|delete]
rossrv
```

## 6.Using rqt_console and roslaunch
* rqt_console显示输出
* rqt_logger_level查看log
* roslaunch
  * 若没启动roscore会自动启动
  * 同时启动多个node
  * 可在launch文件中配置参数
  * launch文件应置于`<package>/launch/XXX.launch`
## 7.Using rosed to edit files in ROS
## 8.Creating a ROS msg and srv
## 9.Writing & Examining a Simple Publisher and Subscriber (Python)
## 10.Writing & Examining a Simple Service and Client (Python)
---

## 最后最后

一些比较实用的小工具

* git  // 代码的版本管理软件（可以有效的防止代码丢失）
* vim  // 远程登录只有终端的机器人时可能并没有GUI软件可以用来修改文本文件