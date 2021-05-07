# READ ME

ROS2 implementation of Interestingness_ros (ROS1) (a ROS2 source package)



## Requirements

**Strongly suggest:** 

`conda deactivate`

- Ubuntu20.04

- CUDA11.2+cudnn

- Pytorch+torchvision (for cuda11.1)

- Opencv-python

- [ROS2 foxy_apt](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html) for rosbag_v2

- [ROS2 foxy_source](https://docs.ros.org/en/foxy/Installation/Ubuntu-Development-Setup.html) for ros1_bridge

- [ROS1 noetic_source](http://wiki.ros.org/noetic/Installation/Ubuntu)

- [rosbag2_bag_v2](https://github.com/ros2/rosbag2_bag_v2/issues) 

  can be installed through command

  `sudo apt install -y ros-foxy-rosbag2-bag-v2-plugins`

- [ros1_bridge](https://index.ros.org/p/ros1_bridge/) (not sure if necessary)

- Another source package interface [Interface](https://github.com/Jaraxxus-Me/Interface.git) in the same /src directory

- [model](https://github.com/wang-chen/interestingness/releases/download/v2.0/vgg16.pt.SubTF.n100usage.mse) in /interestingness_ros2/saves directory



## Quick Start

### Submodule

```shell
cd [to-your-ws]/src/interestingness_ros2
git submodule init
git submodule update
```

### Path

Change the rosbag path in file bags.py.

### Use rosbag2_bag_v2 (recommend)

#### In Shell 1

```shell
source /opt/ros/noetic/setup.bash #this must be the first to source
source /opt/ros/foxy/setup.bash
cd [to-your-ws]/src/interestingness_ros2/launch
python3 bags.py
```

#### In Shell 2

```shell
source /opt/ros/foxy/setup.bash
cd [to-your-ws]
colcon build
. install/setup.bash
ros2 launch interestingness_ros2 interestingness.launch.py
# may wait a second
```

