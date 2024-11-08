# boxer_ros2
[![ubuntu22](https://img.shields.io/badge/-UBUNTU_22.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/jammy/)
[![humble](https://img.shields.io/badge/-HUMBLE-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/index.html)

## Overview
ROS2 packages for Boxer

### Install packages
```shell
cd ~/colcon_ws/src/
git clone https://github.com/roasinc/boxer_ros2.git
```

> [!IMPORTANT]  
> Install on the backpack computer.

### Build
```shellll --from-paths src --ignore-src -y
cd ~/colcon_ws/
rosdep insta
colcon build --symlink-install
```

## Usage
```
ros2 launch boxer_bringup bringup_launch.py
```