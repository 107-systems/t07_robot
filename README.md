<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `t07_ros`
=======================
[![Build Status](https://github.com/107-systems/t07_ros/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/t07_ros/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/t07_ros/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/t07_ros/actions/workflows/spell-check.yml)

ROS control code for the [T07](https://github.com/107-systems/T07) robot.

#### How-to-build
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/t07_ros
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select t07_ros
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch t07_ros t07.py
```

#### Interface Documentation
##### Published Topics
| Default name |                                      Type                                      |
|:------------:|:------------------------------------------------------------------------------:|

##### Parameters
|                  Name                 |     Default      | Description                                                                                   |
|:-------------------------------------:|:----------------:|-----------------------------------------------------------------------------------------------|
