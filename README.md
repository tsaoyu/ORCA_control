# ORCA_control

## ORCA control packages

High quality ROS package for underwater robotics control and navigation.

## Quality assurance

![](https://github.com/tsaoyu/ORCA_control/workflows/CI/badge.svg)

## Contents

- ROS driver for BlueROV2
- Reconfigurable PID position/velocity controller
- Manual Joystick control
- Offline Git utilities

For more advanced position controllers please refer to ORCA advanced control project. 

## Basic usage
A common use case of this repository is the station keeping and way point following of BlueROV2.
The basic assumption on the hardware is that the companinon computer is running ROS and able to communicate with Pixhawk microcontroller.



```bash
roslaunch redrov_missions station_keeping.launch 
```

```bash
# ROS main development environment
source /opt/ros/melodic/setup.bash

# ROS custom workspaces
source ~/ORCA_control/redrov/devel/setup.bash

# ROS network settings

export TOP_NAME=laoge
export ROV_NAME=laodi

export ROS_HOSTNAME=laoge
export ROS_MASTER_URI=http://192.168.2.2:11311

```

## Advanced usage
A more advanced use case is the software and hardware-in-the-loop (HIL) simulation.

## Handy tools
We provide a couple of useful tools to ease the development and deployment of the control software. 


## Acknowledgement
This project is part of 
