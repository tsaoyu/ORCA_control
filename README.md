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
Once the referenced pose (`/pose_ref` by default) and odometry (`/BodyROV01/odom` by default) are provided,
you can launch station keeping mission from `redrov_missions`. 

```bash
roslaunch redrov_missions station_keeping.launch 
```
`ORCA_control` will take over the control of the ROV once `autonomous` mode is enabled. For safety reasons, it is recommended to use joystick to switch to `autonomous` mode by first `ARM` the vehicle and press **A** to initate the station keeping mission.  

![](images/joystick.png)

At any time, you can press `DISARM` to lock the 



## Advanced usage
A more advanced use case is the software and hardware-in-the-loop (HIL) simulation.

## Handy tools
We provide a couple of useful tools to ease the development and deployment of the control software. 

## Network configuration

The basic assumption on the hardware is that the companinon computer is running ROS and able to communicate with Pixhawk microcontroller.
The network configuration of BlueROV2 is shown in the following figure. Please refer to this [tutorial](http://wiki.ros.org/ROS/NetworkSetup) to setup the network between base computer and BlueROV2. 

![](images/network.png)

On your base computer, the `.bashrc` should at least include the following:

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

## Disclaim
The author release the code WITHOUT ANY GUARANTEE.

## Acknowledgement
This project is part of 
