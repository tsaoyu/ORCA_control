#!/bin/bash

export ROS_IP=192.168.2.1
export ROS_MASTER_URI=http://192.168.2.2:11311
source /home/nelson/Playground/ORCA/bluerov/devel/setup.bash
exec "$@"
