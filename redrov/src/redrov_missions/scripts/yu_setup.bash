#!/bin/bash

export ROS_IP=192.168.2.2
export ROS_MASTER_URI=http://192.168.2.2:11311
source /home/yu/ORCA_control/redrov/devel/setup.bash
exec "$@"