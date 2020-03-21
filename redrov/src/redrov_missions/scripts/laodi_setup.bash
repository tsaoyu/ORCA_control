#!/bin/bash

echo export ROS_MASTER_URI=http://192.168.2.2:11311
echo export ROS_HOSTNAME=192.168.2.2
source /home/laodi/ORCA_control/redrov/devel/setup.bash
exec "$@"