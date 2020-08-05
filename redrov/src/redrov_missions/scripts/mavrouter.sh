#!/usr/bin/env bash

# Use mavlink-router to create MAVLink connections
# better CPU usage compared to mavproxy.py but no
# additional module supported
#
# 14550 - Reserved for QGroundControl
# 14551 - ROS based control
# 14552 - Dashboard

mavlink-routerd /dev/autopilot:115200 \
--e 192.168.2.1:14550 \
--e 192.168.2.2:14550 \
--e 192.168.2.2:14551 \
--e 192.168.2.2:14552 \
--l ~/ORCA/logs




