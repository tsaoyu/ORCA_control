#!/usr/bin/env bash

# Use mavproxy.py to create MAVLink connections
# consider use mavrouter.sh if fake GPSInput not used
#
# 14550 - Reserved for QGroundControl
# 14551 - ROS based control
# 14552 - Dashboard


mavproxy.py --master=/dev/ttyPX4 \
--baudrate=115200 \
--load-module='GPSInput' \
--source-system=200 \
--cmd="set heartbeat 0" \
--out udpin:localhost:9000 \
--out udp:192.168.2.1:14550 \
--out udp:192.168.2.2:14550 \
--out udp:192.168.2.2:14551 \
--logfile ~/ORCA/logs
