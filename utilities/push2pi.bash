#!/bin/bash
set -e

echo "Pushing to the robot at ${BLUEROV2_IP:=192.168.2.2}..."
git push jetson@$BLUEROV2_IP:orca-bluerov-bare


echo "Pulling from bareclone on the robot..."
ssh jetson@$BLUEROV2_IP 'cd ~/ORCA_control; git pull bareclone master'
