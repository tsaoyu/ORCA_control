#!/bin/bash

# Set up the repository clones on a blank raspberry pi.
# Prepare the directories expected by push2pi and pullfrompi.
# This requires an internet connection to clone from Github.

ssh jetson@${BLUEROV2_IP:=192.168.2.2} <<'ENDSSH'
git clone git@github.com:tsaoyu/ORCA_control.git ORCA_control
git clone --bare ORCA_control orca-bluerov-bare
cd ORCA_control
git remote add bareclone ~/orca-bluerov-bare
ENDSSH