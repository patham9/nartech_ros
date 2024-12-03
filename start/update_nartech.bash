#!/bin/bash

# User selected Yes, proceed with updates
cd ~/nartech_ws/src/nartech_ros/
git pull
cd ~/AniNAL
git pull
cd ~/OpenNARS-for-Applications
git pull
cd ~/NACE
git pull
cd ~/metta-morph
git pull
cd ~/metta-morph/metta-nars
git pull
cd ~/metta-nars
git pull
cd ~/NARS-GPT
git pull

xmessage -center "Do you want to build NARTECH?" -buttons "Yes:0,No:1"
# Check if the local branch is behind the remote branch
if [ $? -eq 0 ]; then
    # User selected Yes, proceed with updates
    cd ~/nartech_ws
    colcon build
    cd ~/AniNAL
    sh ./build.sh
fi

# Common actions regardless of the choice
xmessage -center "NARTECH ready! Press 'okay', then 'Terminal', then enter './start_nartech.bash'!"
geany /home/nartech/NACE/input.metta &
