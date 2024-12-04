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

# Check if the local branch is behind the remote branch
if zenity --question --text="Do you want to build NARTECH?" --title="Build NARTECH" --icon="/home/nartech/Documents/logo.png"; then
    # User selected Yes, proceed with updates
    cd ~/nartech_ws
    colcon build
    cd ~/AniNAL
    sh ./build.sh
fi

# Common actions regardless of the choice
zenity --info --text "  NARTECH ready! Press 'OK', then 'Terminal', then enter './start_nartech.bash'!  " --title="Update Status" --icon="/home/nartech/Documents/logo.png"

geany /home/nartech/NACE/input.metta &
