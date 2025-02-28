#!/bin/bash

# This updater script pull the src code for
# NACE, ONA, MeTTa-NARS, MeTTa-Morph, NARS-GPT
# as well as builds packages for ROS workspace

mv /home/nartech/Desktop/logo*.png /home/nartech/Documents/
cd ~/nartech_ws/src/nartech_ros/
git pull
cd ~/AniNAL
git pull
cd ~/OpenNARS-for-Applications
git pull
sh build.sh
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

