#!/bin/bash

cd ~/nartech_ws/src/nartech_ros/
# Check if the local branch is behind the remote branch
if git pull | grep -q "Already up to date."; then
    # User selected Yes, proceed with updates
    git pull
    cd ~/nartech_ws
    colcon build
    cd ~/AniNAL
    git pull
    sh ./build.sh
    cd ~/OpenNARS-for-Applications
    git pull
    #./build.sh # Uncomment if it should build every startup
    cd ~/NACE
    git pull
    cd ~/metta-morph
    git pull
    cd ~/metta-morph/metta-nars
    git pull
    # metta-nars auto-compiles at first run when code changes
    cd ~/metta-nars
    git pull
    cd ~/NARS-GPT
    git pull
    #./build.sh # Uncomment if it should build every startup
fi

# Common actions regardless of the choice
xmessage -center "NARtech ready! Press OK then execute ./start_nartech.bash in Terminal of the opening window!"
geany /home/nartech/NACE/input.metta &
