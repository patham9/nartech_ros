#!/bin/bash

# Prompt user with xmessage
xmessage -center "Do you want to update NARtech? Press 'Yes' to proceed or 'No' to skip updates." -buttons "Yes:0,No:1"

# Check the user's response
if [ $? -eq 0 ]; then
    # User selected Yes, proceed with updates
    cd ~/nartech_ws/src/nartech_ros/
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
