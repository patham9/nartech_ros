#Called at startup to keep nartech up-to-date!
#1. we pull to ensure nartech_ws to ensure we have the newest update script
cd ~/nartech_ws/src/nartech_ros/
git pull
#2. we execute the update script
cd ~/nartech_ws/src/nartech_ros/start/
./update_nartech.bash
