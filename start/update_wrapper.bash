# Called at startup to pull updater script
# and invoke it to update the workspace 
# i.e. pull src and build ROS workspace
cd /home/nartech/nartech_ws/src/nartech_ros/start
rm update_nartech.bash
wget https://raw.githubusercontent.com/patham9/nartech_ros/refs/heads/master/start/update_nartech.bash
chmod 775 update_nartech.bash
./update_nartech.bash

