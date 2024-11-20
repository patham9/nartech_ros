# This updater script pull the src code for
# NACE, ONA, MeTTa-NARS, MeTTa-Morph, NARS-GPT
# as well as builds packages for ROS workspace

cd ~/nartech_ws/src/nartech_ros/
git pull

cd ~/nartech_ws
colcon build

cd ~/AniNAL
git pull
sh ./build.sh

cd ~/OpenNARS-for-Applications
git pull
#./build.sh #uncomment if it should build it every startup (not needed by NARtech demos currently)

cd ~/NACE
git pull

cd ~/metta-morph
git pull

cd ~/metta-morph/metta-nars
git pull

#metta-nars in metta-morph automatically re-compiles at first run when code changes
cd ~/metta-nars
git pull

cd ~/NARS-GPT
git pull
#./build.sh #uncomment if it should build it every startup (not needed by NARtech demos currently)

zenity --info --text "  NARTECH workspace is up-to-date!  " --title="Update Status" --icon="/home/nartech/NARTECH/graphics/logo.png"

