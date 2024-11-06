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
xmessage -center "NARtech up-to-date!"
