# nartech_ros

<img width="1071" alt="Image" src="https://github.com/user-attachments/assets/6fcfeaab-55b6-4637-900d-85dc41922a97" />

### Run all necessary ROS nodes, Gazebo simulation, and AI demo file from demos folder:

```./start_nartech.bash metta demo_with_nars.metta```

### For developers, run all necessary ROS nodes, Gazebo simulation, and MeTTa interface first:

```./start_nartech.bash metta```

then open Geany or another Terminal to run the particular AI demo:

```cd /home/nartech/nartech_ws/src/nartech_ros/```

```python3 main.py ./demos/demo_with_nars.metta```

Benefit of this is that this Python3 script can be safely stopped (```sudo killall -9 python3```) and restarted without affecting crucial ROS components and the Gazebo simulation,
this allows continuous work on AI demos in MeTTa.

### Run monolithic NACE+NARS+GPT input demo:

```./start_nartech.bash```

### Test various MeTTabridge plugins without running ROS:

```python3 mettabridge.py ./plugins/Xplugintests/file.metta```

These plugins can also be used standalone as MeTTa extensions via the MeTTa import command.
