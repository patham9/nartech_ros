if [ "$1" == "metta" ]; then
    #then execute "python3 main.py metta" to execute space.metta
    export MY_WORLD=/home/nartech/nartech_ws/src/nartech_ros/worlds/narhouse.sdf
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_SOFTWARE=1
    gnome-terminal -- bash -c "sleep 10 && export LIBGL_ALWAYS_SOFTWARE=0 && export QT_QPA_PLATFORM=xcb && gz sim --render-engine ogre \$MY_WORLD; exec bash" &
    geany /home/nartech/nartech_ws/src/nartech_ros/channels/space.metta &
    if [ "$2" == "space.metta" ]; then
        gnome-terminal -- bash -c "sleep 20 && cd /home/nartech/nartech_ws/src/nartech_ros/channels/ && python3 main.py metta; exec bash" &
    fi
    ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True nav:=True headless:=True autostart:=True use_sim_time:=True rviz_config_file:=nartech_view.rviz world:=$MY_WORLD
    exit 0
fi
if [ "$1" != "nogrid" ] && [ "$2" != "nogrid" ] && [ "$3" != "nogrid" ]; then
    cd /home/nartech/nartech_ws/src/nartech_ros/channels/
    python3 main.py &
    cd /home/nartech/
fi
if [ "$1" != "slow" ] && [ "$2" != "slow" ] && [ "$3" != "slow" ]; then
    export MY_WORLD=/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/warehouse.sdf
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_SOFTWARE=1
    gnome-terminal -- bash -c "sleep 10 && export LIBGL_ALWAYS_SOFTWARE=0 && export QT_QPA_PLATFORM=xcb && gz sim --render-engine ogre \$MY_WORLD; exec bash" &
    if [ "$1" != "nonace" ] && [ "$2" != "nonace" ] && [ "$3" != "nonace" ]; then
        gnome-terminal -- bash -c "sleep 20 && cd /home/nartech/NACE/ && python3 main.py world=-2 nogui; exec bash" &
        python3 /home/nartech/nartech_ws/src/nartech_ros/optional/gui.py &
    fi
    geany /home/nartech/NACE/input.metta &
    ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True nav:=True headless:=True autostart:=True use_sim_time:=True rviz_config_file:=nartech_view.rviz world:=$MY_WORLD
else
    export MY_WORLD=/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/depot.sdf
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_SOFTWARE=1
    if [ "$1" != "nonace" ] && [ "$2" != "nonace" ] && [ "$3" != "nonace" ]; then
        gnome-terminal -- bash -c "sleep 20 && cd /home/nartech/NACE/ && python3 main.py world=-2 nogui; exec bash" &
        python3 /home/nartech/nartech_ws/src/nartech_ros/optional/gui.py &
    fi
    geany /home/nartech/NACE/input.metta &
    ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True nav:=True headless:=False autostart:=True use_sim_time:=True rviz_config_file:=nartech_view.rviz world:=$MY_WORLD
fi
