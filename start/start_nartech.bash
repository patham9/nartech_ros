if [ "$1" == "fast" ]; then
    export MY_WORLD=/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/warehouse.sdf
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_SOFTWARE=1
    gnome-terminal -- bash -c "sleep 10 && export LIBGL_ALWAYS_SOFTWARE=0 && export QT_QPA_PLATFORM=xcb && gz sim --render-engine ogre \$MY_WORLD; exec bash" &
    ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True nav:=True headless:=True autostart:=True use_sim_time:=True rviz_config_file:=nartech_view.rviz world:=$MY_WORLD
else
    export MY_WORLD=/opt/ros/jazzy/share/nav2_minimal_tb4_sim/worlds/depot.sdf
    export QT_QPA_PLATFORM=xcb
    export LIBGL_ALWAYS_SOFTWARE=1
    ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True nav:=True headless:=False autostart:=True use_sim_time:=True rviz_config_file:=nartech_view.rviz world:=$MY_WORLD
fi
