export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True nav:=True headless:=False autostart:=True use_sim_time:=True rviz_config_file:=nartech_view.rviz
