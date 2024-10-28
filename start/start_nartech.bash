export QT_QPA_PLATFORM=xcb
ros2 launch nav2_bringup tb4_simulation_launch.py slam:=True nav:=True headless:=False autostart:=True rviz_config_file:=nartech_view.rviz
