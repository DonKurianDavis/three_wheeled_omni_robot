#!/usr/bin/bash
gnome-terminal -- bash -c "source /home/don/three_wheeled_omni_robot/install/setup.bash;  ros2 launch three_wheeled_omni_robot_description bot_launch.py simulation_mode:=false" &
sleep 5
gnome-terminal -- bash -c "source /home/don/three_wheeled_omni_robot/install/setup.bash;  rviz2 -d three_wheeled_omni_robot/src/three_wheeled_omni_robot_description/config/default.rviz" &
sleep 5
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/omni_wheel_drive_controller/cmd_vel -p stamped:=true"&
sleep 5
gnome-terminal -- bash -c "source /home/don/three_wheeled_omni_robot/install/setup.bash;  ros2 launch three_wheeled_omni_robot_navigation online_async_launch.py use_sim_time:=false" &
sleep 5
gnome-terminal -- bash -c "source /home/don/three_wheeled_omni_robot/install/setup.bash; ros2 launch three_wheeled_omni_robot_navigation navigation_launch.py use_sim_time:=false"