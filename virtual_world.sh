#!/usr/bin/bash
gnome-terminal -- bash -c "source /home/don/three_wheeled_omni_robot/install/setup.bash; ros2 launch three_wheeled_omni_robot_ignition_gazebo gazebo_launch.py " &
sleep 5
gnome-terminal -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/omni_wheel_drive_controller/cmd_vel -p stamped:=true"&
sleep 5
gnome-terminal -- bash -c "source /home/don/three_wheeled_omni_robot/install/setup.bash;  ros2 launch three_wheeled_omni_robot_navigation online_async_launch.py " &
sleep 10
gnome-terminal -- bash -c "source /home/don/three_wheeled_omni_robot/install/setup.bash; ros2 launch three_wheeled_omni_robot_navigation navigation_launch.py "