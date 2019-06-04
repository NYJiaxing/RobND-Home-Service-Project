#!/bin/sh
xterm -e ' ROBOT_INITIAL_POSE="-x 3 -y -3" roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/jeff/Documents/Udacity_HSR/src/world/myworld.world' & 
sleep 5
xterm -e " roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=/home/jeff/Documents/Udacity_HSR/src/shell_scripts/asus_xtion_pro_gmapping_customized.launch.xml" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " roslaunch turtlebot_teleop keyboard_teleop.launch"
