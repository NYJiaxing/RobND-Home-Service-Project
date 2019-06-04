#!/bin/sh
xterm -e ' ROBOT_INITIAL_POSE="-x 3 -y -3" roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/jeff/Documents/Udacity_HSR/src/world/myworld.world' & 
sleep 5
xterm -e " TURTLEBOT_GAZEBO_MAP_FILE=/home/jeff/Documents/Udacity_HSR/src/world/map.yaml roslaunch turtlebot_gazebo amcl_demo.launch custom_amcl_launch_file:=/home/jeff/Documents/Udacity_HSR/src/scripts/asus_xtion_pro_amcl_customized.launch.xml" &
sleep 5
xterm -e " roslaunch add_markers view_navigation_customized.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers" &
sleep 5
xterm -e " rosrun pick_objects pick_objects"
