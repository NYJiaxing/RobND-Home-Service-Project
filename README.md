# RoboND-HomeServiceRobot-Project
A home service robot that localize itself, map its surrounding environment and move picked objects around

### Process
This project using open source ROS packages from Turtlebot 3 and AMCL. 
1. Building my own gazebo world
2. Building a shell file called test_slam.sh to use the depth-sensor mapping the gazebo world. 
   - I do want to mention here which confuse me a lot that how to save the map into png file and yaml file.
   - I got help from this link: http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM that after maped the world, open a new termianl in the folder and type: rosrun map_server map_saver -f /tmp/my_map
3. Building a shell file called test_navigation.sh to test the navigation function
4. Build two ROS packages, add_markers and picks_up to mimic the robot pick up the object from one position and drop to the other place. The object will show up in each place before the robot reach the position

### Result

Picking up ![](/image/Pickingup.png)

Dropping off ![](/image/droppingoff.png)
