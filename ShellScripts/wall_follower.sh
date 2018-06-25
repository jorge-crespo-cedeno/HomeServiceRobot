#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
#export ROBOT_INITIAL_POSE="-x -4.5 -y -0.75"

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$DIR/../World/MyWorld" &
sleep 5

xterm -e "roslaunch turtlebot_gazebo gmapping_demo.launch custom_gmapping_launch_file:=$DIR/../launch/gmapping.launch.xml" &
sleep 5

xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5

rosrun wall_follower wall_follower
