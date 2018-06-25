#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$DIR/../World/MyWorld" &
sleep 5

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$DIR/../World/myMap.yaml" &
sleep 5

roslaunch turtlebot_rviz_launchers view_navigation.launch
