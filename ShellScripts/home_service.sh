#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
#export ROBOT_INITIAL_POSE="-x -4.5 -y -0.75"

xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$DIR/../World/MyWorld" &
sleep 5

xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$DIR/../World/myMap.yaml" &

xterm -e "rosrun rviz rviz -d $DIR/../RvizConfig/view_navigation_markers.rviz" &
sleep 5

xterm -e "rosrun pick_objects pick_objects" &
rosrun add_markers add_markers
