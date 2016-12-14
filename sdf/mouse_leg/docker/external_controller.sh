#!/bin/bash

#source ROS setup
source /catkin_ws/devel/setup.bash 

#set environment variables in script, because they can't be set permanently when using docker exec
export ROS_IP="$(hostname -I | cut -d " " -f 1)" 
export ROS_MASTER_URI=http://192.168.178.28:11311/

##sample script, which takes values from the /mouse_controller topic, multiplies by 10 and outputs them to the /result topic
rosrun beginner_tutorials transformData.py
