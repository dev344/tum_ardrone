#!/usr/bin/env bash
# source /opt/ros/groovy/setup.bash 
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_home
gnome-terminal -x python controller.py 
python event_listener.py
wait
