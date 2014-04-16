#!/usr/bin/env bash
source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/ros_home/tum_simulator
roslaunch cvg_sim_test garching_kitchen.launch

