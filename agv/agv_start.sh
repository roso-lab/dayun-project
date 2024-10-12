#!/bin/bash
source /opt/ros/humble/setup.bash 
export ROS_DOMAIN_ID=42
cd /home/z/airsbot2_driver
source install/local_setup.bash
ros2 launch dog_control_launch dog_control_node_launch.py

