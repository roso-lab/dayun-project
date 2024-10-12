#!/bin/bash
source /opt/ros/humble/setup.bash 
sleep 20
export ROS_DOMAIN_ID=42
cd /home/z/airsbot2_driver/src/odom_navi

python3.10 move_to_point_v2.py
