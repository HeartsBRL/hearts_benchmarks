#!/bin/bash

echo "getting current date"
DATE=$(date "+%y%m%d%H%M")

echo "rosbag recording" 
echo "printing date:" $DATE

rosbag record -O subset $DATE _HEARTS.bag /rockin/robot_pose /rocking/marker_pose /rockin/trajectory /rockin/device
# rosbag record -O subset HEARTS.bag /rockin/robot_pose /rocking/marker_pose /rockin/trajectory /rockin/device /rockin/<device>/image /rockin/<device>/camera_info /rockin/depth_<id>/pointcloud /rockin/scan_<id> tf 

