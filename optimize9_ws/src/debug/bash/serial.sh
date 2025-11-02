#!/bin/bash


source /opt/ros/noetic/setup.bash 
sleep 0.1
source /home/rc/RC_2026/optimize_ws8/devel/setup.bash
sleep 0.1

rosrun serial_node serial_node
