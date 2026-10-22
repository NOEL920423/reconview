#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 緩衝設定
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-DEFAULT?sockets_size=33554432}"
echo "ros2 setup complete"
echo "Fast DDS transports: $FASTDDS_BUILTIN_TRANSPORTS"


# source ~/.bashrc
