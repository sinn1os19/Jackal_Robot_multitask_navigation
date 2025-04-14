#!/bin/bash

echo "阶段1：导航至桥入口"
roslaunch jackal_navigation navigation.launch

echo "完成阶段1，开始桥梁导航..."
sleep 2
roslaunch jackal_navnavigation bridge_navigation.launch