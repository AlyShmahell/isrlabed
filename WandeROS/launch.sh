#!/bin/bash
cd catkin_ws
rm -rf devel build
catkin_make
source devel/setup.bash
rosrun turtlebotcuriosity turtlebotcuriosity.py
