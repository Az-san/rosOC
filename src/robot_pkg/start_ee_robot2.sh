#!/bin/sh

source ~/rosOC/devel/setup.bash

export TURTLEBOT3_MODEL=waffle

roslaunch robot_pkg robot_and_interfaceV2.launch exp_type:="ee"
