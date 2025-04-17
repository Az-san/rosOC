#!/bin/sh

source ~/rosOC/devel/setup.bash

export TURTLEBOT3_MODEL=waffle

roslaunch robot_pkg robot_and_interfaceV_OC.launch exp_type:="ee"


