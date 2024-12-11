#!/bin/bash

cd /home/ros/launch
source /opt/ros/humble/setup.bash
source /home/ros/stingray__ws/install/setup.bash
ros2 launch /home/ros/launch/stingray.launch.py
