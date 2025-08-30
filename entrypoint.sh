#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.sh
source /ofb_ws/install/setup.bash

source /opt/venv/bin/activate

# Ensure Python from venv is used
export PATH="/opt/venv/bin:$PATH"
export PYTHONPATH="/opt/venv/lib/python3.12/site-packages:$PYTHONPATH"

ros2 launch image_processing_node image_processing_node.launch.py
