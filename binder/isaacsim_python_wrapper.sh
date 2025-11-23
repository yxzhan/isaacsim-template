#!/bin/bash

# Isaac-sim python exe wrapper

# unset virtualGL ENV (which breaks ROS2 bridge)
unset LD_PRELOAD
# Clear default ROS ENV
unset PYTHONPATH

# unset CONDA_PREFIX
export PYTHONPATH=/mnt/dev-tools/conda_py311/lib/python3.11/site-packages

# Isaac Sim ROS2 Bridge
export LD_LIBRARY_PATH=$ISAACSIM_PATH/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib

# # setup python 3.11 ROS jazzy environment
# source /mnt/dev-tools/ros_ws/jazzy_py311/install/setup.bash

# # setup Isaac Sim examples ROS workspace
# source /mnt/dev-tools/ros_ws/isaacsim_ws/install/setup.bash

$ISAACSIM_PYTHON_EXE "$@"