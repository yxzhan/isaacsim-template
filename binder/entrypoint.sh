#!/bin/bash

# Setup ROS2 environment
source ${ROS_PATH}/setup.bash

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"