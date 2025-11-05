#!/bin/bash

# Source your workspace
WORKSPACE=~/medos-demo-ws
if [ -f "$WORKSPACE/devel/setup.bash" ]; then
    source "$WORKSPACE/devel/setup.bash"
else
    echo "Error: Workspace setup file not found at $WORKSPACE/devel/setup.bash"
    return
fi

# Set ROS_MASTER_URI (change if needed)
export ROS_MASTER_URI="http://localhost:11311"
# export ROS_IP="localhost"

# Confirm values
echo "Set ROS parameters: "
printenv | grep ROS
