#!/bin/bash
set -e

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source Our Workspace (if built)
if [ -f /pf_sim/install/setup.bash ]; then
  source /pf_sim/install/setup.bash
fi

# Execute the command
exec "$@"
