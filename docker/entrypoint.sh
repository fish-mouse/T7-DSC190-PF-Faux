#!/usr/bin/env bash
set -euo pipefail

# source ROS environment
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# If first arg is "measure", run the harness inside particle_filter
if [ "${1:-}" = "measure" ]; then
  shift
  exec /pf_sim/particle_filter/run_in_container.sh "$@"
fi

# default behavior: run passed command (or bash)
exec "$@"