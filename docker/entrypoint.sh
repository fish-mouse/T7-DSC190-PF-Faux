#!/usr/bin/env bash
set -eo pipefail

# Set variable to prevent unbound error when strict mode enabled
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

# source ROS environment
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# If the workspace was built in /pf_sim/install, source its setup to expose packages
if [ -f /pf_sim/install/setup.bash ]; then
  source /pf_sim/install/setup.bash
fi

# If first arg is "measure", run the harness inside particle_filter
if [ "${1:-}" = "measure" ]; then
  shift
  exec /pf_sim/src/particle_filter/particle_filter/run_in_container.sh "$@"
fi

# default behavior: run passed command (or bash)
exec "$@"