#!/usr/bin/env bash
set -eo pipefail

# source ROS environment (disable strict unset check for ROS scripts)
if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

# source workspace if built
if [ -f /pf_sim/install/setup.bash ]; then
  set +u
  source /pf_sim/install/setup.bash
  set -u
fi

# If first arg is "measure", run the harness inside particle_filter
if [ "${1:-}" = "measure" ]; then
  shift
  exec /pf_sim/particle_filter/run_in_container.sh "$@"
fi

# default behavior: run passed command (or bash)
exec "$@"
