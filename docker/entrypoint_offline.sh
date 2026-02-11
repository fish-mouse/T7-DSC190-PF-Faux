#!/bin/bash
set -eo pipefail

# No ROS2 setup needed for offline testing
export PYTHONPATH=/pf_test:$PYTHONPATH

# If first arg is "test", run container_runner
if [ "${1:-}" = "test" ]; then
  shift
  exec python3 -m particle_filter.container_runner "$@"
fi

# Default: run passed command
exec "$@"