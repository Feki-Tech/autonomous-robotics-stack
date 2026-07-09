#!/bin/sh
# Sources ROS 2 and, when present, the workspace overlay before running the
# container command.
set -e

. "/opt/ros/${ROS_DISTRO}/setup.sh"
# The overlay may have been built outside the container (different prefix);
# probe it in a subshell and skip it gracefully instead of failing the
# whole command.
if [ -f /ws/install/setup.sh ]; then
  if (. /ws/install/setup.sh) > /dev/null 2>&1; then
    . /ws/install/setup.sh
  else
    echo "warning: /ws/install exists but could not be sourced (built outside this container?)" >&2
  fi
fi

exec "$@"
