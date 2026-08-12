#!/usr/bin/env bash
# The infra container: one zenoh router, nothing else. The robot and sim
# containers join this container's network namespace (compose
# `network_mode: "service:core"`), so all of them behave like processes on
# one machine and everything is localhost.
# No -u: colcon's generated setup.bash reads variables it never sets
# (COLCON_TRACE), which nounset turns into a fatal error.
set -eo pipefail

exec ros2 run rmw_zenoh_cpp rmw_zenohd >/tmp/zenoh_router.log 2>&1
