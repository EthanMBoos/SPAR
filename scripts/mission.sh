#!/usr/bin/env bash
# Tell the behavior layer to start or stop the mission. Stopping mid-run is a
# clean preemption: the tree halts the active leaf and cancels its Nav2 goal.
set -eo pipefail

source /ws/scripts/env.sh
CMD="${1:-}"

if [[ "$CMD" != "start" && "$CMD" != "stop" ]]; then
  echo "usage: $(basename "$0") start|stop" >&2
  exit 1
fi

ros2 topic pub --once \
  "$NS/mission/command" std_msgs/msg/String "{data: $CMD}" >/dev/null
echo "mission: $CMD"
