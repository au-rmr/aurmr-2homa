#!/usr/bin/env bash
# A shell on the control host's private ROS domain, inside the running robot container: the UR
# driver's services (dashboard, set_payload, speed slider), ros2 control, the raw controller
# topics. Only works on the control host: the private domain is loopback only.
# Usage, on the control host: AURMR_HOST=<control host> scripts/private_shell.sh [command...]
set -eu
cd "$(git rev-parse --show-toplevel)"
DOMAIN=${AURMR_PRIVATE_DOMAIN:-142}   # robot.launch.py private_domain
URI=file:///repo/robot/tahoma_control/config/cyclonedds_private.xml
CMD=${*:-bash}
exec docker compose -f compose/control.yaml exec -e ROS_DOMAIN_ID="$DOMAIN" \
  -e AURMR_PRIVATE_DOMAIN="$DOMAIN" -e CYCLONEDDS_URI="$URI" robot \
  bash -c "source /ws/install/setup.bash && exec $CMD"
