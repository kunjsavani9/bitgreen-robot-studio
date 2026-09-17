#!/usr/bin/env bash
# Stop the web server and all per-client containers. Host container is kept
# unless --all is passed.
set -uo pipefail
pkill -f "python.*server.py" && echo "✔  server stopped" || echo "–  server not running"
ids=$(docker ps -aq --filter "name=ros_client_")
[[ -n "$ids" ]] && docker rm -f $ids >/dev/null && echo "✔  client containers removed"
if [[ "${1:-}" == "--all" ]]; then
  docker stop ros_desktop >/dev/null && echo "✔  ros_desktop stopped"
fi
