#!/usr/bin/env bash
# Emergency fix for runaway Gazebo logs under QEMU (Apple Silicon).
for c in ros_desktop $(docker ps --format '{{.Names}}' --filter "name=ros_client_"); do
  echo "── $c"
  docker exec "$c" bash -lc 'du -sh ~/.gazebo 2>/dev/null;
    find ~/.gazebo -name "*.log" -type f -exec truncate -s 0 {} + 2>/dev/null;
    du -sh ~/.gazebo 2>/dev/null'
done
