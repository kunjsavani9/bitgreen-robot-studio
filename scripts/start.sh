#!/usr/bin/env bash
# Start BITGREEN Robot Studio: host container + rosbridge + web server.
set -euo pipefail
cd "$(dirname "$0")/.."

HOST_CONTAINER="ros_desktop"
IMAGE="ros_noetic_ready:latest"
ROSBRIDGE_PORT=9091

# 1. Load .env into the environment
if [[ -f .env ]]; then
  set -a; source .env; set +a
else
  echo "⚠  No .env found — copy .env.example to .env (signup email will be disabled)."
fi

# 2. Docker must be running
if ! docker info >/dev/null 2>&1; then
  echo "✖  Docker is not running. Start Docker Desktop and retry."; exit 1
fi

# 3. Host container: start if it exists, create if it doesn't
state=$(docker inspect -f '{{.State.Status}}' "$HOST_CONTAINER" 2>/dev/null || echo "missing")
case "$state" in
  running) echo "✔  $HOST_CONTAINER already running" ;;
  missing)
    echo "…  Creating $HOST_CONTAINER from $IMAGE"
    docker run -d --name "$HOST_CONTAINER" --platform linux/amd64 \
      --shm-size 1g -p ${ROSBRIDGE_PORT}:${ROSBRIDGE_PORT} \
      --entrypoint sleep "$IMAGE" infinity >/dev/null ;;
  *) echo "…  Starting $HOST_CONTAINER ($state)"; docker start "$HOST_CONTAINER" >/dev/null ;;
esac

# 4. roscore + rosbridge inside the host container (idempotent)
docker exec -d "$HOST_CONTAINER" bash -lc "
  source /opt/ros/noetic/setup.bash
  rosnode list >/dev/null 2>&1 || (roscore >/tmp/roscore.log 2>&1 &)
  for i in \$(seq 1 30); do rosnode list >/dev/null 2>&1 && break; sleep 1; done
  pgrep -f rosbridge_websocket >/dev/null || \
    roslaunch rosbridge_server rosbridge_websocket.launch port:=${ROSBRIDGE_PORT} >/tmp/rosbridge.log 2>&1
"
echo "✔  roscore + rosbridge (port ${ROSBRIDGE_PORT}) requested in $HOST_CONTAINER"

# 5. Python venv
if [[ ! -d venv ]]; then
  python3 -m venv venv
  ./venv/bin/pip install -q -r requirements.txt
fi
source venv/bin/activate

# 6. Replace any running server
pkill -f "python.*server.py" 2>/dev/null || true
sleep 1
exec python server.py
