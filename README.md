<p align="center">
  <img src="static/assets/bitgreenlogo.png" alt="BITGREEN" width="180">
</p>

<h1 align="center">BITGREEN Robot Studio</h1>

<p align="center">
  Browser-based, multi-user ROS simulation platform. Every student gets an isolated
  robot environment with a 3D viewer, terminal, code editor and file explorer, with
  nothing to install locally.
</p>

---

## Features

- **Isolated per-student sims.** Each client gets their own Docker container with its own roscore and headless Gazebo.
- **Host dashboard.** The admin can see connected clients, mirror their terminals, browse their files and switch the robot model.
- **Robots.** TurtleBot3 Burger (empty world, TB3 world, generated arena with `move_base` navigation) and UR5 arm (IK, joint jog, trajectory playback).
- **In-browser tooling.** Three.js 3D viewport, xterm.js shells (PTY over WebSocket), CodeMirror editor, 1 GB per-user file space.
- **Accounts.** SQLite + bcrypt, session tokens, and signup verified by email OTP.
- **UI.** Styled after ABB RobotStudio, with BITGREEN blue `#263689`.

## Architecture

```
 Browser (login / index / host)
        │  HTTP + WebSocket /ws/{id}
        ▼
 ┌──────────────────────────┐        docker exec
 │ server.py (FastAPI :8000)│ ─────────────────────────────┐
 │  auth · PTY hub · files  │                              │
 └──────────────────────────┘                              ▼
        │                                  ┌───────────────────────────────┐
        │ docker exec                      │ ros_client_<user>  (×N, max 3)│
        ▼                                  │ roscore · Gazebo · robot      │
 ┌─────────────────────────────┐           └───────────────────────────────┘
 │ ros_desktop (host container)│
 │ roscore · rosbridge :9091   │
 │ Linux accounts per user     │
 └─────────────────────────────┘
```

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for details.

## Repository layout

```
├── server.py              FastAPI app: routes, WebSocket hub, PTYs, file API
├── auth.py                SQLite users, bcrypt, sessions, OTP flow
├── email_otp.py           Gmail SMTP sender for signup codes
├── container_manager.py   Per-client container lifecycle, sim launch, teleop, nav
├── gazebo_manager.py      Gazebo process + log management
├── linux_user.py          Per-user Linux accounts & home-jailed file ops
├── models.py              Robot model catalog (TB3, UR5), worlds, nav map
├── static/                login, signup, client (index) and host pages
├── scripts/               start / stop / Gazebo log cleanup
├── docker/Dockerfile      Reference build for ros_noetic_ready image
└── docs/                  Architecture & troubleshooting
```

## Requirements

- macOS (Apple Silicon tested) or Linux x86_64
- Docker Desktop / Docker Engine. On Apple Silicon, amd64 images run under QEMU.
- Python 3.9+
- The `ros_noetic_ready:latest` image (ROS Noetic, Gazebo, TurtleBot3, UR5, rosbridge)

---

## First-time setup

```bash
# 1. Clone
git clone https://github.com/kunjsavani9/bitgreen-robot-studio.git
cd bitgreen-robot-studio

# 2. Build the ROS image (one time, slow under QEMU)
docker build --platform linux/amd64 -t ros_noetic_ready:latest docker/

# 3. Python environment
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# 4. Configuration
cp .env.example .env
nano .env        # set ROS_ADMIN_USERNAME, ROS_ADMIN_EMAIL, ROS_SMTP_USER, ROS_SMTP_PASS
```

---

## Starting the dashboard

### Option A — one command

```bash
./scripts/start.sh
```

This loads `.env`, starts `ros_desktop`, brings up roscore and rosbridge, activates the venv and launches the server.

### Option B — step by step

**1. Start Docker Desktop.** Wait until it reports running, then verify:

```bash
docker info > /dev/null && echo "Docker OK"
docker images | grep ros_noetic_ready
```

**2. Start the shared host container.**

```bash
# Start the container if it already exists
docker start ros_desktop

# First run only: create the container instead
docker run -d --name ros_desktop --platform linux/amd64 \
  --shm-size 1g -p 9091:9091 \
  --entrypoint sleep ros_noetic_ready:latest infinity
```

**3. Launch roscore + rosbridge (port 9091) inside it.**

```bash
docker exec -d ros_desktop bash -lc \
  "source /opt/ros/noetic/setup.bash && \
   (rosnode list >/dev/null 2>&1 || (roscore >/tmp/roscore.log 2>&1 &)) && sleep 5 && \
   roslaunch rosbridge_server rosbridge_websocket.launch port:=9091 >/tmp/rosbridge.log 2>&1"
```

**4. Go to the project and activate the venv.**

```bash
cd ~/ros_workspace/robot_web_ui     # or wherever you cloned the repo
source venv/bin/activate
```

**5. Load environment variables (admin + SMTP).**

```bash
set -a; source .env; set +a
```

**6. Stop any old instance and start the server.**

```bash
pkill -f server.py
python server.py
```

Expected output:

```
🚀 ROS Ops Center starting on http://0.0.0.0:8000
   Login:      http://localhost:8000/login
   ...
   Linux users OK (N registered)
```

> If you see `Linux user reconcile skipped`, `ros_desktop` isn't running. Go back to step 2.

**7. Open in the browser.**

| Page        | URL                           |
|-------------|-------------------------------|
| Login       | http://localhost:8000/login   |
| Sign up     | http://localhost:8000/signup  |
| Client view | http://localhost:8000         |
| Host panel  | http://localhost:8000/host    |

Sign in with the account matching `ROS_ADMIN_USERNAME` / `ROS_ADMIN_EMAIL` to get the host role.

**Other devices on the same network / hotspot:** replace `localhost` with the Mac's IP.

```bash
ipconfig getifaddr en0      # e.g. 172.20.10.x on an iPhone hotspot
```

### Stopping

```bash
./scripts/stop.sh           # stop server + remove client containers
./scripts/stop.sh --all     # also stop ros_desktop
```

Manual equivalent:

```bash
pkill -f server.py
docker rm -f $(docker ps -aq --filter "name=ros_client_")
docker stop ros_desktop
```

### Health checks

```bash
docker ps                                          # ros_desktop + ros_client_* running?
docker exec ros_desktop tail -20 /tmp/rosbridge.log
docker exec ros_desktop du -sh /root/.gazebo       # watch for runaway logs
./scripts/clean_gazebo_logs.sh                     # if disk is filling up
```

---

## Configuration

| Variable             | Purpose                                     |
|----------------------|---------------------------------------------|
| `ROS_ADMIN_USERNAME` | Reserved admin username                     |
| `ROS_ADMIN_EMAIL`    | Reserved admin email (must match username)  |
| `ROS_SMTP_USER`      | Gmail address that sends OTP codes          |
| `ROS_SMTP_PASS`      | 16-character Gmail App Password             |

Tunables such as `MAX_CONTAINERS`, `SPAWN_TIMEOUT` and `SHM_SIZE` live at the top of `container_manager.py`.

## Development notes

- **HTML changes** need a hard reload (Cmd/Ctrl-Shift-R).
- **Python changes** need a server restart: `pkill -f server.py && python server.py`.
- **Stopping rospy scripts** requires `pkill -9`, because rospy ignores SIGTERM.
- **Browser URL showing `0.0.0.0`:** use `localhost` or the LAN IP instead.

More in [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md).

## Roadmap

- MoveIt / `move_group` support for the UR5 sample script
- Pinned front-end dependencies served locally (offline mode)

## Acknowledgements

Built at BITGREEN Technolabz. Inspired by The Construct's ROS Development Studio.
