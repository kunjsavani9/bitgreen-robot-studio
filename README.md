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

## Table of contents

- [Features](#features)
- [Architecture](#architecture)
- [Repository layout](#repository-layout)
- [Requirements](#requirements)
- [First-time setup](#first-time-setup)
- [Email (Gmail) setup for signup OTP](#email-gmail-setup-for-signup-otp)
- [Starting the dashboard](#starting-the-dashboard)
- [Stopping](#stopping)
- [Health checks](#health-checks)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Development notes](#development-notes)
- [Roadmap](#roadmap)

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
- A Gmail account with 2-Step Verification, for signup emails

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
nano .env        # set ROS_ADMIN_USERNAME and ROS_ADMIN_EMAIL
                 # set ROS_SMTP_USER and ROS_SMTP_PASS (see "Email (Gmail) setup" below)
```

`.env` should end up looking like this:

```dotenv
ROS_ADMIN_USERNAME=your_admin_username
ROS_ADMIN_EMAIL=your_admin_email@gmail.com
ROS_SMTP_USER=yourname@gmail.com
ROS_SMTP_PASS=abcdefghijklmnop
```

---

## Email (Gmail) setup for signup OTP

New users verify their email with a 6-digit code sent over Gmail SMTP. Until this
is configured, **the signup page refuses new accounts**. Existing accounts can
still log in.

Gmail doesn't accept your normal password here. You need an **App Password**.

### 1. Turn on 2-Step Verification

App Passwords only exist on accounts with 2-Step Verification enabled.

1. Open https://myaccount.google.com/security
2. Under **How you sign in to Google**, click **2-Step Verification**.
3. Follow the prompts to turn it on.

### 2. Create an App Password

1. Open https://myaccount.google.com/apppasswords
2. Enter an app name, e.g. `BITGREEN Robot Studio`.
3. Click **Create**.
4. Copy the 16-character password shown, e.g. `abcd efgh ijkl mnop`.
   Google shows it only once.

> Can't find the App Passwords page? It is hidden when 2-Step Verification is
> off, and some work or school Google accounts have it disabled by the admin.
> Use a personal Gmail account instead.

### 3. Add it to `.env`

```bash
nano .env
```

```dotenv
ROS_SMTP_USER=yourname@gmail.com
ROS_SMTP_PASS=abcdefghijklmnop
```

Spaces in the password are stripped automatically, so either form works.
Never commit `.env`; it is already in `.gitignore`.

### 4. Test it before starting the server

```bash
source venv/bin/activate
set -a; source .env; set +a
python -c "import email_otp; print(email_otp.send_otp('yourname@gmail.com', '123456'))"
```

| Output | Meaning |
|---|---|
| `(True, 'sent')` | Working. Check your inbox (and spam) for the code |
| `(False, 'Email service not configured on the server.')` | `.env` wasn't loaded. Run `set -a; source .env; set +a` in the same terminal |
| `(False, 'Email login failed ...')` | Wrong address or App Password. Create a new App Password and try again |
| `(False, 'Could not send email: ...')` | Network issue. Port 587 may be blocked (common on college Wi-Fi); try a hotspot |

The server reads these variables only at startup, so restart it after changing
`.env`. Signup codes expire after 1 minute; use **Resend** on the signup page if
one runs out.

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

**5. Load environment variables (admin + Gmail SMTP).**

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

Sign up with the username and email set in `ROS_ADMIN_USERNAME` / `ROS_ADMIN_EMAIL`
to get the host role. All other accounts are clients.

**Other devices on the same network / hotspot:** replace `localhost` with the Mac's IP.

```bash
ipconfig getifaddr en0      # e.g. 172.20.10.x on an iPhone hotspot
```

**8. Wait for the client simulator.** When a client logs in, their private Gazebo
starts in the background. Under QEMU this takes **60–75 s**. Wait until the
Activity panel says the sim is ready before running ROS commands. Running
`rosnode list` too early prints `Unable to communicate with master`.

---

## Stopping

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

> Removing a `ros_client_<user>` container deletes files saved inside it. Back up
> first with `docker cp ros_client_<user>:/home/<user> ./<user>_backup`.

---

## Health checks

```bash
docker ps                                          # ros_desktop + ros_client_* running?
docker exec ros_desktop tail -20 /tmp/rosbridge.log
docker exec ros_desktop du -sh /root/.gazebo       # watch for runaway logs
docker system df                                   # Docker disk usage
./scripts/clean_gazebo_logs.sh                     # if disk is filling up
```

Check a specific client's simulator:

```bash
docker exec ros_client_<user> bash -lc 'ps aux | grep -E "rosmaster|roslaunch|gzserver" | grep -v grep'
docker exec ros_client_<user> bash -lc 'tail -20 /tmp/roscore.log; echo ----; tail -40 /tmp/gazebo.log'
```

---

## Configuration

| Variable             | Purpose                                     |
|----------------------|---------------------------------------------|
| `ROS_ADMIN_USERNAME` | Reserved admin (host) username              |
| `ROS_ADMIN_EMAIL`    | Reserved admin email (must match username)  |
| `ROS_SMTP_USER`      | Gmail address that sends OTP codes          |
| `ROS_SMTP_PASS`      | 16-character Gmail App Password             |

Tunables at the top of `container_manager.py`:

| Constant         | Default | Meaning                                  |
|------------------|---------|------------------------------------------|
| `MAX_CONTAINERS` | `3`     | Max concurrent client simulators         |
| `SPAWN_TIMEOUT`  | `75.0`  | Seconds to wait for a client sim to boot |
| `SHM_SIZE`       | `1g`    | Shared memory per client container       |

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Linux user reconcile skipped` at startup | `ros_desktop` isn't running. Run `docker start ros_desktop` |
| `ERROR: Unable to communicate with master!` in a client terminal | The sim is still booting; wait 60–75 s. If it persists, see **Client sim won't start** below |
| 3D view never connects / rosbridge errors | Run `docker exec ros_desktop tail /tmp/rosbridge.log`; check that port 9091 is published |
| Browser URL shows `0.0.0.0` | Use `localhost` or the LAN IP instead |
| Phone/laptop on hotspot can't connect | Use the host's hotspot IP (e.g. `172.20.10.x`); iPhone hotspot client isolation may block peers |
| Signup says email not configured | Set `ROS_SMTP_USER` / `ROS_SMTP_PASS` in `.env`, reload it, restart the server |
| `Email login failed` | Create a new Gmail App Password; your normal Gmail password won't work |
| OTP email never arrives | Check spam; test with the `email_otp.send_otp` command above; try a network that doesn't block port 587 |
| UI change not visible | Hard reload (Cmd/Ctrl-Shift-R) |
| Python change not visible | Restart the server |
| Robot keeps moving after stopping a script | rospy ignores SIGTERM; stop uses `pkill -9` |
| Disk suddenly full / `No space left on device` | `./scripts/clean_gazebo_logs.sh`, then `docker system prune -f` |
| Max containers reached | `./scripts/stop.sh` or raise `MAX_CONTAINERS` |

### Client sim won't start

```bash
docker restart ros_client_<user>      # kill stuck processes
pkill -f server.py                    # clear cached "starting" state
set -a; source .env; set +a
python server.py
```

Hard-reload the client's page and wait 60–75 s. If it still fails, back up the
user's files and recreate the container:

```bash
docker cp ros_client_<user>:/home/<user> ./<user>_backup
docker rm -f ros_client_<user>
```

More in [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md).

---

## Development notes

- **HTML changes** need a hard reload (Cmd/Ctrl-Shift-R).
- **Python changes** need a server restart: `pkill -f server.py && python server.py`, with `.env` loaded.
- **Stopping rospy scripts** requires `pkill -9`, because rospy ignores SIGTERM.
- **Never commit** `.env` or `ros_ops.db`; both are git-ignored.

## Roadmap

- MoveIt / `move_group` support for the UR5 sample script
- Pinned front-end dependencies served locally (offline mode)

## Acknowledgements

Built at BITGREEN Technolabz. Inspired by The Construct's ROS Development Studio.
