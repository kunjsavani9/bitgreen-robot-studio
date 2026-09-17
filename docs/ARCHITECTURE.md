# Architecture

## Goals

The platform gives each student an isolated, browser-only ROS environment and gives
the instructor (host) one place to watch and manage every session.

## Components

### Web server — `server.py`
- **Framework.** FastAPI on port 8000. It serves the pages in `static/` and exposes a JSON API under `/api/*`.
- **WebSocket hub.** `/ws/{client_id}` authenticates with `?token=`. Each connection can hold several PTY terminals, and the hub broadcasts client presence and activity to the host.
- **Host terminals** attach to the shared `ros_desktop` container.
- **Client terminals** run `docker exec` into that user's container.
- **Background loop.** A periodic task polls each client's `/odom` (around 5 Hz) and `/scan` (around 2.5 Hz), pushes the results to that client's 3D view, and garbage-collects idle containers.

### Auth — `auth.py`, `email_otp.py`
- **Storage.** Users live in SQLite (`ros_ops.db`, created on first import, git-ignored). Passwords are hashed with bcrypt, and session tokens are generated with `secrets.token_urlsafe`.
- **Signup** stores a pending account and emails a 6-digit OTP. The account activates once the code is verified.
- **Admin role.** One reserved username/email pair from the environment gets it.

### Containers — `container_manager.py`
- **One container per client.** Each is named `ros_client_<username>`, built from `ros_noetic_ready:latest`, and created with `--platform linux/amd64 --entrypoint sleep ... infinity`. There is a hard cap of `MAX_CONTAINERS`.
- **Lifecycle.** A container that is `created` or `exited` gets `docker start`. A missing one gets `docker create` followed by start.
- **Simulation.** A supervisor loop runs `roslaunch` for the selected model and restarts it on crash. A model switch tears down Gazebo but keeps roscore.
- **Robot API.** A persistent `cmd_vel` publisher and odom reader. `stopDrive` sends zero velocity; `hardStop` also kills user scripts and cancels go-to goals.
- **Navigation.** The arena is generated into a map, then `map_server` and `move_base` run with TurtleBot3 parameters.
- **UR5.** Driven through a `JointTrajectory` publisher and `JointState` reader, with joint limits of ±360° and velocity-limited playback.

### Models — `models.py`
The `ModelSpec` catalog records the launch args, topics, env exports and world geometry for each model. The same geometry drives the Gazebo `.world` and the obstacles drawn in the browser.

### Linux users — `linux_user.py`
On startup the server reconciles every registered user into a Linux account inside `ros_desktop`. File operations are jailed to `/home/<username>`.

## Key decisions

| Decision | Why | Trade-off |
|---|---|---|
| Full container per client | Complete ROS/Gazebo isolation, no port juggling | Heavy under QEMU, so capped at 3 |
| `docker exec` instead of ROS networking to clients | Each container keeps default master 11311 privately | Every call is a subprocess |
| Vanilla JS + CDN libs | No build step, easy to hack live | No bundling or type checking |
| SQLite | Zero-ops for a single host | Single-node only |

## Known constraints (Apple Silicon)

- **Gazebo log flooding.** QEMU timing warnings make Gazebo write logs very fast. Logs for both the configured port and `server-11345` are symlinked to `/dev/null`, and a watcher re-pins them every 60 s.
- **Cold starts are slow.** `SPAWN_TIMEOUT` is 75 s to allow for this.
