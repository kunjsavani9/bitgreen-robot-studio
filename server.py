"""
ROS Ops Center — Multi-Client Server
Each client gets a real PTY shell + shared robot state + file manager
"""
import asyncio, os, pty, fcntl, termios, struct, json, base64
import threading, time, signal, pwd, math, subprocess, shlex
from pathlib import Path
from typing import Dict, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, FileResponse, JSONResponse, Response
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import auth  # Stage 1: SQLite user store + bcrypt + session tokens
import linux_user  # Stage 2: per-user Linux accounts in the container
from container_manager import container_manager  # Stage 4 (Option A): per-client container
import models  # Track A1: Model Spec catalog

app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])

# ── STATIC FILES ──────────────────────────────────────────────────────────────
STATIC = Path(__file__).parent / "static"
STATIC.mkdir(exist_ok=True)
app.mount("/static", StaticFiles(directory=str(STATIC)), name="static")

# ── CLIENT REGISTRY ───────────────────────────────────────────────────────────
class Terminal:
    """One PTY (shell) within a connection. A client/host can open several."""
    def __init__(self, tid: str):
        self.tid = tid
        self.fd  = None
        self.pid = None
        self.reader = None

class Client:
    def __init__(self, cid: str, ws: WebSocket):
        self.id      = cid
        self.ws      = ws
        self.role    = "client"   # or "host"
        self.name    = cid
        self.username = cid       # authenticated username (Stage 1)
        self.color   = "#00e5ff"
        # Stage 3: multiple terminals per connection, keyed by terminal_id.
        self.terminals = {}       # tid -> Terminal
        # Back-compat single-PTY accessors (first terminal)
        self.pty_fd  = None
        self.pty_pid = None
        self.cwd     = str(Path.home())
        self.alive   = True
        self.activity= []         # last 20 activity items
        # Host-mirror scrollback: {tid: [b64 chunks]} so a host that connects
        # or refreshes later can replay the client's recent terminal output.
        self.mirror  = {}

class RobotEnv:
    """One isolated robot environment (state + timing) for a single user.
    Each user drives their own robot; states never cross between users."""
    def __init__(self):
        self.state = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "wz": 0.0}
        self.state_time = time.monotonic()
        self.teleop_active_until = 0.0
        self.cmd_vel_relay_time = 0.0
        self.last_odom_time = 0.0
        self.odom_origin = None
        self._was_teleoping = False

    @staticmethod
    def _yaw(yaw): return math.atan2(math.sin(yaw), math.cos(yaw))

    def reanchor_odom(self, st):
        try:
            raw_x = float(st.get("x", 0.0) or 0.0)
            raw_y = float(st.get("y", 0.0) or 0.0)
            raw_yaw = float(st.get("yaw", 0.0) or 0.0)
        except (TypeError, ValueError):
            return
        held_x = float(self.state.get("x", 0.0) or 0.0)
        held_y = float(self.state.get("y", 0.0) or 0.0)
        held_yaw = float(self.state.get("yaw", 0.0) or 0.0)
        origin_yaw = raw_yaw - held_yaw
        cos_o = math.cos(origin_yaw); sin_o = math.sin(origin_yaw)
        origin_x = raw_x - (held_x * cos_o - held_y * sin_o)
        origin_y = raw_y - (held_x * sin_o + held_y * cos_o)
        self.odom_origin = (origin_x, origin_y, origin_yaw)

    def normalize_odom(self, st):
        raw_x = float(st.get("x", 0.0) or 0.0)
        raw_y = float(st.get("y", 0.0) or 0.0)
        raw_yaw = float(st.get("yaw", 0.0) or 0.0)
        if self.odom_origin is None:
            self.odom_origin = (raw_x, raw_y, raw_yaw)
        ox, oy, oyaw = self.odom_origin
        dx = raw_x - ox; dy = raw_y - oy
        c = math.cos(oyaw); s = math.sin(oyaw)
        return {"x": dx*c + dy*s, "y": -dx*s + dy*c, "yaw": self._yaw(raw_yaw - oyaw)}

    def integrate(self):
        now = time.monotonic()
        dt = max(0.0, min(now - self.state_time, 0.25))
        self.state_time = now
        if now >= self.teleop_active_until and now - self.last_odom_time < 0.3:
            return
        vx = float(self.state.get("vx", 0.0) or 0.0)
        wz = float(self.state.get("wz", 0.0) or 0.0)
        if abs(vx) < 0.0001 and abs(wz) < 0.0001:
            return
        yaw = float(self.state.get("yaw", 0.0) or 0.0) + wz * dt
        self.state["yaw"] = yaw
        self.state["x"] = float(self.state.get("x", 0.0) or 0.0) + vx * math.cos(yaw) * dt
        self.state["y"] = float(self.state.get("y", 0.0) or 0.0) + vx * math.sin(yaw) * dt


class Hub:
    def __init__(self):
        self.clients: Dict[str, Client] = {}
        self.host_id: Optional[str]     = None
        # Per-user robot environments — keyed by username. Each user drives
        # their own isolated robot; nothing crosses between users.
        self.envs: Dict[str, RobotEnv] = {}
        self._colors = ["#00e5ff","#00ff9d","#ff9500","#ff2d55","#a78bfa","#ffcc00","#34d399"]
        self._ci = 0

    def env_for(self, username: str) -> RobotEnv:
        if username not in self.envs:
            self.envs[username] = RobotEnv()
        return self.envs[username]

    def add(self, c: Client):
        c.color = self._colors[self._ci % len(self._colors)]
        self._ci += 1
        self.clients[c.id] = c
        # Role is authoritative from the auth token (set before add()):
        # only the real admin account connects with role 'host'. We NEVER
        # promote a client to host based on connection order.
        if c.role == "host":
            # Demote any stale host record and claim host.
            if self.host_id and self.host_id in self.clients and self.host_id != c.id:
                self.clients[self.host_id].role = "client"
            self.host_id = c.id

    def remove(self, cid: str):
        if cid in self.clients:
            c = self.clients.pop(cid)
            if c.pty_pid:
                try: os.kill(c.pty_pid, signal.SIGKILL)
                except: pass
        if self.host_id == cid:
            # Host left — there is no host until the admin reconnects.
            # Do NOT promote a client to host.
            self.host_id = None

    async def broadcast_host(self, msg: dict):
        """Send message to host only"""
        if self.host_id and self.host_id in self.clients:
            try:
                await self.clients[self.host_id].ws.send_json(msg)
            except: pass

    async def broadcast_all(self, msg: dict, exclude=None):
        for cid, c in list(self.clients.items()):
            if cid == exclude: continue
            try: await c.ws.send_json(msg)
            except: pass

    async def send_to_user(self, username: str, msg: dict):
        """Send a message to every connection belonging to one username
        (covers a user's multiple terminals/tabs in Stage 3)."""
        for c in list(self.clients.values()):
            if c.username == username:
                try: await c.ws.send_json(msg)
                except: pass

    def client_list(self):
        out = []
        for c in self.clients.values():
            entry = {"id": c.id, "name": c.name, "role": c.role,
                     "color": c.color, "cwd": c.cwd}
            # Surface which robot model each client currently has loaded, so the
            # host monitor can show it. Tracked per-user in container_manager;
            # absent for the host (no per-user sim there) and that's fine.
            st = container_manager.cs.get(c.name)
            mid = getattr(st, "model_id", None) if st else None
            if mid:
                spec = models.get_spec(mid)
                entry["model_id"] = mid
                entry["model_name"] = spec.name if spec else mid
            out.append(entry)
        return out

hub = Hub()

# ── PTY MANAGEMENT ───────────────────────────────────────────────────────────
DOCKER_CONTAINER = "ros_desktop"   # Docker container clients drop into
ROS_SETUP       = "/opt/ros/noetic/setup.bash"

def container_exists() -> bool:
    """Check if the Docker container is running"""
    import subprocess
    try:
        r = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", DOCKER_CONTAINER],
            capture_output=True, text=True, timeout=3
        )
        return r.stdout.strip() == "true"
    except:
        return False

class CmdVelPublisher:
    def __init__(self):
        self.proc = None

    def _start(self):
        script = r"""
import json
import sys
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('robot_web_ui_cmd_vel', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.sleep(0.2)

for line in sys.stdin:
    try:
        data = json.loads(line)
        msg = Twist()
        msg.linear.x = float(data.get('vx', 0.0) or 0.0)
        msg.angular.z = float(data.get('wz', 0.0) or 0.0)
        pub.publish(msg)
    except Exception:
        pass
"""
        if container_exists():
            cmd = [
                "docker", "exec", "-i", DOCKER_CONTAINER, "bash", "-lc",
                f"source {ROS_SETUP} 2>/dev/null; python3 -u -c {shlex.quote(script)}"
            ]
        else:
            cmd = ["bash", "-lc", f"source {ROS_SETUP} 2>/dev/null; python3 -u -c {shlex.quote(script)}"]

        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
        )

    def publish(self, vx: float, wz: float) -> bool:
        try:
            if self.proc is None or self.proc.poll() is not None:
                self._start()
            self.proc.stdin.write(json.dumps({"vx": vx, "wz": wz}) + "\n")
            self.proc.stdin.flush()
            return True
        except Exception:
            self.proc = None
            return False

cmd_vel_publisher = CmdVelPublisher()

def spawn_pty(cid: str, is_host: bool = False, username: str = None) -> tuple:
    """Host -> Mac shell. Client -> docker exec ros_desktop AS the Linux user
    <username>, landing in /home/<username> with a <username>@host prompt."""
    import shutil
    pid, fd = pty.fork()
    if pid == 0:
        env = os.environ.copy()
        env["TERM"] = "xterm-256color"
        # Encourage colored output in the host's Mac shell (BSD ls + grep).
        env["CLICOLOR"] = "1"
        env["CLICOLOR_FORCE"] = "1"
        env["LSCOLORS"] = "GxFxCxDxBxegedabagaced"
        env["GREP_OPTIONS"] = ""  # avoid deprecation noise

        Y  = "\033[01;33m"  # yellow
        C  = "\033[01;36m"  # cyan
        B  = "\033[1;38;2;38;54;137m"  # BITGREEN blue (#263689)
        R  = "\033[0m"      # reset
        import base64 as _b64
        def _banner(lines, col):
            w = max(len(s) for s in lines)
            border = "+" + "-" * (w + 4) + "+"
            cmds = ["echo '';", f"echo '{col}{border}{R}';"]
            for s in lines:
                cmds.append(f"echo '{col}|  {s.ljust(w)}  |{R}';")
            cmds += [f"echo '{col}{border}{R}';", "echo '';"]
            return "".join(cmds)
        def _b64rc(col, host="\\h"):
            # bash rcfile: load defaults, then recolor the prompt to match the banner.
            # `host` lets us display the client's username instead of the container id.
            rc = ("[ -f /etc/bash.bashrc ] && . /etc/bash.bashrc\n"
                  "[ -f \"$HOME/.bashrc\" ] && . \"$HOME/.bashrc\"\n"
                  "PS1='\\[" + col + "\\]\\u@" + host + ":\\w\\$\\[" + R + "\\] '\n")
            return _b64.b64encode(rc.encode()).decode()

        if is_host:
            shell = os.environ.get("SHELL", "/bin/bash")
            env["HOME"] = str(Path.home())
            os.chdir(env["HOME"])
            bash = shutil.which("bash") or "/bin/bash"
            # Gold prompt to match the banner. zsh reads $ZDOTDIR/.zshrc after the
            # system rc, so re-source the user's config, then recolor the prompt.
            try:
                _zd = Path(env["HOME"]) / ".bitgreen"
                _zd.mkdir(exist_ok=True)
                (_zd / ".zshrc").write_text(
                    '[ -f "$HOME/.zshrc" ] && source "$HOME/.zshrc"\n'
                    "PROMPT='%F{#263689}%n@%m %1~ %#%f '\n"
                )
                env["ZDOTDIR"] = str(_zd)
            except Exception:
                pass
            cmd = _banner([
                "BITGREEN ROBOT STUDIO  -  Host Terminal (Mac shell)",
                "Enter the ROS container:  docker exec -it ros_desktop bash",
                "Then source ROS:  source /opt/ros/noetic/setup.bash",
            ], B) + f"exec {shell}"
            os.execvpe(bash, [bash, "-c", cmd], env)
        else:
            # Determine the Linux username this shell should run as.
            uname = username if (username and linux_user._safe(username)) else None
            # Option A: a client's terminal drops into THEIR OWN container.
            import container_manager as _cm
            client_box = _cm.container_name(uname) if uname else None
            box_running = False
            if client_box:
                # Make sure their container is being started, then wait briefly
                # for it to come up. This prevents a reconnect from falling back
                # to the shared container (which would break isolation).
                try:
                    st = _cm._container_state(client_box)
                    if st in ("created", "exited", "paused", "dead"):
                        _cm._run(["docker", "start", client_box], timeout=40)
                    elif st == "none":
                        # let the manager create+start it
                        _cm.container_manager.ensure(uname)
                except Exception:
                    pass
                # Wait up to ~15s for the container to be running.
                _deadline = time.monotonic() + 15.0
                while time.monotonic() < _deadline:
                    try:
                        if _cm._container_state(client_box) == "running":
                            box_running = True
                            break
                    except Exception:
                        pass
                    time.sleep(0.5)
            if box_running:
                setup = (
                    _banner([
                        "BITGREEN ROBOT STUDIO  -  Your Container",
                        f"Box : {client_box}",
                        "ROS : Noetic (sourced)",
                    ], B)
                    + "source /opt/ros/noetic/setup.bash 2>/dev/null;"
                    + "export TURTLEBOT3_MODEL=burger;"
                    + f"echo {_b64rc(B, uname or chr(92)+chr(104))} | base64 -d > /root/.bitgreen_bashrc 2>/dev/null;"
                    + "cd /root 2>/dev/null;"
                    + "exec bash --rcfile /root/.bitgreen_bashrc"
                )
                args = ["docker", "exec", "-it", client_box, "bash", "-c", setup]
                os.execvpe("docker", args, env)
            elif container_exists():
                # Fallback: client's own container not up yet -> shared box.
                uname2 = uname
                run_as_user = False
                if uname2:
                    try:
                        if not linux_user.linux_user_exists(uname2):
                            linux_user.create_linux_user(uname2)
                        run_as_user = linux_user.linux_user_exists(uname2)
                    except Exception:
                        run_as_user = False
                banner = _banner([
                    "BITGREEN ROBOT STUDIO  -  Client Terminal",
                    "(your container is still starting)",
                ], B)
                if run_as_user:
                    setup = (
                        f"{banner}"
                        f"exec su - {shlex.quote(uname2)} -c {shlex.quote('bash -l')}"
                    )
                    args = ["docker", "exec", "-it", DOCKER_CONTAINER, "bash", "-c", setup]
                else:
                    setup = (
                        f"{banner}"
                        "source /opt/ros/noetic/setup.bash 2>/dev/null;"
                        "export TURTLEBOT3_MODEL=burger;"
                        f"echo {_b64rc(B, uname2 or chr(92)+chr(104))} | base64 -d > /root/.bitgreen_bashrc 2>/dev/null;"
                        "exec bash --rcfile /root/.bitgreen_bashrc"
                    )
                    args = ["docker", "exec", "-it", DOCKER_CONTAINER, "bash", "-c", setup]
                os.execvpe("docker", args, env)
            else:
                shell = os.environ.get("SHELL", "/bin/bash")
                env["HOME"] = str(Path.home())
                os.chdir(env["HOME"])
                os.execvpe(shell, [shell], env)
    else:
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        return pid, fd

def set_pty_size(fd: int, rows: int, cols: int):
    try:
        fcntl.ioctl(fd, termios.TIOCSWINSZ,
                    struct.pack("HHHH", rows, cols, 0, 0))
    except: pass

async def pty_reader(client: Client, term: "Terminal"):
    """Read one terminal's PTY output and send to the client's WebSocket,
    tagged with its terminal_id so the UI routes it to the right tab."""
    loop = asyncio.get_event_loop()
    while client.alive and term.fd:
        try:
            data = await loop.run_in_executor(None, _read_pty, term.fd)
            if data:
                b64 = base64.b64encode(data).decode()
                await client.ws.send_json({
                    "type": "terminal_output",
                    "terminal_id": term.tid,
                    "data": b64
                })
                # Mirror the RAW stream (uncut, base64) to the host, tagged with
                # terminal_id so the host shows one mirror tab per client terminal.
                if client.role != "host":
                    buf = client.mirror.setdefault(term.tid, [])
                    buf.append(b64)
                    if len(buf) > 400:
                        del buf[:len(buf)-400]
                    await hub.broadcast_host({
                        "type": "client_mirror",
                        "id": client.id,
                        "terminal_id": term.tid,
                        "data": b64,
                    })
                    # Clean (ANSI-stripped) one-liner for the LIVE ACTIVITY log.
                    clean = _strip_ansi(data.decode("utf-8", "replace")).strip()
                    if clean and len(clean) > 1:
                        snippet = clean.replace("\r", " ").replace("\n", " ")[:120]
                        client.activity = (client.activity + [snippet])[-20:]
                        await hub.broadcast_host({
                            "type": "client_activity",
                            "id": client.id,
                            "text": snippet,
                            "cwd": client.cwd
                        })
            else:
                await asyncio.sleep(0.02)
        except Exception:
            await asyncio.sleep(0.05)

import re as _re
_ANSI_RE = _re.compile(r"\x1b\[[0-9;?]*[ -/]*[@-~]|\x1b[\]P^_].*?(?:\x07|\x1b\\)|[\x00-\x08\x0b\x0c\x0e-\x1f]")
def _strip_ansi(s: str) -> str:
    return _ANSI_RE.sub("", s)

def _read_pty(fd):
    try:
        return os.read(fd, 4096)
    except (OSError, BlockingIOError):
        return b""

# ── FILE MANAGER API ─────────────────────────────────────────────────────────
ROS_ROOT = Path.home() / "ros_workspace"

def _require_auth(request: Request):
    """Validate the session token on an HTTP request. Returns {'username','role'}
    or raises 401. Token comes from Authorization: Bearer or ?token=."""
    h = request.headers.get("authorization", "")
    token = h[7:].strip() if h.lower().startswith("bearer ") else request.query_params.get("token", "")
    info = auth.validate_token(token)
    if not info:
        raise HTTPException(401, "Invalid or expired session.")
    return info

@app.post("/api/file/save")
async def save_file(request: Request):
    info = _require_auth(request)
    req = await request.json()
    path = req.get("path", "~")
    content = req.get("content", "")
    if info["role"] == "client":
        # Quota check before writing
        if linux_user.would_exceed(info["username"], len(content.encode("utf-8"))):
            raise HTTPException(413, "Storage quota exceeded (1 GB limit).")
        # Write inside the user's container home (/home/<username>)
        ok, res = linux_user.write_file(info["username"], path, content)
        if not ok:
            raise HTTPException(400, str(res))
        return {"ok": True, "path": res}
    # Host: full Mac-filesystem access (unchanged)
    try:
        p = Path(path).expanduser().resolve()
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(content)
        return {"ok": True, "path": str(p)}
    except Exception as e:
        raise HTTPException(400, str(e))

@app.get("/api/files")
async def list_files(request: Request, path: str = "~"):
    info = _require_auth(request)
    if info["role"] == "client":
        # List inside the user's container home, jailed to /home/<username>
        return linux_user.list_dir(info["username"], path)
    # Host: full Mac-filesystem access (unchanged)
    try:
        p = Path(path).expanduser().resolve()
        if not p.exists(): raise HTTPException(404, "Not found")
        items = []
        for item in sorted(p.iterdir(), key=lambda x: (not x.is_dir(), x.name.lower())):
            try:
                stat = item.stat()
                items.append({
                    "name":  item.name,
                    "path":  str(item),
                    "type":  "dir" if item.is_dir() else "file",
                    "size":  stat.st_size,
                    "mtime": stat.st_mtime,
                    "ext":   item.suffix.lower()
                })
            except: pass
        return {"path": str(p), "parent": str(p.parent), "items": items}
    except HTTPException: raise
    except Exception as e:
        raise HTTPException(400, str(e))

@app.post("/api/file/upload")
async def upload_file(request: Request):
    info = _require_auth(request)
    try:
        form = await request.form()
        file = form.get("file")
        path = form.get("path", "~")
        if not file:
            return JSONResponse({"ok": False, "error": "No file provided."}, status_code=400)
        content_bytes = await file.read()
        if info["role"] == "client":
            # Quota check before writing
            if linux_user.would_exceed(info["username"], len(content_bytes)):
                return JSONResponse({"ok": False, "error": "Storage quota exceeded (1 GB limit)."}, status_code=413)
            # Place the upload inside the user's container home
            dest_path = path.rstrip("/") + "/" + file.filename if path not in ("~", "") else "~/" + file.filename
            ok, res = linux_user.write_file_bytes(info["username"], dest_path, content_bytes)
            if not ok:
                return JSONResponse({"ok": False, "error": str(res)}, status_code=400)
            return {"ok": True, "path": res, "name": file.filename}
        # Host: Mac filesystem
        dest = Path(path).expanduser().resolve() / file.filename
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(content_bytes)
        return {"ok": True, "path": str(dest), "name": file.filename}
    except Exception as e:
        # Always return JSON so the client never tries to parse an HTML 500 page.
        return JSONResponse({"ok": False, "error": f"Upload failed: {e}"}, status_code=500)

@app.get("/api/file/read")
async def read_file(request: Request, path: str):
    info = _require_auth(request)
    if info["role"] == "client":
        content, err = linux_user.read_file(info["username"], path)
        if err:
            raise HTTPException(404 if err == "not found" else 400, err)
        return {"path": path, "content": content, "size": len(content)}
    # Host: Mac filesystem
    try:
        p = Path(path).expanduser().resolve()
        if not p.is_file(): raise HTTPException(404)
        if p.stat().st_size > 500_000:
            raise HTTPException(413, "File too large (>500KB)")
        content = p.read_text(errors="replace")
        return {"path": str(p), "content": content, "size": p.stat().st_size}
    except HTTPException: raise
    except Exception as e: raise HTTPException(400, str(e))

@app.get("/api/file/download")
async def download_file(request: Request, path: str):
    info = _require_auth(request)
    if info["role"] == "client":
        data = linux_user.read_file_bytes(info["username"], path)
        if data is None:
            raise HTTPException(404, "Not found")
        fname = path.rstrip("/").split("/")[-1] or "download"
        return Response(content=data, media_type="application/octet-stream",
                        headers={"Content-Disposition": f'attachment; filename="{fname}"'})
    # Host: Mac filesystem
    p = Path(path).expanduser().resolve()
    if not p.is_file(): raise HTTPException(404)
    return FileResponse(str(p), filename=p.name)

@app.get("/api/clients")
async def get_clients():
    return hub.client_list()

@app.post("/api/file/delete")
async def delete_file_endpoint(request: Request):
    info = _require_auth(request)
    body = await request.json()
    path = body.get("path", "")
    if info["role"] == "client":
        ok, res = linux_user.delete_path(info["username"], path)
        if not ok:
            return JSONResponse({"ok": False, "error": str(res)}, status_code=400)
        return {"ok": True, "path": res}
    # Host: Mac filesystem
    try:
        import shutil
        p = Path(path).expanduser().resolve()
        if not p.exists():
            return JSONResponse({"ok": False, "error": "Not found"}, status_code=404)
        if p.is_dir(): shutil.rmtree(p)
        else: p.unlink()
        return {"ok": True, "path": str(p)}
    except Exception as e:
        return JSONResponse({"ok": False, "error": str(e)}, status_code=400)

# ── QUOTA (soft 1 GB per client) ────────────────────────────────────────────
@app.get("/api/quota")
async def api_quota(request: Request):
    info = _require_auth(request)
    if info["role"] == "host":
        # Host uses the Mac filesystem; quota doesn't apply.
        return {"used": 0, "limit": 0, "over": False, "pct": 0, "applies": False}
    q = linux_user.quota_info(info["username"])
    q["applies"] = True
    return q

# ── HOST-ONLY: registered users + delete ──────────────────────────────────────
def _require_host(request: Request):
    info = _require_auth(request)
    if info["role"] != "host":
        raise HTTPException(403, "Host only.")
    return info

@app.get("/api/users")
async def api_users(request: Request):
    _require_host(request)
    online = {c.username for c in hub.clients.values()}
    users = []
    for u in auth.all_users():
        users.append({
            "username": u["username"],
            "email":    u["email"],
            "role":     u["role"],
            "online":   u["username"] in online,
            "is_admin": (u["username"] == auth.ADMIN_USERNAME),
        })
    # Sort: online first, then alphabetical
    users.sort(key=lambda x: (not x["online"], x["username"]))
    return {"users": users}

@app.post("/api/admin/delete_user")
async def api_delete_user(request: Request):
    _require_host(request)
    body = await request.json()
    target = (body.get("username", "") or "").strip().lower()
    if not target:
        raise HTTPException(400, "No username.")
    if target == auth.ADMIN_USERNAME:
        raise HTTPException(400, "The admin account cannot be deleted.")
    # 1) Kill any live sessions: close their websocket connections.
    for c in list(hub.clients.values()):
        if c.username == target:
            try:
                await c.ws.send_json({"type": "kicked", "msg": "Your account was removed by the host."})
                await c.ws.close()
            except Exception:
                pass
    # 2) Remove the Linux user + home (reclaims their space).
    try:
        linux_user.delete_linux_user(target)
    except Exception as e:
        print(f"[delete_user] linux removal warning for {target}: {e}")
    # 3) Remove their per-user robot env.
    hub.envs.pop(target, None)
    # 4) Delete the DB account + sessions.
    ok, msg = auth.delete_user(target)
    if not ok:
        raise HTTPException(400, msg)
    return {"ok": True, "username": target}

# ── HOST-ONLY: browse/edit/delete a client's container home ──────────────────
def _valid_client(username: str) -> bool:
    username = (username or "").strip().lower()
    if not username or username == auth.ADMIN_USERNAME:
        return False
    return any(u["username"] == username for u in auth.all_users())

@app.get("/api/admin/files")
async def admin_files(request: Request, user: str, path: str = "~"):
    _require_host(request)
    if not _valid_client(user):
        raise HTTPException(404, "No such client.")
    return linux_user.list_dir(user, path)

@app.get("/api/admin/file/read")
async def admin_read(request: Request, user: str, path: str):
    _require_host(request)
    if not _valid_client(user):
        raise HTTPException(404, "No such client.")
    content, err = linux_user.read_file(user, path)
    if err:
        raise HTTPException(404 if err == "not found" else 400, err)
    return {"path": path, "content": content, "size": len(content)}

@app.post("/api/admin/file/save")
async def admin_save(request: Request):
    _require_host(request)
    body = await request.json()
    user = (body.get("user", "") or "").strip().lower()
    if not _valid_client(user):
        raise HTTPException(404, "No such client.")
    ok, res = linux_user.write_file(user, body.get("path", "~"), body.get("content", ""))
    if not ok:
        raise HTTPException(400, str(res))
    return {"ok": True, "path": res}

@app.post("/api/admin/file/delete")
async def admin_delete_file(request: Request):
    _require_host(request)
    body = await request.json()
    user = (body.get("user", "") or "").strip().lower()
    if not _valid_client(user):
        raise HTTPException(404, "No such client.")
    ok, res = linux_user.delete_path(user, body.get("path", ""))
    if not ok:
        raise HTTPException(400, str(res))
    return {"ok": True, "path": res}

@app.get("/api/admin/file/download")
async def admin_download(request: Request, user: str, path: str):
    _require_host(request)
    if not _valid_client(user):
        raise HTTPException(404, "No such client.")
    data = linux_user.read_file_bytes(user, path)
    if data is None:
        raise HTTPException(404, "Not found")
    fname = path.rstrip("/").split("/")[-1] or "download"
    return Response(content=data, media_type="application/octet-stream",
                    headers={"Content-Disposition": f'attachment; filename="{fname}"'})
# ── AUTH API (Stage 1) ────────────────────────────────────────────────────────
def _bearer(request: Request) -> str:
    # Accept token from Authorization header or ?token= query param
    h = request.headers.get("authorization", "")
    if h.lower().startswith("bearer "):
        return h[7:].strip()
    return request.query_params.get("token", "")

@app.post("/api/signup")
async def api_signup(request: Request):
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"ok": False, "error": "Invalid request."}, status_code=400)
    ok, res = auth.begin_signup(
        body.get("email", ""), body.get("username", ""), body.get("password", "")
    )
    if not ok:
        return JSONResponse({"ok": False, "error": res}, status_code=400)
    # Admin is exempt from OTP — account created directly, Linux user too.
    if isinstance(res, dict) and res.get("admin"):
        try:
            linux_user.create_linux_user(res["username"])
        except Exception as e:
            print(f"[signup] linux user create warning for {res['username']}: {e}")
        return {"ok": True, "verified": True, "username": res["username"]}
    # Client: pending OTP verification — no account yet.
    return {"ok": True, "verified": False, "email": res["email"], "username": res["username"]}

@app.post("/api/verify_otp")
async def api_verify_otp(request: Request):
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"ok": False, "error": "Invalid request."}, status_code=400)
    ok, res = auth.verify_otp(body.get("email", ""), body.get("code", ""))
    if not ok:
        return JSONResponse({"ok": False, "error": res}, status_code=400)
    # Verified — now create the Linux user/home for this client.
    try:
        linux_user.create_linux_user(res["username"])
    except Exception as e:
        print(f"[verify_otp] linux user create warning for {res['username']}: {e}")
    # Option A: pre-create this client's personal container (STOPPED — it starts
    # on first login). Cheap at registration; Gazebo only runs when they connect.
    if res.get("role") != "host":
        try:
            cres = container_manager.create(res["username"])
            if not cres.get("ok"):
                print(f"[verify_otp] container create warning for {res['username']}: {cres.get('error')}")
        except Exception as e:
            print(f"[verify_otp] container create warning for {res['username']}: {e}")
    return {"ok": True, "username": res["username"], "role": res["role"]}

@app.post("/api/resend_otp")
async def api_resend_otp(request: Request):
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"ok": False, "error": "Invalid request."}, status_code=400)
    ok, msg = auth.resend_otp(body.get("email", ""))
    if not ok:
        return JSONResponse({"ok": False, "error": msg}, status_code=400)
    return {"ok": True, "message": msg}

@app.post("/api/login")
async def api_login(request: Request):
    try:
        body = await request.json()
    except Exception:
        return JSONResponse({"ok": False, "error": "Invalid request."}, status_code=400)
    ok, res = auth.login(body.get("username", ""), body.get("password", ""))
    if not ok:
        return JSONResponse({"ok": False, "error": res}, status_code=401)
    return {"ok": True, **res}

@app.get("/api/me")
async def api_me(request: Request):
    info = auth.validate_token(_bearer(request))
    if not info:
        return JSONResponse({"ok": False, "error": "Invalid or expired session."}, status_code=401)
    return {"ok": True, **info}

@app.post("/api/logout")
async def api_logout(request: Request):
    auth.logout(_bearer(request))
    return {"ok": True}

# ── PAGE ROUTES ───────────────────────────────────────────────────────────────
@app.get("/")
async def index():
    return FileResponse(str(STATIC / "index.html"))

@app.get("/host")
async def host_page():
    return FileResponse(str(STATIC / "host.html"))

@app.get("/client")
async def client_page():
    return FileResponse(str(STATIC / "index.html"))

@app.get("/login")
async def login_page():
    return FileResponse(str(STATIC / "login.html"))

@app.get("/signup")
async def signup_page():
    return FileResponse(str(STATIC / "signup.html"))

@app.get("/favicon.ico", include_in_schema=False)
async def favicon():
    return Response(status_code=204)

# ── Stage 4: per-client Gazebo odom feedback + idle GC ───────────────────────
async def gazebo_odom_loop():
    """Poll each ready client sim's /odom and push it to that client's 3D view
    so the robot + trail draw from REAL robot motion. read_odom is a blocking
    docker exec, so we run it in a thread executor. Also GC idle sims."""
    loop = asyncio.get_event_loop()
    gc_counter = 0
    scan_counter = 0
    while True:
        try:
            # Throttle LIDAR to roughly half the odom rate (~2.5 Hz): a scan is
            # ~360 floats, far heavier than a pose, and bandwidth/CPU matter under
            # emulation. Downsampled to every Nth beam below.
            scan_counter += 1
            do_scan = (scan_counter % 2 == 0)
            # snapshot usernames with a ready container
            ready = [u for u, c in list(container_manager.cs.items())
                     if c.state == "ready"]
            for uname in ready:
                # only bother if that user is currently connected
                if not any(c.username == uname for c in hub.clients.values()):
                    continue
                # Tell the client once when their sim first becomes ready.
                _cc = container_manager.cs.get(uname)
                if _cc and not getattr(_cc, "_notified_ready", False):
                    _cc._notified_ready = True
                    await hub.send_to_user(uname, {
                        "type": "sim_status",
                        "sim": {"state": "ready"},
                    })
                # ── Arm models: stream joint angles instead of odom/scan. An arm
                #    has no /odom, so this branch fully replaces the mobile path. ──
                if getattr(_cc, "is_arm", False):
                    joints = await loop.run_in_executor(
                        None, container_manager.read_joints, uname)
                    if joints:
                        await hub.send_to_user(uname, {
                            "type": "joint_state",
                            "positions": joints,
                            "names": list(getattr(_cc, "joint_names", ()) or ()),
                        })
                    continue
                odom = await loop.run_in_executor(None, container_manager.read_odom, uname)
                if not odom:
                    continue
                env = hub.env_for(uname)
                # Treat real odom like the host's odom path: normalize to the
                # session origin so the view starts at (0,0).
                try:
                    norm = env.normalize_odom(odom)
                except (TypeError, ValueError):
                    continue
                env.state["x"] = norm["x"]
                env.state["y"] = norm["y"]
                env.state["yaw"] = norm["yaw"]
                await hub.send_to_user(uname, {
                    "type": "robot_state",
                    "state": {
                        "x": norm["x"], "y": norm["y"], "yaw": norm["yaw"],
                        "vx": env.state.get("vx", 0.0),
                        "wz": env.state.get("wz", 0.0),
                    },
                    "source": "odom",
                })
                # ── LIDAR (Track B-B): only for models with a lidar, throttled ──
                if do_scan and getattr(_cc, "has_lidar", False):
                    scan = await loop.run_in_executor(
                        None, container_manager.read_scan, uname)
                    if scan and scan.get("ranges"):
                        step = 4   # send every 4th beam (360 -> ~90 points)
                        await hub.send_to_user(uname, {
                            "type": "scan",
                            "ranges": scan["ranges"][::step],
                            "angle_min": scan.get("angle_min", 0.0),
                            "angle_increment": scan.get("angle_increment", 0.0),
                            "step": step,
                        })
            # idle GC every ~150 cycles (~30s at 5Hz)
            gc_counter += 1
            if gc_counter >= 150:
                gc_counter = 0
                await loop.run_in_executor(None, container_manager.gc_idle, 180.0)
        except Exception:
            pass
        await asyncio.sleep(0.2)   # 5 Hz odom poll for smoother motion + denser path


@app.on_event("startup")
async def _start_background_tasks():
    asyncio.create_task(gazebo_odom_loop())


# ── MAIN WEBSOCKET ────────────────────────────────────────────────────────────
@app.websocket("/ws/{client_id}")
async def websocket_endpoint(ws: WebSocket, client_id: str):
    await ws.accept()

    # ── Stage 1: authenticate via session token (?token=...) ─────────────────
    token = ws.query_params.get("token", "")
    info = auth.validate_token(token)
    if not info:
        try:
            await ws.send_json({"type": "auth_error", "msg": "Invalid or expired session. Please log in again."})
        except Exception:
            pass
        await ws.close(code=4401)
        return

    username = info["username"]
    role = info["role"]
    # Build the connection id from the authenticated identity. Host keeps the
    # '_host' suffix so the existing role/host detection continues to work.
    conn_id = (username + "_host") if role == "host" else username

    client = Client(conn_id, ws)
    client.username = username
    client.name = username
    if role == "host":
        client.role = "host"
    hub.add(client)

    is_host = (client.role == "host") or conn_id.endswith("_host")

    # ── Stage 4: per-client Gazebo ────────────────────────────────────────────
    # Each CLIENT gets their own isolated Gazebo (real ROS, own robot). The HOST
    # works in the shared ros_desktop container; binding it here means a host
    # model switch (re)launches the selected robot's full stack — Gazebo world,
    # controllers, nodes and topics — right there, the same machinery clients use.
    try:
        if is_host:
            container_manager.ensure(
                username,
                name=container_manager.HOST_CONTAINER,   # "ros_desktop"
                is_host=True,
            )
        else:
            container_manager.ensure(username)   # client: own isolated container
    except Exception:
        pass

    def open_terminal(tid: str):
        """Spawn a PTY for terminal id `tid` and start its reader."""
        try:
            pid, fd = spawn_pty(conn_id, is_host=is_host, username=username)
        except Exception as e:
            return None, str(e)
        t = Terminal(tid)
        t.pid, t.fd = pid, fd
        client.terminals[tid] = t
        # keep back-compat single-PTY pointers on the first terminal
        if client.pty_fd is None:
            client.pty_fd, client.pty_pid = fd, pid
        t.reader = asyncio.create_task(pty_reader(client, t))
        return t, None

    def close_terminal(tid: str):
        t = client.terminals.pop(tid, None)
        if not t:
            return
        if t.reader:
            t.reader.cancel()
        if t.pid:
            try: os.kill(t.pid, signal.SIGKILL)
            except: pass
        if t.fd:
            try: os.close(t.fd)
            except: pass
        # if we closed the primary, repoint back-compat fields
        if client.pty_fd == t.fd:
            nxt = next(iter(client.terminals.values()), None)
            client.pty_fd  = nxt.fd  if nxt else None
            client.pty_pid = nxt.pid if nxt else None
        # Drop the host-mirror buffer for this terminal and tell the host.
        client.mirror.pop(tid, None)
        if client.role != "host":
            asyncio.create_task(hub.broadcast_host({
                "type": "client_mirror_closed",
                "id": client.id,
                "terminal_id": tid,
            }))

    # Open the first terminal ('t1')
    t, err = open_terminal("t1")
    if err:
        await ws.send_json({"type":"error","msg": f"PTY failed: {err}"})

    # Notify everyone
    await ws.send_json({
        "type": "welcome",
        "id":   client.id,
        "username": username,
        "role": client.role,
        "color": client.color,
        "clients": hub.client_list(),
        "robot_state": hub.env_for(username).state,
        "sim": container_manager.status(username),
        # Track A1: model picker catalog. Host AND clients pick from this shared,
        # server-owned library. `sim.model_id` says which one is currently loaded.
        "models": models.catalog_public(),
        # Navigation state, so the client UI restores it on (re)connect instead
        # of silently reverting the goal box to relative-move.
        "nav_on": container_manager.nav_status(username),
    })
    await hub.broadcast_all({"type":"client_joined","id":client.id,"color":client.color,"role":client.role}, exclude=conn_id)
    # Also push the full updated client list so every dashboard's count/list
    # refreshes immediately (no manual refresh needed).
    await hub.broadcast_all({"type":"clients_update","clients":hub.client_list()})

    # Use conn_id for cleanup below
    client_id = conn_id

    try:
        def _term_for(m):
            """Resolve the target Terminal from a message's terminal_id,
            defaulting to 't1' for backward compatibility."""
            tid = m.get("terminal_id", "t1")
            return client.terminals.get(tid)

        while True:
            msg = await ws.receive_json()
            mtype = msg.get("type","")

            # ── Open a new terminal ───────────────────────────────────────────
            if mtype == "terminal_create":
                tid = msg.get("terminal_id")
                if tid and tid not in client.terminals:
                    _t, err = open_terminal(tid)
                    if err:
                        await ws.send_json({"type":"error","msg":f"PTY failed: {err}"})
                    else:
                        await ws.send_json({"type":"terminal_created","terminal_id":tid})

            # ── Close a terminal ──────────────────────────────────────────────
            elif mtype == "terminal_close":
                tid = msg.get("terminal_id")
                if tid and tid in client.terminals and len(client.terminals) > 1:
                    close_terminal(tid)
                    await ws.send_json({"type":"terminal_closed","terminal_id":tid})

            # ── Terminal input ────────────────────────────────────────────────
            elif mtype == "terminal_input":
                t = _term_for(msg)
                if t and t.fd:
                    os.write(t.fd, base64.b64decode(msg["data"]))

            # ── Terminal resize ───────────────────────────────────────────────
            elif mtype == "terminal_resize":
                t = _term_for(msg)
                if t and t.fd:
                    set_pty_size(t.fd, msg.get("rows",24), msg.get("cols",80))

            # ── Run script ───────────────────────────────────────────────────
            elif mtype == "run_script":
                t = _term_for(msg)
                if t and t.fd:
                    script = msg.get("code", "").strip()
                    if script:
                        _is_host = (client.role == "host") or str(client.id).endswith("_host")
                        # Target container: host -> shared ros_desktop; client ->
                        # their OWN container (Option A). Write the file there
                        # out-of-band, then run it there, streaming output to PTY.
                        tmp = "/tmp/ros_ops_run.py"
                        if _is_host:
                            target = DOCKER_CONTAINER
                        else:
                            target = container_manager.exec_prefix(username)  # their container name or None
                        wrote = False
                        try:
                            if target:
                                p = subprocess.run(
                                    ["docker", "exec", "-i", target,
                                     "bash", "-c", "cat > " + tmp + " && chmod 644 " + tmp],
                                    input=script.encode(), timeout=10,
                                )
                                wrote = (p.returncode == 0)
                            else:
                                os.write(t.fd, b"echo 'Your simulator is still starting; try again in a moment.'\n")
                        except Exception as e:
                            os.write(t.fd, ("echo 'run_script: could not write file: %s'\n" % e).encode())

                        if wrote:
                            # Inherit the CURRENT model's env (e.g. TURTLEBOT3_MODEL
                            # for TB3; nothing for the UR5) instead of hard-coding it.
                            menv = container_manager.model_env_exports(username)
                            if _is_host:
                                # Host PTY is the Mac shell -> docker exec into
                                # the shared container to reach ROS + the file.
                                run_cmd = (
                                    "docker exec " + target + " bash -lc "
                                    "'source /opt/ros/noetic/setup.bash 2>/dev/null; "
                                    + menv +
                                    "python3 " + tmp + "'\n"
                                )
                            else:
                                # Client PTY is ALREADY inside their container, so
                                # run directly — no docker exec (there's no docker
                                # binary inside the container).
                                run_cmd = (
                                    "source /opt/ros/noetic/setup.bash 2>/dev/null; "
                                    + menv +
                                    "python3 " + tmp + "\n"
                                )
                            os.write(t.fd, run_cmd.encode())

            # ── CWD update ───────────────────────────────────────────────────
            elif mtype == "cwd_update":
                client.cwd = msg.get("cwd", client.cwd)
                await hub.broadcast_host({"type":"client_cwd","id":client.id,"cwd":client.cwd})

            # ── Go-to-goal (client double-clicked odom to set a target) ───────
            elif mtype == "goto":
                # Option A: host drives its OWN container sim, same as a client.
                if True:
                    try:
                        gx = float(msg.get("x"))
                        gy = float(msg.get("y"))
                    except (TypeError, ValueError):
                        gx = gy = None
                    gyaw = msg.get("yaw", None)
                    if gx is None or gy is None:
                        await ws.send_json({"type": "goto_status",
                                            "text": "Invalid target."})
                    elif not container_manager.is_ready(username):
                        await ws.send_json({"type": "goto_status",
                                            "text": "Your sim isn't ready yet."})
                    else:
                        _loop = asyncio.get_event_loop()
                        _uname = username
                        def _done(reason, info=None, _ln=_loop, _u=_uname):
                            if reason == "reached":
                                txt = "Reached target ✓"
                            elif reason == "stopped_short":
                                txt = "Stopped near target"
                            elif reason == "cancelled":
                                txt = "Stopped."
                            elif reason == "timeout":
                                txt = "Timed out before reaching target."
                            else:
                                txt = "Could not start go-to."
                            if info and info.get("pos_err") is not None:
                                txt += f" (off by {info['pos_err']*100:.0f} cm"
                                if info.get("yaw_err") is not None:
                                    txt += f", {info['yaw_err']:.0f}°"
                                txt += ")"
                            fut = asyncio.run_coroutine_threadsafe(
                                hub.send_to_user(_u, {"type": "goto_status", "text": txt}), _ln)
                            try: fut.result(timeout=2)
                            except Exception: pass
                        def _phase(text, _ln=_loop, _u=_uname):
                            fut = asyncio.run_coroutine_threadsafe(
                                hub.send_to_user(_u, {"type": "goto_status", "text": text}), _ln)
                            try: fut.result(timeout=2)
                            except Exception: pass
                        container_manager.goto(username, gx, gy, gyaw,
                                               on_done=_done, on_phase=_phase)
                        await ws.send_json({"type": "goto_status",
                                            "text": "Driving to target…"})

            # ── Obstacle Detection auto-drive (emanual 10.2) ──────────────────
            elif mtype == "obstacle_mode":
                # Option A: host uses its own container sim, like a client.
                want_on = bool(msg.get("on", False))
                if not container_manager.is_ready(username):
                    await ws.send_json({"type": "obstacle_status", "on": False,
                                        "text": "Your sim isn't ready yet."})
                elif want_on and not getattr(
                        container_manager.cs.get(username), "has_lidar", False):
                    await ws.send_json({"type": "obstacle_status", "on": False,
                                        "text": "This robot has no LIDAR."})
                else:
                    loop = asyncio.get_event_loop()
                    if want_on:
                        ok = await loop.run_in_executor(
                            None, container_manager.start_obstacle_mode, username)
                        _sd = container_manager.ContainerManager.AUTO_STOP_DIST
                        await ws.send_json({
                            "type": "obstacle_status", "on": bool(ok),
                            "text": (f"Obstacle detection ON — driving forward, "
                                     f"holds heading straight, stops within {_sd:g} m "
                                     f"of anything ahead (steer to continue).")
                                    if ok else "Could not start obstacle mode."})
                    else:
                        await loop.run_in_executor(
                            None, container_manager.stop_obstacle_mode, username)
                        await ws.send_json({"type": "obstacle_status", "on": False,
                                            "text": "Obstacle detection OFF."})

            # ── ROS Navigation (move_base): set a goal, plans around obstacles ──
            elif mtype == "nav_mode":
                # Option A: host uses its own container sim, like a client.
                want_on = bool(msg.get("on", False))
                loop = asyncio.get_event_loop()
                if want_on and not container_manager.cs.get(username):
                    await ws.send_json({"type": "nav_status", "on": False,
                                        "text": "Your sim isn't ready yet."})
                elif want_on and not models.supports_navigation(
                        getattr(container_manager.cs.get(username), "model_id", "")):
                    await ws.send_json({"type": "nav_status", "on": False,
                                        "text": "This robot/world doesn't support navigation."})
                elif want_on:
                    await ws.send_json({"type": "nav_status", "on": False,
                                        "text": "Starting navigation stack (move_base)… ~20s."})
                    ok = await loop.run_in_executor(
                        None, container_manager.start_navigation, username)
                    await ws.send_json({
                        "type": "nav_status", "on": bool(ok),
                        "text": ("Navigation ready — double-click an odometry value to "
                                 "set a goal; move_base will plan a path around the pillars.")
                                if ok else "Could not start navigation."})
                else:
                    await loop.run_in_executor(
                        None, container_manager.stop_navigation, username)
                    await ws.send_json({"type": "nav_status", "on": False,
                                        "text": "Navigation OFF."})

            elif mtype == "nav_goal":
                if container_manager.nav_status(username):
                    try:
                        gx = float(msg.get("x"))
                        gy = float(msg.get("y"))
                        gyaw = float(msg.get("yaw") or 0.0)
                    except (TypeError, ValueError):
                        gx = gy = gyaw = None
                    if gx is not None:
                        loop = asyncio.get_event_loop()
                        await loop.run_in_executor(
                            None, container_manager.send_nav_goal, username, gx, gy, gyaw)
                        await ws.send_json({
                            "type": "nav_status", "on": True,
                            "text": f"Navigating to ({gx:.1f}, {gy:.1f}) — planning around obstacles…"})
                else:
                    await ws.send_json({"type": "nav_status", "on": False,
                                        "text": "Turn Navigation on first."})

            # ── STOP: kill running script + cancel goto + zero velocity ───────
            elif mtype == "stop_motion":
                is_host = (client.role == "host") or str(client.id).endswith("_host")
                loop = asyncio.get_event_loop()
                # Option A: stop THIS user's OWN container robot (host + clients).
                await loop.run_in_executor(None, container_manager.stop_motion, username)
                # The host's run_script still executes inside the shared ros_desktop,
                # so also SIGKILL any leftover script there (rospy ignores SIGTERM).
                if is_host:
                    def _kill_host_script():
                        try:
                            subprocess.run(
                                ["docker", "exec", DOCKER_CONTAINER, "bash", "-lc",
                                 "pkill -9 -f /tmp/ros_ops_run.py 2>/dev/null; "
                                 "pkill -9 -f ros_ops_run 2>/dev/null; true"],
                                timeout=8,
                            )
                        except Exception:
                            pass
                    await loop.run_in_executor(None, _kill_host_script)
                # Ctrl-C this user's PTY too (their Python may run in that shell).
                try:
                    for t in client.terminals.values():
                        if t and t.fd:
                            os.write(t.fd, b"\x03")
                except Exception:
                    pass
                # reflect zeroed velocity in this user's env
                try:
                    env = hub.env_for(username)
                    env.state["vx"] = 0.0
                    env.state["wz"] = 0.0
                except Exception:
                    pass

            # ── Robot state — PER-USER isolated environment ──────────────────
            elif mtype == "robot_state":
                state  = msg.get("state", {})
                source = msg.get("source", "teleop")  # 'teleop' | 'code' | 'cmd_vel' | 'odom'
                now = time.monotonic()
                env = hub.env_for(username)        # this user's own robot
                is_host = (client.role == "host")

                if source in ("teleop", "code"):
                    try:
                        vx = float(state.get("vx", env.state.get("vx", 0.0)) or 0.0)
                        wz = float(state.get("wz", env.state.get("wz", 0.0)) or 0.0)
                    except (TypeError, ValueError):
                        vx = wz = 0.0
                    # Option A: host and clients BOTH drive their OWN container.
                    # If obstacle mode is on, manual driving STEERS it (turn /
                    # reverse take over briefly; forward into an obstacle is
                    # blocked) instead of cancelling it. Otherwise drive directly.
                    if container_manager.auto_status(username):
                        container_manager.auto_steer(username, vx, wz)
                    else:
                        # Drives THEIR OWN Gazebo robot if it's ready; otherwise
                        # falls back to the JS kinematic sim (sim still booting).
                        # publish returns False if not ready.
                        container_manager.publish(username, vx, wz)
                    env.state["vx"] = vx
                    env.state["wz"] = wz
                    moving = abs(vx) > 0.001 or abs(wz) > 0.001
                    env.teleop_active_until = now + 0.5 if moving else now
                    if moving:
                        env._was_teleoping = True
                elif source == "cmd_vel":
                    # Real-robot cmd_vel relay only matters for the host's env.
                    if now < env.teleop_active_until or now - env.cmd_vel_relay_time < 0.04:
                        continue
                    env.cmd_vel_relay_time = now

                if source == "odom":
                    # A user with a READY container gets odom from the server-side
                    # odom loop, so ignore any browser-relayed odom (host included).
                    if (not is_host) or container_manager.is_ready(username):
                        continue
                    if now < env.teleop_active_until:
                        env.last_odom_time = now
                        continue
                    if env._was_teleoping:
                        env._was_teleoping = False
                        env.reanchor_odom(state)
                    try:
                        state = env.normalize_odom(state)
                    except (TypeError, ValueError):
                        continue

                # When a user has a READY real Gazebo (host or client), the real
                # /odom (pushed by the odom loop) is the ONLY position source.
                # Running the JS kinematic integration too would fight it (jerk).
                client_has_real_sim = container_manager.is_ready(username)

                if source in ("teleop", "code", "cmd_vel") and not client_has_real_sim:
                    env.integrate()

                if not client_has_real_sim:
                    for key in ("x", "y", "yaw"):
                        if key in state:
                            try:
                                env.state[key] = float(state[key])
                            except (TypeError, ValueError):
                                pass

                if source == "odom":
                    env.last_odom_time = now
                    env.state_time = now

                if source == "cmd_vel":
                    for key in ("vx", "wz"):
                        if key in state:
                            try:
                                env.state[key] = float(state[key])
                            except (TypeError, ValueError):
                                pass

                # For a client with a real sim, only emit velocity echoes here;
                # the position comes from the odom loop. Skip re-broadcasting a
                # stale JS position on every teleop tick (that's the jerk).
                if client_has_real_sim and source in ("teleop", "code"):
                    await hub.send_to_user(username, {
                        "type": "robot_state",
                        "state": {"vx": env.state.get("vx", 0.0),
                                  "wz": env.state.get("wz", 0.0)},
                        "source": "vel",
                    })
                    continue

                broadcast_state = {
                    "x":   env.state["x"],
                    "y":   env.state["y"],
                    "yaw": env.state["yaw"],
                    "vx":  env.state.get("vx", 0.0),
                    "wz":  env.state.get("wz", 0.0),
                }

                # Send back ONLY to this user's own connections (isolation).
                await hub.send_to_user(username, {
                    "type":  "robot_state",
                    "state": broadcast_state,
                    "source": source,
                })

            # ── Chat/broadcast ────────────────────────────────────────────────
            elif mtype == "broadcast":
                await hub.broadcast_all({
                    "type":"broadcast","from":client.id,
                    "color":client.color,"text":msg.get("text","")
                })

            # ── Host → kick client ────────────────────────────────────────────
            elif mtype == "kick" and client.role == "host":
                target = msg.get("target")
                if target and target in hub.clients:
                    await hub.clients[target].ws.send_json({"type":"kicked"})

            # ── Host requests a client's buffered mirror (replay) ──────────────
            elif mtype == "request_mirror" and client.role == "host":
                target = msg.get("target")
                tc = hub.clients.get(target)
                if tc and tc.role != "host":
                    # Send the list of terminals + their buffered scrollback.
                    payload = {}
                    for tid, chunks in tc.mirror.items():
                        payload[tid] = chunks[-400:]
                    await ws.send_json({
                        "type": "mirror_history",
                        "id": target,
                        "terminals": payload,
                    })

            # ── Switch robot model (Track A1) ─────────────────────────────────
            elif mtype == "set_model":
                # Option A: host switches models on its OWN container, like a client.
                if True:
                    model_id = msg.get("model_id", "")
                    res = container_manager.switch_model(username, model_id)
                    if res.get("ok"):
                        # Re-center the new robot in the 3D view: clear the odom
                        # origin so it re-anchors on the new sim's first sample,
                        # and zero the held pose so the dot starts at (0,0).
                        env = hub.env_for(username)
                        env.odom_origin = None
                        env._was_teleoping = False
                        for k in ("x", "y", "yaw", "vx", "wz"):
                            env.state[k] = 0.0
                    await ws.send_json({
                        "type": "model_status",
                        "ok": res.get("ok", False),
                        "model_id": res.get("model_id", model_id),
                        "state": ("switching" if res.get("ok") else "error"),
                        "msg": res.get("error") or res.get("note", ""),
                    })
                    # Let the host monitor reflect the new model immediately.
                    if res.get("ok"):
                        await hub.broadcast_host({"type": "clients_update",
                                                  "clients": hub.client_list()})

            # ── Arm joint control (Track A-arm) ──────────────────────────────
            elif mtype == "joint_cmd":
                # positions are in the spec's kinematic order (radians). The
                # joint_state feedback loop reflects the resulting motion, so no
                # explicit ack is needed here.
                positions = msg.get("positions") or []
                try:
                    duration = float(msg.get("time", 0.4) or 0.4)
                except (TypeError, ValueError):
                    duration = 0.4
                await asyncio.get_event_loop().run_in_executor(
                    None, container_manager.publish_joints, username, positions, duration)

            # ── Ping ─────────────────────────────────────────────────────────
            elif mtype == "ping":
                await ws.send_json({"type":"pong"})

    except (WebSocketDisconnect, Exception):
        pass
    finally:
        client.alive = False
        # Cancel all terminal readers and kill their PTYs
        for t in list(client.terminals.values()):
            if t.reader:
                t.reader.cancel()
            if t.pid:
                try: os.kill(t.pid, signal.SIGKILL)
                except: pass
        hub.remove(client_id)
        # Stage 4: if this user has no remaining connections, let their Gazebo
        # idle-out (GC handles it after the grace period). We don't kill it
        # immediately so a quick refresh/reconnect reuses the running sim.
        await hub.broadcast_all({"type":"client_left","id":client_id})
        await hub.broadcast_host({"type":"clients_update","clients":hub.client_list()})

if __name__ == "__main__":
    print("🚀 ROS Ops Center starting on http://0.0.0.0:8000")
    print("   Login:      http://localhost:8000/login")
    print("   Sign up:    http://localhost:8000/signup")
    print("   Host panel: http://localhost:8000/host")
    print("   Client:     http://localhost:8000")
    # Stage 2: ensure every registered user has a Linux account in the container
    # (recreates any lost to a container rebuild).
    try:
        users = [u["username"] for u in auth.all_users()]
        if users:
            summary = linux_user.reconcile_users(users)
            if summary.get("ok"):
                if summary["created"]:
                    print(f"   Reconciled Linux users: created {summary['created']}")
                else:
                    print(f"   Linux users OK ({len(users)} registered)")
            else:
                print(f"   Linux user reconcile skipped: {summary.get('reason')}")
    except Exception as e:
        print(f"   Linux user reconcile warning: {e}")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level="warning")
