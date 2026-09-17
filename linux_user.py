"""
ROS Ops Center — Linux user management (Stage 2)
Creates real Linux users inside the ros_desktop container so each client gets
true filesystem isolation under /home/<username> and a real shell prompt.

All users share the one ROS install (their .bashrc sources it) — no reinstall.
"""
import subprocess, shlex, re

DOCKER_CONTAINER = "ros_desktop"
ROS_SETUP        = "/opt/ros/noetic/setup.bash"

# Must match auth.USERNAME_RE — defence in depth against shell injection.
_SAFE = re.compile(r"^[a-z][a-z0-9_]{2,31}$")


def _safe(username: str) -> bool:
    return bool(_SAFE.match(username or ""))


def _docker(args, timeout=15):
    """Run a docker command, return (rc, stdout, stderr). Never raises."""
    try:
        r = subprocess.run(
            ["docker"] + args, capture_output=True, text=True, timeout=timeout
        )
        return r.returncode, r.stdout, r.stderr
    except Exception as e:
        return 1, "", str(e)


def container_running() -> bool:
    rc, out, _ = _docker(
        ["inspect", "-f", "{{.State.Running}}", DOCKER_CONTAINER], timeout=5
    )
    return rc == 0 and out.strip() == "true"


def linux_user_exists(username: str) -> bool:
    if not _safe(username):
        return False
    rc, _, _ = _docker(["exec", DOCKER_CONTAINER, "id", "-u", username], timeout=8)
    return rc == 0


def create_linux_user(username: str) -> tuple:
    """Create /home/<username> as a real Linux user inside the container and seed
    its .bashrc to source ROS. Idempotent. Returns (ok, message)."""
    if not _safe(username):
        return False, "unsafe username"
    if not container_running():
        # Not fatal for signup — we reconcile on next server start / first login.
        return False, "container not running"
    if linux_user_exists(username):
        return True, "exists"

    # Create the user with a home dir and bash shell.
    rc, out, err = _docker(
        ["exec", DOCKER_CONTAINER, "useradd", "-m", "-s", "/bin/bash", username],
        timeout=15,
    )
    if rc != 0 and "already exists" not in (err + out):
        return False, f"useradd failed: {err.strip() or out.strip()}"

    # Seed .bashrc: source ROS + helpful defaults. Written via a heredoc so we
    # don't fight quoting. Appended only if not already present.
    bashrc = (
        "grep -q 'ros/noetic/setup.bash' /home/{u}/.bashrc 2>/dev/null || "
        "cat >> /home/{u}/.bashrc <<'EOF'\n"
        "# --- ROS Ops Center ---\n"
        "source /opt/ros/noetic/setup.bash 2>/dev/null\n"
        "export TURTLEBOT3_MODEL=burger\n"
        "export TERM=xterm-256color\n"
        "export CLICOLOR=1\n"
        "force_color_prompt=yes\n"
        "alias ls='ls --color=auto'\n"
        "alias grep='grep --color=auto'\n"
        "alias ll='ls -alF --color=auto'\n"
        "PS1='\\[\\e[1;38;2;38;54;137m\\]\\u@\\h\\[\\e[00m\\]:\\[\\e[01;34m\\]\\w\\[\\e[00m\\]$ '\n"
        "cd ~\n"
        "EOF\n"
        "chown {u}:{u} /home/{u}/.bashrc"
    ).format(u=username)
    _docker(["exec", DOCKER_CONTAINER, "bash", "-lc", bashrc], timeout=15)
    return True, "created"


def ensure_color_bashrc(username: str):
    """Add color/prompt settings to an EXISTING user's .bashrc if missing.
    Idempotent — guarded by a marker line."""
    if not _safe(username) or not container_running():
        return
    block = (
        "grep -q 'ROS Ops color' /home/{u}/.bashrc 2>/dev/null || "
        "cat >> /home/{u}/.bashrc <<'EOF'\n"
        "# --- ROS Ops color ---\n"
        "export TERM=xterm-256color\n"
        "export CLICOLOR=1\n"
        "alias ls='ls --color=auto'\n"
        "alias grep='grep --color=auto'\n"
        "alias ll='ls -alF --color=auto'\n"
        "PS1='\\[\\e[1;38;2;38;54;137m\\]\\u@\\h\\[\\e[00m\\]:\\[\\e[01;34m\\]\\w\\[\\e[00m\\]$ '\n"
        "EOF\n"
        "chown {u}:{u} /home/{u}/.bashrc"
    ).format(u=username)
    _docker(["exec", DOCKER_CONTAINER, "bash", "-lc", block], timeout=15)

    return True, "created"


def reconcile_users(usernames):
    """On server startup, ensure every DB user has a Linux account in the
    container (recreates any lost to a container rebuild). Returns a summary."""
    if not container_running():
        return {"ok": False, "reason": "container not running", "created": []}
    created = []
    for u in usernames:
        if not _safe(u):
            continue
        if not linux_user_exists(u):
            res = create_linux_user(u)
            ok = bool(res[0]) if isinstance(res, (tuple, list)) and res else False
            if ok:
                created.append(u)
        # Ensure colors/prompt for every user (existing ones too).
        try:
            ensure_color_bashrc(u)
        except Exception:
            pass
    return {"ok": True, "created": created}


# ── Container filesystem ops, scoped to /home/<username> ──────────────────────
import os as _os, base64 as _b64, json as _json, posixpath as _pp

def _home(username: str) -> str:
    return "/home/" + username

def _resolve_in_home(username: str, path: str) -> str:
    """Resolve a requested path to an absolute path INSIDE the user's home.
    Anything that would escape the home is clamped back to the home root."""
    home = _home(username)
    if not path or path in ("~", "", "."):
        return home
    p = path
    if p.startswith("~"):
        p = home + p[1:]
    if not p.startswith("/"):
        p = _pp.join(home, p)
    # Normalise and jail: the final path must stay within home.
    p = _pp.normpath(p)
    if p != home and not p.startswith(home + "/"):
        return home
    return p

def list_dir(username: str, path: str):
    """List a directory inside the user's container home. Returns a dict like
    the old Mac file API: {path, parent, items[]}."""
    if not _safe(username) or not container_running():
        return {"path": _home(username), "parent": _home(username), "items": []}
    target = _resolve_in_home(username, path)
    # Use a tiny python one-liner in the container to emit JSON listing.
    script = (
        "import os,json,sys\n"
        "d=sys.argv[1]\n"
        "home=sys.argv[2]\n"
        "out={'path':d,'parent':(os.path.dirname(d) if d!=home else home),'items':[]}\n"
        "try:\n"
        " for n in sorted(os.listdir(d)):\n"
        "  fp=os.path.join(d,n)\n"
        "  try:\n"
        "   st=os.stat(fp); isdir=os.path.isdir(fp)\n"
        "   ext=os.path.splitext(n)[1].lower()\n"
        "   out['items'].append({'name':n,'path':fp,'type':'dir' if isdir else 'file','size':st.st_size,'mtime':st.st_mtime,'ext':ext})\n"
        "  except: pass\n"
        " out['items'].sort(key=lambda x:(x['type']!='dir', x['name'].lower()))\n"
        "except Exception as e:\n"
        " out['error']=str(e)\n"
        "print(json.dumps(out))\n"
    )
    rc, sout, serr = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "python3", "-c", script, target, _home(username)],
        timeout=15,
    )
    if rc != 0:
        return {"path": target, "parent": _home(username), "items": [], "error": serr.strip()}
    try:
        return _json.loads(sout.strip().splitlines()[-1])
    except Exception:
        return {"path": target, "parent": _home(username), "items": []}

def read_file(username: str, path: str, max_bytes: int = 500_000):
    if not _safe(username) or not container_running():
        return None, "unavailable"
    target = _resolve_in_home(username, path)
    rc, sout, serr = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "bash", "-lc",
         f"if [ -f {shlex.quote(target)} ]; then wc -c < {shlex.quote(target)}; else echo MISSING; fi"],
        timeout=10,
    )
    if rc != 0 or sout.strip() == "MISSING":
        return None, "not found"
    try:
        if int(sout.strip()) > max_bytes:
            return None, "File too large (>500KB)"
    except ValueError:
        pass
    rc, sout, serr = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "base64", target], timeout=15,
    )
    if rc != 0:
        return None, serr.strip() or "read error"
    try:
        content = _b64.b64decode(sout).decode("utf-8", errors="replace")
        return content, None
    except Exception as e:
        return None, str(e)

def write_file(username: str, path: str, content: str):
    """Write text to a file inside the user's home (creating parent dirs)."""
    if not _safe(username):
        return False, "unsafe username"
    if not container_running():
        return False, "container not running"
    target = _resolve_in_home(username, path)
    if target == _home(username):
        return False, "invalid path"
    b64 = _b64.b64encode(content.encode("utf-8")).decode("ascii")
    cmd = (
        f"mkdir -p {shlex.quote(_pp.dirname(target))} && "
        f"echo {shlex.quote(b64)} | base64 -d > {shlex.quote(target)}"
    )
    rc, sout, serr = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "bash", "-lc", cmd], timeout=15,
    )
    if rc != 0:
        return False, serr.strip() or "write failed"
    return True, target

def write_file_bytes(username: str, path: str, data: bytes):
    if not _safe(username) or not container_running():
        return False, "unavailable"
    target = _resolve_in_home(username, path)
    if target == _home(username):
        return False, "invalid path"
    b64 = _b64.b64encode(data).decode("ascii")
    cmd = (
        f"mkdir -p {shlex.quote(_pp.dirname(target))} && "
        f"echo {shlex.quote(b64)} | base64 -d > {shlex.quote(target)}"
    )
    rc, _, serr = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "bash", "-lc", cmd], timeout=30,
    )
    return (rc == 0), (target if rc == 0 else (serr.strip() or "write failed"))

def read_file_bytes(username: str, path: str):
    if not _safe(username) or not container_running():
        return None
    target = _resolve_in_home(username, path)
    rc, sout, _ = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "base64", target], timeout=30,
    )
    if rc != 0:
        return None
    try:
        return _b64.b64decode(sout)
    except Exception:
        return None

def delete_path(username: str, path: str):
    """Delete a file or directory inside the user's home. Jailed to the home;
    refuses to delete the home root itself. Returns (ok, message)."""
    if not _safe(username) or not container_running():
        return False, "unavailable"
    target = _resolve_in_home(username, path)
    if target == _home(username):
        return False, "cannot delete home root"
    rc, _, serr = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "rm", "-rf", target], timeout=20,
    )
    return (rc == 0), (target if rc == 0 else (serr.strip() or "delete failed"))


# ── Quota (soft, app-enforced) ────────────────────────────────────────────────
QUOTA_BYTES = 1024 * 1024 * 1024  # 1 GB per user

def home_usage_bytes(username: str) -> int:
    """Return disk usage of /home/<username> in bytes (0 on any error)."""
    if not _safe(username) or not container_running():
        return 0
    rc, sout, _ = _docker(
        ["exec", DOCKER_CONTAINER, "du", "-sb", _home(username)], timeout=20,
    )
    if rc != 0:
        return 0
    try:
        return int(sout.split()[0])
    except (ValueError, IndexError):
        return 0

def quota_info(username: str) -> dict:
    used = home_usage_bytes(username)
    return {"used": used, "limit": QUOTA_BYTES,
            "over": used >= QUOTA_BYTES,
            "pct": min(100, round(used * 100 / QUOTA_BYTES)) if QUOTA_BYTES else 0}

def would_exceed(username: str, incoming_bytes: int) -> bool:
    """True if writing incoming_bytes more would push the user over quota."""
    return (home_usage_bytes(username) + max(0, incoming_bytes)) > QUOTA_BYTES


# ── Account deletion (remove Linux user + home) ──────────────────────────────
def delete_linux_user(username: str) -> tuple:
    """Delete the Linux user AND their home directory inside the container.
    Returns (ok, message). Safe no-op if the user/container is absent."""
    if not _safe(username):
        return False, "unsafe username"
    if not container_running():
        return False, "container not running"
    if not linux_user_exists(username):
        return True, "already gone"
    # Kill any lingering processes for the user, then userdel -r (removes home).
    _docker(["exec", DOCKER_CONTAINER, "pkill", "-9", "-u", username], timeout=10)
    rc, out, err = _docker(
        ["exec", DOCKER_CONTAINER, "userdel", "-r", username], timeout=20,
    )
    if rc != 0 and "does not exist" not in (err + out):
        # Fall back to removing the home dir even if userdel partially failed.
        _docker(["exec", DOCKER_CONTAINER, "rm", "-rf", _home(username)], timeout=20)
        return True, f"userdel warning: {err.strip() or out.strip()}"
    return True, "deleted"


# ── Soft disk quota (app-enforced) ────────────────────────────────────────────
QUOTA_BYTES = 1024 * 1024 * 1024  # 1 GB per user

def usage_bytes(username: str) -> int:
    """Current disk usage of the user's home, in bytes (via du). 0 on failure."""
    if not _safe(username) or not container_running():
        return 0
    rc, sout, _ = _docker(
        ["exec", "-u", username, DOCKER_CONTAINER, "bash", "-lc",
         f"du -sb {shlex.quote(_home(username))} 2>/dev/null | cut -f1"],
        timeout=15,
    )
    if rc != 0:
        return 0
    try:
        return int(sout.strip().split()[0])
    except (ValueError, IndexError):
        return 0

def quota_info(username: str) -> dict:
    used = usage_bytes(username)
    return {
        "used": used,
        "limit": QUOTA_BYTES,
        "percent": round(min(100.0, used / QUOTA_BYTES * 100), 1) if QUOTA_BYTES else 0,
        "over": used >= QUOTA_BYTES,
    }

def has_room(username: str, incoming_bytes: int = 0) -> bool:
    """True if the user is under quota (optionally accounting for an incoming write)."""
    return (usage_bytes(username) + max(0, incoming_bytes)) <= QUOTA_BYTES
