"""
gazebo_manager.py — per-user isolated Gazebo simulators.

Each user gets their OWN roscore + headless Gazebo + turtlebot, running inside
the shared `ros_desktop` container but on dedicated ports, so they never see or
collide with each other's robots. Both the user's Python (rospy) and their
Robot API drive THEIR master; their /odom is read back to draw the path.

Design constraints honored:
  * One Gazebo per user, reused across reconnects (never double-spawn).
  * Lazy spawn + idle teardown (caller decides when to call ensure/stop).
  * All ROS lives in the container; we orchestrate via `docker exec`.
  * Ports are container-internal, so they don't touch the Mac host.

Port scheme (container-internal):
  user slot N (0..MAX-1):
    ROS master      : 11320 + N
    Gazebo master   : 11350 + N
"""

import subprocess
import threading
import time
import json
import shlex

DOCKER_CONTAINER = "ros_desktop"
ROS_SETUP        = "/opt/ros/noetic/setup.bash"

ROS_PORT_BASE    = 11320
GZ_PORT_BASE     = 11350
MAX_SLOTS        = 4          # hard cap on concurrent per-user sims
SPAWN_TIMEOUT    = 45.0       # seconds to wait for /odom to appear
TURTLEBOT_MODEL  = "burger"
LAUNCH = "turtlebot3_gazebo turtlebot3_empty_world.launch gui:=false"


def _docker_bg(inner_bash: str):
    """Run a bash command inside the container, detached (returns immediately)."""
    return subprocess.Popen(
        ["docker", "exec", DOCKER_CONTAINER, "bash", "-lc", inner_bash],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )


def _docker_run(inner_bash: str, timeout: float = 15.0):
    """Run a bash command inside the container, wait, return (rc, out, err)."""
    try:
        p = subprocess.run(
            ["docker", "exec", DOCKER_CONTAINER, "bash", "-lc", inner_bash],
            capture_output=True, text=True, timeout=timeout,
        )
        return p.returncode, p.stdout, p.stderr
    except Exception as e:
        return 1, "", str(e)


def container_running() -> bool:
    try:
        r = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", DOCKER_CONTAINER],
            capture_output=True, text=True, timeout=3,
        )
        return r.stdout.strip() == "true"
    except Exception:
        return False


class Sim:
    """One user's simulator: ports, env prefix, lifecycle state."""
    def __init__(self, username: str, slot: int):
        self.username = username
        self.slot = slot
        self.ros_port = ROS_PORT_BASE + slot
        self.gz_port  = GZ_PORT_BASE + slot
        self.state = "starting"     # starting | ready | error | stopped
        self.error = None
        self.started_at = time.monotonic()
        self.last_seen = time.monotonic()   # updated on activity; used for idle GC
        # cmd_vel publisher process (long-lived, reads JSON from stdin)
        self._pub = None
        self._pub_lock = threading.Lock()
        # the detached docker exec holding roscore+gazebo for this sim
        self._proc = None
        # persistent /odom subscriber + its latest cached pose
        self._odom_proc = None
        self._odom_latest = None

    @property
    def env_prefix(self) -> str:
        return (f"source {ROS_SETUP} 2>/dev/null; "
                f"export ROS_MASTER_URI=http://localhost:{self.ros_port}; "
                f"export GAZEBO_MASTER_URI=http://localhost:{self.gz_port}; "
                f"export TURTLEBOT3_MODEL={TURTLEBOT_MODEL}; ")

    def env_dict(self) -> dict:
        """Env vars a client's Python/shell should use to reach THIS sim."""
        return {
            "ROS_MASTER_URI": f"http://localhost:{self.ros_port}",
            "GAZEBO_MASTER_URI": f"http://localhost:{self.gz_port}",
            "TURTLEBOT3_MODEL": TURTLEBOT_MODEL,
        }


class GazeboManager:
    def __init__(self):
        self.sims = {}                 # username -> Sim
        self._slots = {}               # slot -> username
        self._lock = threading.Lock()

    # ── slot allocation ──────────────────────────────────────────────────────
    def _alloc_slot(self, username: str):
        for n in range(MAX_SLOTS):
            if n not in self._slots:
                self._slots[n] = username
                return n
        return None

    def _free_slot(self, slot: int):
        self._slots.pop(slot, None)

    # ── public API ───────────────────────────────────────────────────────────
    def status(self, username: str) -> dict:
        s = self.sims.get(username)
        if not s:
            return {"state": "none"}
        return {"state": s.state, "slot": s.slot, "ros_port": s.ros_port,
                "error": s.error}

    def env_for(self, username: str):
        s = self.sims.get(username)
        return s.env_dict() if s and s.state == "ready" else None

    def touch(self, username: str):
        s = self.sims.get(username)
        if s:
            s.last_seen = time.monotonic()

    def ensure(self, username: str) -> dict:
        """Ensure this user has a sim. Idempotent: if one exists (starting or
        ready) we reuse it. Spawns in a background thread; returns immediately
        with current status. Caller polls status()/env_for()."""
        if not container_running():
            return {"state": "error", "error": "container not running"}
        with self._lock:
            existing = self.sims.get(username)
            if existing and existing.state in ("starting", "ready"):
                existing.last_seen = time.monotonic()
                return self.status(username)
            slot = self._alloc_slot(username)
            if slot is None:
                return {"state": "error",
                        "error": f"no free sim slots (max {MAX_SLOTS})"}
            sim = Sim(username, slot)
            self.sims[username] = sim
        # launch in background so we don't block the websocket
        threading.Thread(target=self._spawn, args=(sim,), daemon=True).start()
        return self.status(username)

    def stop(self, username: str):
        with self._lock:
            sim = self.sims.pop(username, None)
            if not sim:
                return
            self._free_slot(sim.slot)
        # kill the publisher
        try:
            if sim._pub and sim._pub.poll() is None:
                sim._pub.terminate()
        except Exception:
            pass
        # kill the detached launch exec
        try:
            if sim._proc and sim._proc.poll() is None:
                sim._proc.terminate()
        except Exception:
            pass
        # kill the odom reader
        try:
            if sim._odom_proc and sim._odom_proc.poll() is None:
                sim._odom_proc.terminate()
        except Exception:
            pass
        # kill everything bound to this sim's ports inside the container
        kill = (
            f"pkill -f 'roscore -p {sim.ros_port}' 2>/dev/null; "
            f"pkill -f 'rosmaster.*{sim.ros_port}' 2>/dev/null; "
            f"pkill -f 'gazebo_{sim.slot}.log' 2>/dev/null; "
            f"pkill -f 'localhost:{sim.gz_port}' 2>/dev/null; "
            f"pkill -f 'localhost:{sim.ros_port}' 2>/dev/null; "
            "true"
        )
        _docker_run(kill, timeout=10)
        sim.state = "stopped"

    def stop_all(self):
        for u in list(self.sims.keys()):
            self.stop(u)

    def gc_idle(self, idle_secs: float = 90.0):
        """Stop sims whose last_seen is older than idle_secs. Call periodically."""
        now = time.monotonic()
        for u, s in list(self.sims.items()):
            if now - s.last_seen > idle_secs:
                self.stop(u)

    # ── spawn (runs in background thread) ─────────────────────────────────────
    def _spawn(self, sim: Sim):
        # Launch roscore AND gazebo inside ONE detached docker exec, so they
        # share a single process tree that stays alive. Source ROS explicitly
        # and log diagnostics so failures are visible in /tmp/gazebo_<slot>.log.
        log = f"/tmp/gazebo_{sim.slot}.log"
        rlog = f"/tmp/roscore_{sim.slot}.log"
        launch = (
            f"source {ROS_SETUP}; "
            f"export ROS_MASTER_URI=http://localhost:{sim.ros_port}; "
            f"export GAZEBO_MASTER_URI=http://localhost:{sim.gz_port}; "
            f"export TURTLEBOT3_MODEL={TURTLEBOT_MODEL}; "
            # Cap Gazebo's console log: under emulation it can grow to tens of GB over
            # a long-running host session and fill the disk (it crashed Docker once).
            # Pin this run's log file to /dev/null so it can never grow.
            # Clear any stale logs first, then pin BOTH the per-sim port dir AND
            # Gazebo's default master dir (11345) to /dev/null. gzserver often
            # picks the default 11345 regardless of GAZEBO_MASTER_URI, and it can
            # recreate its server-<port> dir on startup (destroying an early
            # symlink), so we also re-apply the cap AFTER launch via a watcher.
            f"rm -rf ~/.gazebo/server-* ~/.gazebo/log 2>/dev/null; "
            f"for p in {sim.gz_port} 11345; do "
            f"mkdir -p ~/.gazebo/server-$p; "
            f"ln -sf /dev/null ~/.gazebo/server-$p/default.log 2>/dev/null; "
            f"done; "
            # Watcher: every 60s re-pin any default.log that gzserver recreated
            # as a real file, so the cap survives Gazebo rebuilding its dir.
            f"(while true; do sleep 60; "
            f"for d in ~/.gazebo/server-*; do "
            f"[ -f \"$d/default.log\" ] && [ ! -L \"$d/default.log\" ] && "
            f"ln -sf /dev/null \"$d/default.log\" 2>/dev/null; "
            f"done; done) & "
            f"echo \"PATH=$PATH\" > {log}; "
            f"echo \"roslaunch=$(which roslaunch)\" >> {log}; "
            f"echo \"roscore=$(which roscore)\" >> {log}; "
            f"roscore -p {sim.ros_port} > {rlog} 2>&1 & "
            f"sleep 5; "
            f"roslaunch {LAUNCH} >> {log} 2>&1 & "
            f"wait"
        )
        sim._proc = subprocess.Popen(
            ["docker", "exec", DOCKER_CONTAINER, "bash", "-lc", launch],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        # Wait until /odom is actually present (robot spawned & publishing).
        deadline = time.monotonic() + SPAWN_TIMEOUT
        while time.monotonic() < deadline:
            rc, out, _ = _docker_run(
                f"source {ROS_SETUP}; "
                f"export ROS_MASTER_URI=http://localhost:{sim.ros_port}; "
                f"rostopic list 2>/dev/null", timeout=8
            )
            if rc == 0 and "/odom" in out and "/cmd_vel" in out:
                sim.state = "ready"
                sim.last_seen = time.monotonic()
                return
            time.sleep(2.0)
        sim.state = "error"
        sim.error = "sim did not become ready in time"

    # ── cmd_vel publishing (Robot API path) ──────────────────────────────────
    def _ensure_pub(self, sim: Sim):
        """Start a long-lived rospy publisher for this sim that reads JSON
        {vx,wz} lines from stdin (same pattern as the host CmdVelPublisher)."""
        if sim._pub and sim._pub.poll() is None:
            return sim._pub
        script = r"""
import json, sys, rospy
from geometry_msgs.msg import Twist
rospy.init_node('webui_cmdvel', anonymous=True, disable_signals=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rospy.sleep(0.3)
for line in sys.stdin:
    try:
        d = json.loads(line)
        m = Twist()
        m.linear.x  = float(d.get('vx', 0.0) or 0.0)
        m.angular.z = float(d.get('wz', 0.0) or 0.0)
        pub.publish(m)
    except Exception:
        pass
"""
        cmd = ["docker", "exec", "-i", DOCKER_CONTAINER, "bash", "-lc",
               f"source {ROS_SETUP} 2>/dev/null; "
               f"export ROS_MASTER_URI=http://localhost:{sim.ros_port}; "
               f"python3 -u -c {shlex.quote(script)}"]
        sim._pub = subprocess.Popen(
            cmd, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        return sim._pub

    def publish(self, username: str, vx: float, wz: float) -> bool:
        sim = self.sims.get(username)
        if not sim or sim.state != "ready":
            return False
        with sim._pub_lock:
            try:
                p = self._ensure_pub(sim)
                p.stdin.write(json.dumps({"vx": vx, "wz": wz}) + "\n")
                p.stdin.flush()
                sim.last_seen = time.monotonic()
                return True
            except Exception:
                sim._pub = None
                return False

    # ── odom reading (path drawing) ───────────────────────────────────────────
    def _ensure_odom_reader(self, sim: Sim):
        """Start a long-lived odom subscriber that prints the latest pose as a
        JSON line whenever it updates. Far cheaper than spawning rospy each poll."""
        if sim._odom_proc and sim._odom_proc.poll() is None:
            return sim._odom_proc
        reader = r"""
import rospy, math, json, sys
from nav_msgs.msg import Odometry
rospy.init_node('webui_odom_read', anonymous=True, disable_signals=True)
def cb(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
    sys.stdout.write(json.dumps({'x': p.x, 'y': p.y, 'yaw': yaw})+'\n')
    sys.stdout.flush()
rospy.Subscriber('/odom', Odometry, cb, queue_size=1)
rospy.spin()
"""
        cmd = ["docker", "exec", "-i", DOCKER_CONTAINER, "bash", "-lc",
               f"source {ROS_SETUP} 2>/dev/null; "
               f"export ROS_MASTER_URI=http://localhost:{sim.ros_port}; "
               f"python3 -u -c {shlex.quote(reader)}"]
        sim._odom_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        # Background thread drains stdout, keeping only the latest pose.
        def _drain(p, s):
            try:
                for line in p.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        s._odom_latest = json.loads(line)
                    except Exception:
                        pass
            except Exception:
                pass
        threading.Thread(target=_drain, args=(sim._odom_proc, sim), daemon=True).start()
        return sim._odom_proc

    def read_odom(self, username: str):
        """Return the latest {x,y,yaw} from this user's /odom, or None.
        Cheap: just returns the last value cached by the persistent reader."""
        sim = self.sims.get(username)
        if not sim or sim.state != "ready":
            return None
        try:
            self._ensure_odom_reader(sim)
        except Exception:
            return None
        d = sim._odom_latest
        if d and "x" in d:
            sim.last_seen = time.monotonic()
            return dict(d)
        return None


# Singleton
gazebo_manager = GazeboManager()
