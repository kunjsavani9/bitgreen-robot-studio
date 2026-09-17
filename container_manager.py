"""
container_manager.py — Option A: one full Docker container per client.

Each client gets their OWN container (ros_client_<username>) started from the
ros_noetic_ready image, each running its own roscore + headless Gazebo +
turtlebot. Total OS-level isolation — separate ROS install context, separate
everything. Heavier than the per-port approach but fully isolated.

Lifecycle:
  ensure(username)  -> create+start container, launch Gazebo (background), reuse if exists
  publish(username) -> drive that client's robot via their container's /cmd_vel
  read_odom(...)    -> latest pose from that client's /odom (persistent reader)
  run_env(username) -> the `docker exec` target for running their Python
  stop(username)    -> stop+remove their container, free resources
  gc_idle(...)      -> stop containers unused beyond a grace period

Each container is its own ROS world on the DEFAULT master (port 11311 INSIDE
the container — never published to the host, so no collisions). We talk to each
via `docker exec <container> ...`.
"""

import subprocess
import threading
import time
import json
import shlex
import base64
import math

import models  # Track A1: Model Spec catalog

IMAGE            = "ros_noetic_ready:latest"
ROS_SETUP        = "/opt/ros/noetic/setup.bash"
NAME_PREFIX      = "ros_client_"
MAX_CONTAINERS   = 3          # hard cap on concurrent client containers
SPAWN_TIMEOUT    = 75.0       # seconds to wait for /odom (fresh container is slower)
SHM_SIZE         = "1g"
TURTLEBOT_MODEL  = "burger"
LAUNCH = "turtlebot3_gazebo turtlebot3_empty_world.launch gui:=false"
# Unique token embedded in each sim supervisor's command line so a model switch
# can kill the whole supervisor loop by pattern (not just the roslaunch it runs).
SIM_SUPERVISOR_TAG = "WEBUI_SIM_SUPERVISOR"


def _safe(username: str) -> bool:
    return bool(username) and username.replace("_", "").isalnum() and username[0].isalpha()


def container_name(username: str) -> str:
    return NAME_PREFIX + username


def _run(args, timeout=20):
    try:
        p = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout, p.stderr
    except Exception as e:
        return 1, "", str(e)


def _container_state(name: str) -> str:
    """Return 'running' | 'exited' | 'none'."""
    rc, out, _ = _run(
        ["docker", "inspect", "-f", "{{.State.Status}}", name], timeout=8
    )
    if rc != 0:
        return "none"
    return out.strip() or "none"


class ClientContainer:
    def __init__(self, username: str):
        self.username = username
        self.name = container_name(username)
        self.state = "starting"     # starting | ready | error | stopped
        self.error = None
        self.started_at = time.monotonic()
        self.last_seen = time.monotonic()
        # ── Model topic abstraction (Track A0) ────────────────────────────────
        # The control plumbing reads these instead of hardcoding /cmd_vel and
        # /odom. Defaults preserve today's behavior (burger). Later phases set
        # them from a Model Spec in _spawn()/switch_model() so different robots
        # (which use different topic names) plug into the same control loop.
        self.cmd_vel_topic = "/cmd_vel"
        self.odom_topic = "/odom"
        # ── Sensors / capabilities (Track B + C) ──────────────────────────────
        self.scan_topic = "/scan"
        self.capabilities = ("mobile_base",)
        self.has_lidar = False
        # ── Manipulator arm (Track A-arm) ─────────────────────────────────────
        # When the active model is an arm, these replace the mobile plumbing:
        # joints are driven by a JointTrajectory publisher and read from
        # /joint_states (mapped by name into kinematic order).
        self.is_arm = False
        self.joint_cmd_topic = ""
        self.joint_state_topic = "/joint_states"
        self.joint_names = ()              # kinematic order (base -> tip)
        self._joint_pub = None             # JointTrajectory publisher process
        self._joint_pub_lock = threading.Lock()
        self._joint_proc = None            # /joint_states reader process
        self._joint_latest = None          # dict: joint_name -> position (rad)
        self._joint_latest_ts = 0.0
        self._joint_target = None          # last commanded positions (kinematic order)
        # Which catalog model this container is running (Track A1).
        self.model_id = models.DEFAULT_MODEL_ID
        self._pub = None
        self._pub_lock = threading.Lock()
        self._odom_proc = None
        self._odom_latest = None
        self._scan_proc = None
        self._scan_latest = None
        self._scan_latest_ts = 0.0
        self._proc = None
        self._notified_ready = False
        # go-to-goal controller state
        self._goto_thread = None
        self._goto_cancel = threading.Event()
        self._goto_final = None
        # obstacle-detection auto-drive (emanual 10.2) state
        self._auto_thread = None
        self._auto_cancel = threading.Event()
        self._auto_on = False
        self._auto_blocked = False      # latched: obstacle within stop distance ahead
        self._auto_manual_until = 0.0   # auto loop yields to manual steering until this time
        # ROS Navigation stack (move_base) state
        self._nav_on = False
        self._nav_proc = None
        # currently running user script (PID inside the container) for STOP
        self._script_pid = None


class ContainerManager:
    def __init__(self):
        self.cs = {}            # username -> ClientContainer
        self._lock = threading.Lock()

    # ── status / env ─────────────────────────────────────────────────────────
    def status(self, username: str) -> dict:
        c = self.cs.get(username)
        if not c:
            return {"state": "none"}
        return {"state": c.state, "name": c.name, "error": c.error,
                "model_id": getattr(c, "model_id", models.DEFAULT_MODEL_ID),
                "is_arm": getattr(c, "is_arm", False),
                "joint_names": list(getattr(c, "joint_names", ()) or ())}

    def touch(self, username: str):
        c = self.cs.get(username)
        if c:
            c.last_seen = time.monotonic()

    def is_ready(self, username: str) -> bool:
        c = self.cs.get(username)
        return bool(c and c.state == "ready")

    # ── create (registration: container exists but stays STOPPED) ──────────────
    def create(self, username: str) -> dict:
        """Create the client's persistent container WITHOUT starting it. Called
        at registration. Cheap: no Gazebo runs until they log in. Idempotent."""
        if not _safe(username):
            return {"ok": False, "error": "unsafe username"}
        name = container_name(username)
        st = _container_state(name)
        if st in ("running", "exited"):
            return {"ok": True, "note": "already exists"}
        rc, _, err = _run(
            ["docker", "create", "--name", name,
             "--shm-size", SHM_SIZE,
             "--platform", "linux/amd64",
             "--entrypoint", "sleep",
             IMAGE, "infinity"],
            timeout=60,
        )
        if rc != 0:
            return {"ok": False, "error": err.strip()[:200]}
        return {"ok": True, "note": "created (stopped)"}

    # ── ensure (idempotent, background spawn) ──────────────────────────────────
    def ensure(self, username: str, model_id: str = None) -> dict:
        if not _safe(username):
            return {"state": "error", "error": "unsafe username"}
        with self._lock:
            existing = self.cs.get(username)
            if existing and existing.state in ("starting", "ready"):
                # Trust "starting" (spawn in progress). For "ready", double-check
                # the container is REALLY running — it may have been stopped by
                # GC, a crash, or a disconnect while our in-memory state is stale.
                if existing.state == "starting":
                    existing.last_seen = time.monotonic()
                    return self.status(username)
                really_running = (_container_state(existing.name) == "running")
                if really_running:
                    existing.last_seen = time.monotonic()
                    return self.status(username)
                # Stale: container is down. Drop it and re-spawn below.
                self.cs.pop(username, None)
            if len(self.cs) >= MAX_CONTAINERS and username not in self.cs:
                return {"state": "error",
                        "error": f"max {MAX_CONTAINERS} client containers running"}
            c = ClientContainer(username)
            if model_id and models.is_valid(model_id):
                c.model_id = model_id
            self.cs[username] = c
        threading.Thread(target=self._spawn, args=(c,), daemon=True).start()
        return self.status(username)

    # ── spawn ──────────────────────────────────────────────────────────────────
    def _spawn(self, c: ClientContainer):
        name = c.name
        # 1. Ensure a container exists and is running.
        st = _container_state(name)
        if st == "running":
            pass  # reuse
        elif st in ("created", "exited", "paused", "dead"):
            # Container exists (from create() at registration, or a prior
            # session). Start it — do NOT docker run (that collides on name).
            rc, _, err = _run(["docker", "start", name], timeout=40)
            if rc != 0:
                c.state = "error"
                c.error = "docker start failed: " + (err.strip()[:200])
                return
        else:
            # Truly absent -> create + start fresh.
            rc, _, err = _run(
                ["docker", "run", "-d", "--name", name,
                 "--shm-size", SHM_SIZE,
                 "--platform", "linux/amd64",
                 "--entrypoint", "sleep",
                 IMAGE, "infinity"],
                timeout=60,
            )
            if rc != 0:
                c.state = "error"
                c.error = "docker run failed: " + (err.strip()[:200])
                return
        time.sleep(2.0)

        # 2. Launch the model's roscore + Gazebo, then 3. wait until its topics
        #    appear. Both are spec-driven (Track A1) and shared with switch_model.
        self._launch_sim(c)
        self._wait_ready(c)

    # ── spec-driven sim launch / readiness (shared by _spawn + switch_model) ────
    def _launch_sim(self, c: ClientContainer):
        """(Re)launch roscore + the model's Gazebo inside the container, reading
        env / launch command / control topics from the Model Spec. Stores the
        model's cmd_vel/odom topics on the container so publish()/read_odom()
        (and the readiness check) target the right topics for this robot."""
        name = c.name
        spec = models.get_spec(c.model_id) or models.get_spec(models.DEFAULT_MODEL_ID)
        # Track A0 seams: control loop reads these.
        c.cmd_vel_topic = spec.cmd_vel_topic
        c.odom_topic = spec.odom_topic
        # Track B/C: sensors + capabilities for this model.
        c.scan_topic = spec.scan_topic
        c.capabilities = tuple(spec.capabilities)
        c.has_lidar = ("lidar" in spec.capabilities)
        # Track A-arm: manipulator plumbing (no-op for mobile robots). For arms
        # there is no generated world and no LIDAR, so the arena/LIDAR blocks
        # below are skipped and the spec's bringup launch runs as-is.
        c.is_arm = ("arm" in spec.capabilities)
        c.joint_cmd_topic = spec.joint_cmd_topic
        c.joint_state_topic = spec.joint_state_topic
        c.joint_names = tuple(spec.joint_names)
        # Per-spec env (e.g. TURTLEBOT3_MODEL=waffle), each value shell-quoted.
        env_exports = "".join(
            f"export {k}={shlex.quote(str(v))}; " for k, v in spec.env.items()
        )
        launch_args = spec.launch or LAUNCH
        # Generated world (the big arena): write a self-contained SDF world + a
        # launch file into the container so roslaunch can load them. Both are
        # built from the SAME geometry the dashboard overlays, and the robot is
        # spawned at the origin (cleared centre cell) so its odom frame coincides
        # with the world frame and the overlay lines up with no offset.
        if models.uses_generated_world(c.model_id):
            sdf = models.world_sdf(c.model_id) or ""
            lxml = models.world_launch_xml(c.model_id) or ""
            b64w = base64.b64encode(sdf.encode()).decode()
            b64l = base64.b64encode(lxml.encode()).decode()
            write_cmd = (
                f"mkdir -p {shlex.quote(models.ARENA_DIR)} && "
                f"echo {shlex.quote(b64w)} | base64 -d > {shlex.quote(models.ARENA_WORLD_PATH)} && "
                f"echo {shlex.quote(b64l)} | base64 -d > {shlex.quote(models.ARENA_LAUNCH_PATH)}"
            )
            _run(["docker", "exec", name, "bash", "-lc", write_cmd], timeout=20)
            launch_args = f"{models.ARENA_LAUNCH_PATH} gui:=false"
        # Size the simulated LIDAR for the scaled-up arena WITHOUT overloading the
        # emulator. The real TB3 LIDAR reaches 3.5 m at 360 beams; here we set the
        # range just far enough to see the pillar grid (~16.5 m) and drop the beam
        # count so each scan is cheap enough that the sensor keeps publishing under
        # the AMCL + move_base load (full 35 m range starved /scan). Both values
        # live only in the LIDAR ray sensor of these xacros, so the numeric-match
        # sed is safe and idempotent (re-spawns on a persisted container re-apply).
        if c.has_lidar:
            rng = models.LIDAR_MAX_RANGE
            nbeams = models.LIDAR_SAMPLES
            patch = (
                f"source {ROS_SETUP} 2>/dev/null; "
                f"D=$(rospack find turtlebot3_description 2>/dev/null)/urdf; "
                f"for f in turtlebot3_waffle.gazebo.xacro turtlebot3_waffle_pi.gazebo.xacro; do "
                f"[ -f \"$D/$f\" ] && sed -i -E "
                f"-e 's|<max>[0-9.]+</max>|<max>{rng}</max>|' "
                f"-e 's|<samples>[0-9]+</samples>|<samples>{nbeams}</samples>|' "
                f"\"$D/$f\"; "
                f"done"
            )
            _run(["docker", "exec", name, "bash", "-lc", patch], timeout=20)
        # Emulation hardening (amd64-on-arm64 under QEMU): force software GL and
        # skip the online model-DB fetch. Reliability: keep an existing roscore
        # (fast switch), wait for the master before roslaunch, crash-retry.
        #
        # IMPORTANT: every attempt FIRST kills any stray gzserver/gzclient by
        # PROCESS NAME (`pkill gzserver`, NOT `pkill -f gzserver`) so a retry can
        # never leave TWO robot models in one world (which makes the pose
        # ping-pong). Name matching is critical here: `-f` would match this very
        # supervisor's own command line (it contains the text "gzserver") and
        # kill the loop itself. The `: SIM_SUPERVISOR_TAG` no-op marks this
        # bash process so switch_model can kill the whole loop by pattern.
        launch = (
            f"source {ROS_SETUP}; "
            f"{env_exports}"
            f"export LIBGL_ALWAYS_SOFTWARE=1; "
            f"export GAZEBO_MODEL_DATABASE_URI=''; "
            # Gazebo writes a per-run console log under ~/.gazebo/server-<port>/ that,
            # under emulation, can balloon to tens of GB and fill the host disk (it
            # crashed Docker once). Clear stale logs, then pin this run's log file to
            # /dev/null (client uses Gazebo's default master 11345) so it can't grow.
            f"rm -rf ~/.gazebo/server-* ~/.gazebo/log 2>/dev/null; "
            f"mkdir -p ~/.gazebo/server-11345 && ln -sf /dev/null ~/.gazebo/server-11345/default.log 2>/dev/null; "
            # gzserver can recreate its server-<port> dir on startup, replacing the
            # symlink above with a real (growable) file. A background watcher re-pins
            # any recreated default.log to /dev/null every 60s so it can never grow.
            f"(while true; do sleep 60; "
            f"for d in ~/.gazebo/server-*; do "
            f"[ -f \"$d/default.log\" ] && [ ! -L \"$d/default.log\" ] && "
            f"ln -sf /dev/null \"$d/default.log\" 2>/dev/null; "
            f"done; done) & "
            f": {SIM_SUPERVISOR_TAG}; "
            f"rosnode list >/dev/null 2>&1 || (roscore >/tmp/roscore.log 2>&1 &); "
            f"for i in $(seq 1 30); do rosnode list >/dev/null 2>&1 && break; sleep 1; done; "
            f"for t in 1 2 3; do "
            f"pkill gzserver 2>/dev/null; pkill gzclient 2>/dev/null; sleep 1; "
            f"roslaunch {launch_args} >>/tmp/gazebo.log 2>&1; "
            f"echo \"[supervisor] roslaunch exited (attempt $t), cleaning + retrying\" >>/tmp/gazebo.log; "
            f"sleep 2; "
            f"done & "
            f"wait"
        )
        c._proc = subprocess.Popen(
            ["docker", "exec", name, "bash", "-lc", launch],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )

    def _wait_ready(self, c: ClientContainer):
        """Block until the model's cmd_vel + odom topics are present, then mark
        the container ready (or error on timeout)."""
        name = c.name
        deadline = time.monotonic() + SPAWN_TIMEOUT
        while time.monotonic() < deadline:
            rc, out, _ = _run(
                ["docker", "exec", name, "bash", "-lc",
                 f"source {ROS_SETUP}; rostopic list 2>/dev/null"],
                timeout=8,
            )
            if rc == 0 and (
                (c.is_arm and c.joint_state_topic in out and c.joint_cmd_topic in out)
                or (not c.is_arm and c.odom_topic in out and c.cmd_vel_topic in out)
            ):
                c.state = "ready"
                c.last_seen = time.monotonic()
                return
            time.sleep(2.0)
        c.state = "error"
        c.error = "sim did not become ready in time"

    # ── switch the loaded robot model (Track A1) ───────────────────────────────
    def switch_model(self, username: str, model_id: str) -> dict:
        """Switch the client's robot to a different catalog model. The container
        stays up; only the Gazebo sim is torn down and relaunched (roscore is
        kept for a faster switch). Heavy work runs in a background thread; the
        sim becomes 'ready' again when the new model's topics appear."""
        if not models.is_valid(model_id):
            return {"ok": False, "error": "unknown model id"}
        c = self.cs.get(username)
        # Not up yet (or stale) — just spawn fresh on the requested model.
        if not c or _container_state(c.name) != "running":
            self.ensure(username, model_id=model_id)
            return {"ok": True, "note": "spawning", "model_id": model_id}
        if c.model_id == model_id and c.state == "ready":
            return {"ok": True, "note": "already loaded", "model_id": model_id}
        c.model_id = model_id
        c.state = "starting"
        c._notified_ready = False          # so server re-announces "ready"
        threading.Thread(target=self._do_switch, args=(c,), daemon=True).start()
        return {"ok": True, "note": "switching", "model_id": model_id}

    def _do_switch(self, c: ClientContainer):
        # 1. Cancel any active go-to and stop motion on the OLD robot.
        try:
            c._goto_cancel.set()
        except Exception:
            pass
        try:
            c._auto_cancel.set()
            c._auto_on = False
        except Exception:
            pass
        try:
            if getattr(c, "_nav_on", False):
                self.stop_navigation(c.username)
        except Exception:
            pass
        try:
            self.publish(c.username, 0.0, 0.0)
        except Exception:
            pass
        # 2. Drop helper processes bound to the old topics (they re-spawn lazily
        #    on the new topics via publish()/read_odom()).
        for attr in ("_pub", "_odom_proc", "_scan_proc", "_joint_pub", "_joint_proc"):
            p = getattr(c, attr, None)
            try:
                if p and p.poll() is None:
                    p.terminate()
            except Exception:
                pass
            setattr(c, attr, None)
        c._odom_latest = None
        c._scan_latest = None
        c._joint_latest = None
        # 3. Tear down the old sim. Kill the old supervisor LOOP first (by its
        #    unique marker) so it can't relaunch the old robot when we kill
        #    gzserver — then kill gazebo / roslaunch / the spawners. roscore is
        #    kept for a fast switch. Terminating the host-side docker exec is NOT
        #    enough (it doesn't reap the in-container tree), hence the pattern
        #    kills inside the container.
        try:
            if c._proc and c._proc.poll() is None:
                c._proc.terminate()
        except Exception:
            pass
        # Kill the OLD supervisor loop FIRST (so it can't relaunch the old robot
        # when gazebo dies). Its marker is written SPLIT ('WEBUI_''SIM_…') so
        # this kill command's own argv doesn't self-match. Then kill the sim by
        # PROCESS NAME (a name match never hits our bash), and reap orphaned
        # helper readers/publishers left by earlier switches (split for the same
        # no-self-match reason). roscore is kept for a fast switch.
        _sa, _sb = SIM_SUPERVISOR_TAG[:6], SIM_SUPERVISOR_TAG[6:]
        _run(["docker", "exec", c.name, "bash", "-lc",
              f"pkill -9 -f '{_sa}''{_sb}' 2>/dev/null; "
              "pkill -9 gzserver 2>/dev/null; pkill -9 gzclient 2>/dev/null; "
              "pkill -9 -f 'webui_''odom_read' 2>/dev/null; "
              "pkill -9 -f 'webui_''scan_read' 2>/dev/null; "
              "pkill -9 -f 'webui_''cmdvel' 2>/dev/null; true"],
             timeout=15)
        time.sleep(2.0)
        # 4. Relaunch with the new spec + wait for its topics.
        self._launch_sim(c)
        self._wait_ready(c)

    # ── stop / gc ──────────────────────────────────────────────────────────────
    def stop(self, username: str, remove: bool = False):
        """Stop the client's container (frees RAM) but KEEP it by default so
        their files/setup persist between sessions. Pass remove=True only to
        fully delete (e.g. account deletion)."""
        with self._lock:
            c = self.cs.pop(username, None)
        # kill our helper processes regardless
        if c:
            for proc_attr in ("_pub", "_odom_proc", "_scan_proc", "_proc", "_nav_proc"):
                p = getattr(c, proc_attr, None)
                try:
                    if p and p.poll() is None:
                        p.terminate()
                except Exception:
                    pass
            c._nav_on = False
        name = c.name if c else container_name(username)
        _run(["docker", "stop", name], timeout=30)
        if remove:
            _run(["docker", "rm", name], timeout=20)
        if c:
            c.state = "stopped"

    def destroy(self, username: str):
        """Fully stop AND remove the client's container (account deletion)."""
        self.stop(username, remove=True)

    def stop_all(self):
        for u in list(self.cs.keys()):
            self.stop(u)   # persist; just frees RAM

    def gc_idle(self, idle_secs: float = 180.0):
        """Stop (not remove) containers unused beyond the grace period — frees
        RAM while keeping the client's data for next time."""
        now = time.monotonic()
        for u, c in list(self.cs.items()):
            if now - c.last_seen > idle_secs:
                self.stop(u)   # persist

    # ── run env (for client Python) ────────────────────────────────────────────
    def exec_prefix(self, username: str):
        """Returns the docker-exec command list prefix to run a command inside
        THIS client's container with ROS sourced, or None if not ready."""
        c = self.cs.get(username)
        if not c or c.state != "ready":
            return None
        return c.name

    # ── cmd_vel publishing (Robot API) ─────────────────────────────────────────
    def _ensure_pub(self, c: ClientContainer):
        if c._pub and c._pub.poll() is None:
            return c._pub
        script = r"""
import json, sys, rospy
from geometry_msgs.msg import Twist
rospy.init_node('webui_cmdvel', anonymous=True, disable_signals=True)
topic = sys.argv[1] if len(sys.argv) > 1 else '/cmd_vel'
pub = rospy.Publisher(topic, Twist, queue_size=1)
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
        topic = getattr(c, "cmd_vel_topic", "/cmd_vel") or "/cmd_vel"
        cmd = ["docker", "exec", "-i", c.name, "bash", "-lc",
               f"source {ROS_SETUP} 2>/dev/null; "
               f"python3 -u -c {shlex.quote(script)} {shlex.quote(topic)}"]
        c._pub = subprocess.Popen(
            cmd, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        return c._pub

    def publish(self, username: str, vx: float, wz: float) -> bool:
        c = self.cs.get(username)
        if not c or c.state != "ready":
            return False
        # Teleop safety guard: block FORWARD into a close obstacle (LIDAR models).
        # Reverse (vx<0) and turning (wz) are never blocked, so you can back out.
        # Fail-open: if there's no fresh scan, drive normally.
        if vx > 0.0 and getattr(c, "has_lidar", False):
            try:
                self._ensure_scan_reader(c)   # idempotent; keeps /scan flowing
                fresh = (time.time() - getattr(c, "_scan_latest_ts", 0.0)) < self.TELEOP_GUARD_STALE
                if fresh:
                    nearest = self._front_nearest(c, self.TELEOP_GUARD_FOV)
                    if nearest is not None:
                        if nearest < self.TELEOP_GUARD_DIST:
                            vx = 0.0          # too close ahead — hold forward
                        elif nearest < self.TELEOP_GUARD_SLOW:
                            # ramp forward speed down between SLOW and DIST so a fast
                            # approach doesn't coast past the stop line
                            span = self.TELEOP_GUARD_SLOW - self.TELEOP_GUARD_DIST
                            scale = (nearest - self.TELEOP_GUARD_DIST) / span
                            vx = vx * max(0.0, min(1.0, scale))
            except Exception:
                pass
        with c._pub_lock:
            try:
                p = self._ensure_pub(c)
                p.stdin.write(json.dumps({"vx": vx, "wz": wz}) + "\n")
                p.stdin.flush()
                c.last_seen = time.monotonic()
                return True
            except Exception:
                c._pub = None
                return False

    # ── odom reading (path drawing) ────────────────────────────────────────────
    def _ensure_odom_reader(self, c: ClientContainer):
        if c._odom_proc and c._odom_proc.poll() is None:
            return c._odom_proc
        reader = r"""
import rospy, math, json, sys
from nav_msgs.msg import Odometry
rospy.init_node('webui_odom_read', anonymous=True, disable_signals=True)
topic = sys.argv[1] if len(sys.argv) > 1 else '/odom'
def cb(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
    sys.stdout.write(json.dumps({'x': p.x, 'y': p.y, 'yaw': yaw})+'\n')
    sys.stdout.flush()
rospy.Subscriber(topic, Odometry, cb, queue_size=1)
rospy.spin()
"""
        topic = getattr(c, "odom_topic", "/odom") or "/odom"
        cmd = ["docker", "exec", "-i", c.name, "bash", "-lc",
               f"source {ROS_SETUP} 2>/dev/null; "
               f"python3 -u -c {shlex.quote(reader)} {shlex.quote(topic)}"]
        c._odom_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        def _drain(p, cc):
            try:
                for line in p.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        cc._odom_latest = json.loads(line)
                    except Exception:
                        pass
            except Exception:
                pass
        threading.Thread(target=_drain, args=(c._odom_proc, c), daemon=True).start()
        return c._odom_proc

    def read_odom(self, username: str):
        c = self.cs.get(username)
        if not c or c.state != "ready":
            return None
        try:
            fresh = (c._odom_proc is None or c._odom_proc.poll() is not None)
            self._ensure_odom_reader(c)
        except Exception:
            return None
        # If the reader was just (re)started, give it a moment to subscribe and
        # receive the first /odom message (rospy init takes a couple seconds).
        if fresh:
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline and not c._odom_latest:
                time.sleep(0.2)
        d = c._odom_latest
        if d and "x" in d:
            c.last_seen = time.monotonic()
            return dict(d)
        return None

    # ── Manipulator arm: joint command + joint-state reader (Track A-arm) ───────
    # Mirrors the cmd_vel publisher + odom reader, but speaks JointTrajectory and
    # JointState. Only used when the active model has the 'arm' capability.
    def _ensure_joint_pub(self, c: ClientContainer):
        if c._joint_pub and c._joint_pub.poll() is None:
            return c._joint_pub
        script = r"""
import json, sys, rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
rospy.init_node('webui_jointcmd', anonymous=True, disable_signals=True)
topic = sys.argv[1]
names = sys.argv[2].split(',')
pub = rospy.Publisher(topic, JointTrajectory, queue_size=1)
# A fresh publisher needs a moment to establish its link to the controller;
# ROS drops anything published before that handshake. Wait for the subscriber
# (up to 3s) so the FIRST command isn't lost. The command sits buffered in the
# stdin pipe meanwhile, so nothing is dropped on our side.
_t0 = rospy.get_time()
while pub.get_num_connections() < 1 and (rospy.get_time() - _t0) < 3.0:
    rospy.sleep(0.05)
for line in sys.stdin:
    try:
        d = json.loads(line)
        pos = [float(x) for x in d.get('positions', [])]
        if len(pos) != len(names):
            continue
        t = float(d.get('time', 0.4) or 0.4)
        m = JointTrajectory()
        m.joint_names = names
        p = JointTrajectoryPoint()
        p.positions = pos
        p.time_from_start = rospy.Duration(max(0.05, t))
        m.points = [p]
        pub.publish(m)
    except Exception:
        pass
"""
        names = ",".join(c.joint_names)
        cmd = ["docker", "exec", "-i", c.name, "bash", "-lc",
               f"source {ROS_SETUP} 2>/dev/null; "
               f"python3 -u -c {shlex.quote(script)} "
               f"{shlex.quote(c.joint_cmd_topic)} {shlex.quote(names)}"]
        c._joint_pub = subprocess.Popen(
            cmd, stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        return c._joint_pub

    def publish_joints(self, username: str, positions, duration: float = 0.4) -> bool:
        """Stream a single-point JointTrajectory to the arm controller. positions
        are in the spec's KINEMATIC order (base -> tip)."""
        c = self.cs.get(username)
        if not c or c.state != "ready" or not c.is_arm:
            return False
        try:
            positions = [float(x) for x in positions]
        except Exception:
            return False
        if len(positions) != len(c.joint_names):
            return False
        with c._joint_pub_lock:
            try:
                p = self._ensure_joint_pub(c)
                p.stdin.write(json.dumps({"positions": positions, "time": duration}) + "\n")
                p.stdin.flush()
                c._joint_target = positions
                c.last_seen = time.monotonic()
                return True
            except Exception:
                c._joint_pub = None
                return False

    def _ensure_joint_reader(self, c: ClientContainer):
        if c._joint_proc and c._joint_proc.poll() is None:
            return c._joint_proc
        reader = r"""
import rospy, json, sys
from sensor_msgs.msg import JointState
rospy.init_node('webui_jointread', anonymous=True, disable_signals=True)
topic = sys.argv[1] if len(sys.argv) > 1 else '/joint_states'
last = [0.0]
def cb(m):
    now = rospy.get_time()
    if now - last[0] < 0.05:      # ~20 Hz cap
        return
    last[0] = now
    d = {n: round(float(p), 5) for n, p in zip(m.name, m.position)}
    sys.stdout.write(json.dumps(d) + '\n'); sys.stdout.flush()
rospy.Subscriber(topic, JointState, cb, queue_size=1)
rospy.spin()
"""
        topic = getattr(c, "joint_state_topic", "/joint_states") or "/joint_states"
        cmd = ["docker", "exec", "-i", c.name, "bash", "-lc",
               f"source {ROS_SETUP} 2>/dev/null; "
               f"python3 -u -c {shlex.quote(reader)} {shlex.quote(topic)}"]
        c._joint_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        def _drain(p, cc):
            try:
                for line in p.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        cc._joint_latest = json.loads(line)
                        cc._joint_latest_ts = time.time()
                    except Exception:
                        pass
            except Exception:
                pass
        threading.Thread(target=_drain, args=(c._joint_proc, c), daemon=True).start()
        return c._joint_proc

    def read_joints(self, username: str):
        """Latest joint positions in the spec's KINEMATIC order (mapped by NAME,
        since /joint_states may report a different/alphabetical order), or None."""
        c = self.cs.get(username)
        if not c or c.state != "ready" or not c.is_arm:
            return None
        try:
            fresh = (c._joint_proc is None or c._joint_proc.poll() is not None)
            self._ensure_joint_reader(c)
        except Exception:
            return None
        if fresh:
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline and not c._joint_latest:
                time.sleep(0.2)
        d = c._joint_latest
        if not d:
            return None
        try:
            return [float(d.get(n, 0.0)) for n in c.joint_names]
        except Exception:
            return None

    # ── LIDAR scan reader (Track B-A) ──────────────────────────────────────────
    # Mirrors the odom reader: a persistent /scan subscriber per container that
    # keeps the latest LaserScan in a compact form. Only started for models whose
    # spec declares the 'lidar' capability (burger has none, so it never runs).
    def _ensure_scan_reader(self, c: ClientContainer):
        if c._scan_proc and c._scan_proc.poll() is None:
            return c._scan_proc
        # Throttle to ~5 Hz and drop intensities; replace inf/nan with 0.0 so the
        # JSON stays valid. Ranges are rounded to keep the line small.
        reader = r"""
import rospy, json, sys, math, time
from sensor_msgs.msg import LaserScan
rospy.init_node('webui_scan_read', anonymous=True, disable_signals=True)
topic = sys.argv[1] if len(sys.argv) > 1 else '/scan'
_last = [0.0]
def cb(m):
    now = time.time()
    if now - _last[0] < 0.18:
        return
    _last[0] = now
    rs = []
    for r in m.ranges:
        if r is None or math.isinf(r) or math.isnan(r):
            rs.append(0.0)
        else:
            rs.append(round(r, 3))
    sys.stdout.write(json.dumps({'angle_min': round(m.angle_min, 5),
                                 'angle_increment': round(m.angle_increment, 6),
                                 'ranges': rs}) + '\n')
    sys.stdout.flush()
rospy.Subscriber(topic, LaserScan, cb, queue_size=1)
rospy.spin()
"""
        topic = getattr(c, "scan_topic", "/scan") or "/scan"
        cmd = ["docker", "exec", "-i", c.name, "bash", "-lc",
               f"source {ROS_SETUP} 2>/dev/null; "
               f"python3 -u -c {shlex.quote(reader)} {shlex.quote(topic)}"]
        c._scan_proc = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1,
        )
        def _drain(p, cc):
            try:
                for line in p.stdout:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        cc._scan_latest = json.loads(line)
                        cc._scan_latest_ts = time.time()
                    except Exception:
                        pass
            except Exception:
                pass
        threading.Thread(target=_drain, args=(c._scan_proc, c), daemon=True).start()
        return c._scan_proc

    def read_scan(self, username: str):
        """Latest LIDAR scan for a LIDAR-equipped model, else None."""
        c = self.cs.get(username)
        if not c or c.state != "ready" or not getattr(c, "has_lidar", False):
            return None
        try:
            fresh = (c._scan_proc is None or c._scan_proc.poll() is not None)
            self._ensure_scan_reader(c)
        except Exception:
            return None
        if fresh:
            deadline = time.monotonic() + 6.0
            while time.monotonic() < deadline and not c._scan_latest:
                time.sleep(0.2)
        d = c._scan_latest
        if d and "ranges" in d:
            return dict(d)
        return None

    # ── go-to-goal (closed-loop, server-side, odom feedback) ───────────────────
    def goto(self, username: str, gx: float, gy: float, gyaw_deg=None,
             on_done=None, on_phase=None):
        """Drive the client's robot to (gx, gy) and optionally face gyaw_deg.
        Runs in a background thread using odom feedback + publish(). Cancels any
        previous goto for this user. `on_done(reason)` is called when finished:
        reason in {'reached', 'cancelled', 'error', 'timeout'}. `on_phase(text)`
        is called when the controller changes phase (driving -> turning)."""
        c = self.cs.get(username)
        if not c or c.state != "ready":
            if on_done:
                on_done("error")
            return False
        # cancel any in-flight goto first
        self.cancel_goto(username)
        # manual navigation overrides obstacle auto-drive
        self.stop_obstacle_mode(username)
        c._goto_cancel = threading.Event()
        ev = c._goto_cancel

        def _run_goto():
            import math
            reason = "error"
            try:
                # make sure odom reader is alive
                self._ensure_odom_reader(c)
                def pose():
                    return c._odom_latest
                # wait for a first odom sample
                t0 = time.monotonic()
                while not pose() and time.monotonic() - t0 < 6.0:
                    if ev.is_set():
                        return
                    time.sleep(0.2)
                if not pose():
                    reason = "error"
                    return

                def norm(a):
                    while a > math.pi:  a -= 2*math.pi
                    while a < -math.pi: a += 2*math.pi
                    return a

                POS_TOL = 0.025         # metres (final accepted error ~2.5cm)
                YAW_TOL = math.radians(1.5)
                V_MAX, W_MAX = 0.16, 0.7
                deadline = time.monotonic() + 150.0   # safety cap

                if on_phase:
                    try: on_phase("Driving to point…")
                    except Exception: pass

                # Phase 1: drive to the (x, y) point. Decisive approach (no crawl)
                # so it doesn't systematically stop short; correction passes clean
                # up any overshoot. Drives until WITHIN tolerance, not "close".
                def drive_to_point():
                    while not ev.is_set() and time.monotonic() < deadline:
                        p = pose()
                        if not p:
                            time.sleep(0.05); continue
                        dx, dy = gx - p["x"], gy - p["y"]
                        dist = math.hypot(dx, dy)
                        if dist <= POS_TOL:
                            break
                        heading = math.atan2(dy, dx)
                        yaw_err = norm(heading - p["yaw"])
                        w = max(-W_MAX, min(W_MAX, 1.6 * yaw_err))
                        if abs(yaw_err) > math.radians(25):
                            v = 0.0                       # rotate in place to aim
                        else:
                            # higher gain + floor so it actually reaches, doesn't crawl
                            v = max(0.05, min(V_MAX, dist * 0.9))
                        self.publish(username, v, w)
                        time.sleep(0.05)
                    self.publish(username, 0.0, 0.0)

                for _pass in range(5):
                    drive_to_point()
                    if ev.is_set():
                        break
                    time.sleep(0.6)          # let it fully stop + odom catch up
                    p = pose() or {}
                    err = math.hypot(gx - p.get("x", 1e9), gy - p.get("y", 1e9))
                    if err <= POS_TOL:
                        break               # genuinely at the point
                    # else: overshot/short -> loop drives again to correct

                if ev.is_set():
                    reason = "cancelled"; return

                # Phase 2: rotate to the requested final yaw (if any).
                if gyaw_deg is not None:
                    if on_phase:
                        try: on_phase("At point — turning to final angle…")
                        except Exception: pass
                    goal_yaw = math.radians(float(gyaw_deg))
                    for _yp in range(5):
                        while not ev.is_set() and time.monotonic() < deadline:
                            p = pose()
                            if not p:
                                time.sleep(0.05); continue
                            err = norm(goal_yaw - p["yaw"])
                            if abs(err) <= YAW_TOL:
                                break
                            w = max(-W_MAX, min(W_MAX, 1.1 * err))
                            if abs(w) < 0.10:
                                w = 0.10 if w > 0 else -0.10
                            self.publish(username, 0.0, w)
                            time.sleep(0.05)
                        self.publish(username, 0.0, 0.0)
                        if ev.is_set():
                            break
                        time.sleep(0.4)      # settle, re-check (coast correction)
                        p = pose() or {}
                        if "yaw" in p and abs(norm(goal_yaw - p["yaw"])) <= YAW_TOL:
                            break

                # Settle fully, THEN measure the true final error (robot has
                # finished coasting), so the report matches reality.
                self.publish(username, 0.0, 0.0)
                time.sleep(0.5)
                pf = pose() or {}
                pos_err = math.hypot(gx - pf.get("x", 1e9), gy - pf.get("y", 1e9))
                yaw_err_deg = None
                if gyaw_deg is not None and "yaw" in pf:
                    yaw_err_deg = abs(math.degrees(norm(
                        math.radians(float(gyaw_deg)) - pf["yaw"])))
                c._goto_final = {"pos_err": pos_err, "yaw_err": yaw_err_deg}

                if ev.is_set():
                    reason = "cancelled"
                elif time.monotonic() >= deadline:
                    reason = "timeout"
                elif pos_err <= POS_TOL * 1.6:
                    reason = "reached"
                else:
                    reason = "stopped_short"
            finally:
                self.publish(username, 0.0, 0.0)
                c._goto_thread = None
                if on_done:
                    try: on_done(reason, getattr(c, "_goto_final", None))
                    except Exception: pass

        t = threading.Thread(target=_run_goto, daemon=True)
        c._goto_thread = t
        t.start()
        return True

    def cancel_goto(self, username: str):
        c = self.cs.get(username)
        if not c:
            return
        try:
            c._goto_cancel.set()
        except Exception:
            pass

    # ── obstacle-detection auto-drive (emanual 10.2, LIDAR-driven) ─────────────
    # Drives straight forward while the path ahead is clear, and STOPS AND STAYS
    # stopped when the nearest return in the front ±90° sector is within
    # AUTO_STOP_DIST. It never steers on its own — the only thing that changes the
    # heading is the client turning the robot (manual teleop), which briefly takes
    # over via auto_steer(); once pointed at a clear path the robot resumes. A
    # hysteresis gap (resume only when clear beyond AUTO_RESUME_DIST) stops tiny
    # odom drift from un-sticking the stop and making the robot wander.
    AUTO_FWD_SPEED   = 0.15   # m/s forward when clear
    AUTO_STOP_DIST   = 0.75   # m; stop if anything ahead is closer than this
    AUTO_RESUME_DIST = 0.95   # m; only resume once the front is clear beyond this
    AUTO_FOV_DEG     = 90.0   # half field-of-view: watch the front ±90°
    AUTO_HEADING_KP  = 1.6    # heading-hold gain (rad/s per rad of yaw error)
    AUTO_HEADING_WMAX = 0.4   # cap on the heading-hold correction (rad/s)
    AUTO_SCAN_STALE  = 1.0    # s; if no fresh /scan within this, treat as blocked

    # ── teleop safety guard (LIDAR) ────────────────────────────────────────────
    # Manual teleop has no obstacle awareness, so driving forward into a wall/pillar
    # pins the robot. This guard blocks ONLY the forward component when something is
    # close ahead; reverse + turning stay free so you can always back out. Narrow
    # cone (so turning isn't blocked by side walls) and FAIL-OPEN (no fresh scan ->
    # don't block, so teleop never feels dead). Nav is unaffected: move_base writes
    # /cmd_vel directly and never goes through publish().
    TELEOP_GUARD_DIST  = 0.30   # m; hard-stop forward only if something is this close
    TELEOP_GUARD_SLOW  = 0.45   # m; start ramping forward speed down only from here
    TELEOP_GUARD_FOV   = 20.0   # deg half-cone ahead — narrow, so off-axis pillars
                                # in the arena don't crawl your forward speed. Only
                                # an obstacle nearly dead-ahead slows/stops you.
    TELEOP_GUARD_STALE = 1.0    # s; ignore the guard if the scan is older than this
    # The sim LIDAR sometimes emits junk beams pinned at its min range (~0.12 m) in
    # open space. Those are closer than the robot's own body, so any return below
    # this floor is a sensor artifact, not a real obstacle — ignore it.
    SCAN_MIN_VALID     = 0.15   # m; discard returns at/below this as artifacts

    def _front_nearest(self, c, fov_deg=None):
        """Nearest valid return in the front ±fov_deg sector (default AUTO_FOV_DEG),
        or None if clear / no scan. Rejects ISOLATED single-beam spikes: the sim
        LIDAR can emit one stray short beam in open space, and a lone beam would
        otherwise dead-stop forward teleop. A REAL obstacle lights up several
        adjacent beams, so a return is only trusted if a nearby beam corroborates
        it at a similar range."""
        d = c._scan_latest
        if not d or not d.get("ranges"):
            return None
        amin = d.get("angle_min", 0.0)
        ainc = d.get("angle_increment", 0.0) or 0.0
        half = math.radians(self.AUTO_FOV_DEG if fov_deg is None else fov_deg)
        # Collect valid, in-cone returns as (beam_index, range).
        valid = []
        for i, r in enumerate(d["ranges"]):
            if not r or r <= self.SCAN_MIN_VALID:   # <=0/inf, or min-range junk
                continue
            ang = amin + i * ainc
            while ang > math.pi:  ang -= 2 * math.pi
            while ang < -math.pi: ang += 2 * math.pi
            if abs(ang) <= half:
                valid.append((i, r))
        if not valid:
            return None
        # Walk returns nearest-first; accept the first one corroborated by a
        # neighbouring beam (within 2 indices) at a similar range (within 0.4 m).
        valid.sort(key=lambda t: t[1])
        for idx, r in valid:
            for j, rj in valid:
                if j != idx and abs(j - idx) <= 2 and abs(rj - r) <= 0.4:
                    return r
        return None   # only lone spikes -> treat as clear (don't block forward)

    def start_obstacle_mode(self, username: str) -> bool:
        c = self.cs.get(username)
        if not c or c.state != "ready" or not getattr(c, "has_lidar", False):
            return False
        self.cancel_goto(username)
        self.stop_obstacle_mode(username)
        c._auto_cancel = threading.Event()
        ev = c._auto_cancel
        c._auto_on = True
        c._auto_blocked = False
        c._auto_manual_until = 0.0

        def _run_auto():
            try:
                self.read_scan(username)              # ensure /scan reader is alive
                self.read_odom(username)              # ensure /odom reader is alive
                blocked = False                        # latched stop state
                lock_yaw = None                        # heading to hold while driving
                while not ev.is_set():
                    # ── decide blocked, with hysteresis + stale-scan safety ──────
                    stale = (time.time() - getattr(c, "_scan_latest_ts", 0.0)
                             > self.AUTO_SCAN_STALE)
                    if stale:
                        blocked = True                 # no fresh LIDAR -> never drive blind
                    else:
                        nearest = self._front_nearest(c)
                        if nearest is not None and nearest < self.AUTO_STOP_DIST:
                            blocked = True
                        elif nearest is None or nearest > self.AUTO_RESUME_DIST:
                            blocked = False
                    c._auto_blocked = blocked

                    # ── yield to the client while they're steering ───────────────
                    if time.time() < c._auto_manual_until:
                        lock_yaw = None                # re-lock heading after they steer
                        time.sleep(0.05)
                        continue

                    if blocked:
                        lock_yaw = None
                        self.publish(username, 0.0, 0.0)
                        time.sleep(0.1)
                        continue

                    # ── drive forward, holding a straight heading ────────────────
                    cur = (c._odom_latest or {}).get("yaw")
                    wz = 0.0
                    if cur is not None:
                        if lock_yaw is None:
                            lock_yaw = cur             # lock the heading we start straight on
                        err = lock_yaw - cur
                        while err > math.pi:  err -= 2 * math.pi
                        while err < -math.pi: err += 2 * math.pi
                        wz = max(-self.AUTO_HEADING_WMAX,
                                 min(self.AUTO_HEADING_WMAX, self.AUTO_HEADING_KP * err))
                    self.publish(username, self.AUTO_FWD_SPEED, wz)
                    time.sleep(0.1)
            finally:
                self.publish(username, 0.0, 0.0)
                c._auto_on = False
                c._auto_blocked = False
                c._auto_thread = None

        t = threading.Thread(target=_run_auto, daemon=True)
        c._auto_thread = t
        t.start()
        return True

    def auto_steer(self, username: str, vx: float, wz: float) -> None:
        """Client steering while obstacle mode is on: briefly take over the auto
        loop and apply the command. Turning and reversing are always allowed;
        forward is blocked while an obstacle is within the stop distance, so the
        client can't drive into it. This does NOT cancel obstacle mode."""
        c = self.cs.get(username)
        if not c:
            return
        c._auto_manual_until = time.time() + 0.4
        if c._auto_blocked and vx > 0.0:
            vx = 0.0
        self.publish(username, vx, wz)

    def stop_obstacle_mode(self, username: str) -> bool:
        c = self.cs.get(username)
        if not c:
            return False
        try:
            c._auto_cancel.set()
        except Exception:
            pass
        c._auto_on = False
        c._auto_blocked = False
        for _ in range(2):
            self.publish(username, 0.0, 0.0)
            time.sleep(0.05)
        return True

    def auto_status(self, username: str) -> bool:
        c = self.cs.get(username)
        return bool(c and getattr(c, "_auto_on", False))

    # ── ROS Navigation stack (move_base) ───────────────────────────────────────
    # Brings up map_server (our generated arena map) + a static map->odom + the
    # real move_base (TurtleBot3 stock costmap/DWA params) inside the container.
    # Goals go to /move_base_simple/goal; move_base plans a global path AROUND the
    # pillars and follows it with the DWA local planner. Heavy under emulation.
    def start_navigation(self, username: str) -> bool:
        c = self.cs.get(username)
        if not c or c.state != "ready" or not models.supports_navigation(c.model_id):
            return False
        # navigation is the sole driver: stop manual auto-drive + goto
        self.cancel_goto(username)
        self.stop_obstacle_mode(username)
        self.stop_navigation(username)            # clean any previous nav

        # 1. write geom + map builder + nav launch into the container, and build
        #    the map if it isn't there yet (cached across restarts).
        files = {
            models.NAV_GEOM_PATH:    json.dumps(models.nav_map_inputs(c.model_id)),
            models.NAV_BUILDER_PATH: models.MAP_BUILDER_PY,
            models.NAV_LAUNCH_PATH:  models.nav_launch_xml(c.model_id) or "",
        }
        parts = [f"mkdir -p {shlex.quote(models.ARENA_DIR)}"]
        for path, content in files.items():
            b64 = base64.b64encode(content.encode()).decode()
            parts.append(f"echo {shlex.quote(b64)} | base64 -d > {shlex.quote(path)}")
        parts.append(
            f"test -f {shlex.quote(models.NAV_MAP_PGM)} || "
            f"python3 {shlex.quote(models.NAV_BUILDER_PATH)} "
            f"{shlex.quote(models.NAV_GEOM_PATH)} {shlex.quote(models.NAV_MAP_PGM)} "
            f"{shlex.quote(models.NAV_MAP_YAML)}"
        )
        _run(["docker", "exec", c.name, "bash", "-lc", " && ".join(parts)], timeout=180)

        # 2. launch the nav stack (tagged so we can kill it), detached.
        spec = models.get_spec(c.model_id)
        env_exports = "".join(
            f"export {k}={shlex.quote(str(v))}; " for k, v in spec.env.items()
        )
        launch_cmd = (
            f": {models.NAV_TAG}; source {ROS_SETUP} 2>/dev/null; {env_exports}"
            f"export LIBGL_ALWAYS_SOFTWARE=1; "
            f"roslaunch {shlex.quote(models.NAV_LAUNCH_PATH)} "
            f"> {shlex.quote(models.NAV_LOG_PATH)} 2>&1"
        )
        c._nav_proc = subprocess.Popen(
            ["docker", "exec", c.name, "bash", "-lc", launch_cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        c._nav_on = True
        return True

    def stop_navigation(self, username: str) -> bool:
        c = self.cs.get(username)
        if not c:
            return False
        c._nav_on = False
        # Kill move_base + map_server by process name, and the nav roslaunch +
        # static tf by split-pattern (so the kill command can't match itself, and
        # so we never touch the SIM's roslaunch).
        kill = (
            "pkill -9 move_base 2>/dev/null; "
            "pkill -9 map_server 2>/dev/null; "
            "pkill -9 amcl 2>/dev/null; "
            "pkill -9 -f 'nav''.launch' 2>/dev/null; "
            "pkill -9 -f 'WEBUI_''NAV' 2>/dev/null; true"
        )
        try:
            _run(["docker", "exec", c.name, "bash", "-lc", kill], timeout=15)
        except Exception:
            pass
        try:
            if c._nav_proc:
                c._nav_proc.terminate()
        except Exception:
            pass
        c._nav_proc = None
        self.publish(username, 0.0, 0.0)
        return True

    def nav_status(self, username: str) -> bool:
        c = self.cs.get(username)
        return bool(c and getattr(c, "_nav_on", False))

    def send_nav_goal(self, username: str, x: float, y: float, yaw_deg: float) -> bool:
        """Publish a goal (map frame) to move_base; it plans + drives there."""
        c = self.cs.get(username)
        if not c or not getattr(c, "_nav_on", False):
            return False
        pub = r"""
import rospy, sys, math
from geometry_msgs.msg import PoseStamped
rospy.init_node('webui_nav_goal', anonymous=True, disable_signals=True)
p = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
m = PoseStamped()
m.header.frame_id = 'map'
m.header.stamp = rospy.Time.now()
m.pose.position.x = x
m.pose.position.y = y
m.pose.orientation.z = math.sin(yaw / 2.0)
m.pose.orientation.w = math.cos(yaw / 2.0)
t0 = rospy.Time.now()
r = rospy.Rate(20)
while p.get_num_connections() < 1 and (rospy.Time.now() - t0).to_sec() < 3.0:
    r.sleep()
p.publish(m)
rospy.sleep(0.6)
"""
        yaw_rad = math.radians(yaw_deg)
        cmd = (f"source {ROS_SETUP} 2>/dev/null; "
               f"python3 -u -c {shlex.quote(pub)} {x} {y} {yaw_rad}")
        try:
            _run(["docker", "exec", c.name, "bash", "-lc", cmd], timeout=10)
            return True
        except Exception:
            return False

    # ── STOP: kill running user script + cancel goto + zero velocity ───────────
    def stop_motion(self, username: str):
        """Hard stop for this client: cancel any goto, kill any running user
        Python script in their container, and publish zero velocity."""
        c = self.cs.get(username)
        if not c:
            return
        # 1. cancel goto controller
        self.cancel_goto(username)
        # 1b. cancel obstacle auto-drive
        try:
            c._auto_cancel.set()
            c._auto_on = False
        except Exception:
            pass
        # 2. kill any python running the user's script inside the container
        #    (rospy can ignore SIGTERM, so SIGKILL).
        if c.state == "ready":
            try:
                _run(["docker", "exec", c.name, "bash", "-lc",
                      "pkill -9 -f /tmp/ros_ops_run.py 2>/dev/null; "
                      "pkill -9 -f ros_ops_run 2>/dev/null; true"], timeout=8)
            except Exception:
                pass
        # 3. zero velocity (a few times to be sure it lands)
        for _ in range(3):
            self.publish(username, 0.0, 0.0)
            time.sleep(0.05)


# Singleton
container_manager = ContainerManager()
