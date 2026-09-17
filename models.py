"""
models.py — Model Spec catalog (Track A1).

A ModelSpec describes how to launch and control ONE robot. The control plumbing
in container_manager.py reads the spec instead of hardcoding TURTLEBOT3_MODEL,
the launch command, and the /cmd_vel / /odom topics — so adding a robot becomes
"add a spec".

Phase A1 ships the three builtin TurtleBot3 variants (all already baked into the
ros_noetic image). Later phases add `source="git"` models, sensors
(lidar/camera), and manipulators; the `capabilities` field is here already so
the dashboard and sensor/arm tracks can branch on it without another refactor.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import math


@dataclass(frozen=True)
class ModelSpec:
    id: str
    name: str
    description: str = ""
    source: str = "builtin"                      # "builtin" | "git"
    env: dict = field(default_factory=dict)      # env vars exported before launch
    launch: str = ""                             # roslaunch args
    cmd_vel_topic: str = "/cmd_vel"
    odom_topic: str = "/odom"
    scan_topic: str = "/scan"          # LIDAR scan (Track B); used when 'lidar' in capabilities
    capabilities: Tuple[str, ...] = ("mobile_base",)
    world: str = "empty"               # key into WORLD_GEOMETRY (for the 3D obstacle overlay)
    # Manipulator-arm control (Track A-arm): used when 'arm' in capabilities. The
    # arm is driven by streaming a trajectory_msgs/JointTrajectory to
    # joint_cmd_topic and read back from /joint_states. joint_names is the
    # KINEMATIC order (base -> tip); /joint_states may report a different order,
    # so feedback is always mapped by name.
    joint_cmd_topic: str = ""          # e.g. /eff_joint_traj_controller/command
    joint_state_topic: str = "/joint_states"
    joint_names: Tuple[str, ...] = ()
    # git-only (unused in A1, reserved for Track A3/A4):
    git_url: str = ""
    build_cmd: str = ""


# Burger stays in the empty world (light, no LIDAR). The sensor models go in
# turtlebot3_world (a ring of cylindrical pillars + an outer wall) so their LIDAR
# actually has obstacles to detect — an empty world returns all-inf (nothing to
# draw). Both ship with the ros_noetic image.
_TB3_EMPTY = "turtlebot3_gazebo turtlebot3_empty_world.launch gui:=false"
_TB3_WORLD = "turtlebot3_gazebo turtlebot3_world.launch gui:=false"

CATALOG: Dict[str, ModelSpec] = {
    "tb3_burger": ModelSpec(
        id="tb3_burger",
        name="TurtleBot3 Burger",
        description="Small 2-wheel differential drive. Lightest to simulate.",
        env={"TURTLEBOT3_MODEL": "burger"},
        launch=_TB3_EMPTY,
        capabilities=("mobile_base",),
    ),
    "tb3_waffle": ModelSpec(
        id="tb3_waffle",
        name="TurtleBot3 Waffle",
        description="Larger base with camera + 360° LIDAR, in a big obstacle arena.",
        env={"TURTLEBOT3_MODEL": "waffle"},
        launch=_TB3_WORLD,
        capabilities=("mobile_base", "lidar", "camera"),
        world="ros_ops_arena",
    ),
    "tb3_waffle_pi": ModelSpec(
        id="tb3_waffle_pi",
        name="TurtleBot3 Waffle Pi",
        description="Waffle variant with Pi camera + LIDAR, in a big obstacle arena.",
        env={"TURTLEBOT3_MODEL": "waffle_pi"},
        launch=_TB3_WORLD,
        capabilities=("mobile_base", "lidar", "camera"),
        world="ros_ops_arena",
    ),
    # ── Manipulator arms ──────────────────────────────────────────────────────
    # UR5: 6-DOF industrial arm. No mobile base / odom / LIDAR — it is driven by
    # streaming joint-angle trajectories to the effort joint-trajectory
    # controller that ur_gazebo's bringup starts, and read back from /joint_states.
    "ur5": ModelSpec(
        id="ur5",
        name="Universal Robots UR5",
        description="6-DOF industrial arm. Joint control (shoulder, elbow, 3× wrist).",
        launch="ur_gazebo ur5_bringup.launch gui:=false",
        capabilities=("arm",),
        joint_cmd_topic="/eff_joint_traj_controller/command",
        joint_state_topic="/joint_states",
        joint_names=(
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
        ),
    ),
}

DEFAULT_MODEL_ID = "tb3_burger"


# ── World obstacle geometry (for the dashboard's 3D overlay) ───────────────────
# These primitives let the web 3D view draw the SAME static obstacles the robot
# sees in Gazebo, as solid shapes (not just the LIDAR dot cloud). Coordinates are
# in the ROS world frame (metres), matching /odom — the client maps them with the
# same x,y -> three.js (x, 0, -y) convention as the robot and the scan cloud.
#
# turtlebot3_world is fully primitive in its model.sdf: 9 cylinders (r=0.15,
# h=0.5) on a 3x3 grid at x,y in {-1.1, 0, +1.1}. The outer boundary is a 6-piece
# hexagonal mesh wall; we approximate it as a regular hexagon ring (the cylinders
# are exact, the wall is a faithful boundary, not a mesh copy).

def _hexagon_walls(apothem: float, height: float, thickness: float = 0.05) -> List[dict]:
    """6 wall segments forming a regular hexagon (one flat side facing +x),
    returned as endpoint pairs so the client can draw each as a thin box."""
    R = apothem / math.cos(math.radians(30))   # vertex (circum) radius
    verts = [(R * math.cos(math.radians(a)), R * math.sin(math.radians(a)))
             for a in (30, 90, 150, 210, 270, 330)]
    segs = []
    for i in range(6):
        x1, y1 = verts[i]
        x2, y2 = verts[(i + 1) % 6]
        segs.append({"x1": round(x1, 3), "y1": round(y1, 3),
                     "x2": round(x2, 3), "y2": round(y2, 3),
                     "h": height, "t": thickness})
    return segs


_ARENA_SCALE = 10.0                       # 10x bigger than the stock turtlebot3_world
# The real TB3 LIDAR only reaches 3.5 m, but the arena is scaled up, so the sensor
# would be blind across the wide gaps (breaking SLAM/AMCL). Size it to just see the
# pillar grid (~11 m apart) — NOT the full arena. Full range (~35 m) made each scan
# ~10x heavier and starved /scan under emulation. Patched into the robot's xacro at
# spawn (container_manager._launch_sim), along with a lower beam count.
LIDAR_MAX_RANGE = round(1.65 * _ARENA_SCALE, 1)   # ~16.5 m: sees the nearest pillars
LIDAR_SAMPLES   = 120                              # down from 360 — lighter scans under QEMU
_CYL_R   = round(0.15 * _ARENA_SCALE, 3)  # 1.5 m pillar radius
_CYL_H   = round(0.50 * _ARENA_SCALE, 3)  # 5.0 m pillar height
_GRID    = round(1.10 * _ARENA_SCALE, 3)  # 11.0 m grid spacing
_APOTHEM = round(3.30 * _ARENA_SCALE, 3)  # 33.0 m wall distance from centre
_WALL_T  = round(0.05 * _ARENA_SCALE, 3)  # 0.5 m wall thickness

# 8 pillars on a 3x3 grid with the CENTRE removed — the robot spawns at (0,0),
# so that cell must stay clear (and it doubles as the odom origin).
_ARENA_CYLINDERS = [
    {"x": x, "y": y, "r": _CYL_R, "h": _CYL_H}
    for x in (-_GRID, 0.0, _GRID) for y in (-_GRID, 0.0, _GRID)
    if not (x == 0.0 and y == 0.0)
]

WORLD_GEOMETRY: Dict[str, dict] = {
    "empty": {"cylinders": [], "walls": []},
    "ros_ops_arena": {
        "cylinders": _ARENA_CYLINDERS,
        "walls": _hexagon_walls(apothem=_APOTHEM, height=_CYL_H, thickness=_WALL_T),
    },
}


def world_geometry(model_id: str) -> dict:
    """Obstacle primitives for the world the given model launches in."""
    s = CATALOG.get(model_id)
    key = (s.world if s else "empty")
    return WORLD_GEOMETRY.get(key, WORLD_GEOMETRY["empty"])


def has_world_geometry(model_id: str) -> bool:
    g = world_geometry(model_id)
    return bool(g.get("cylinders") or g.get("walls"))


# ── Generated Gazebo world (so the REAL sim matches the 3D overlay exactly) ────
# The big arena isn't a stock turtlebot3 world, so we generate a self-contained
# .world (ground + sun + the same pillars/walls the dashboard draws) and a launch
# file that loads it and spawns the robot at the origin (the cleared centre cell).
# Spawning at (0,0) makes the robot's odom frame coincide with the world frame,
# so the overlay lines up with the robot with no offset.
ARENA_DIR         = "/root/.ros_ops"
ARENA_WORLD_PATH  = ARENA_DIR + "/arena.world"
ARENA_LAUNCH_PATH = ARENA_DIR + "/arena.launch"
_GENERATED_WORLDS = {"ros_ops_arena"}


def uses_generated_world(model_id: str) -> bool:
    s = CATALOG.get(model_id)
    return bool(s and s.world in _GENERATED_WORLDS)


def world_sdf(model_id: str) -> Optional[str]:
    """A complete Gazebo SDF world for a generated world, else None."""
    s = CATALOG.get(model_id)
    if not s or s.world not in _GENERATED_WORLDS:
        return None
    g = WORLD_GEOMETRY[s.world]
    links = []
    for i, c in enumerate(g["cylinders"]):
        links.append(f"""
      <link name='pillar_{i}'>
        <pose>{c['x']} {c['y']} {c['h']/2.0} 0 0 0</pose>
        <collision name='c'><geometry><cylinder><radius>{c['r']}</radius><length>{c['h']}</length></cylinder></geometry></collision>
        <visual name='v'><geometry><cylinder><radius>{c['r']}</radius><length>{c['h']}</length></cylinder></geometry>
          <material><ambient>0.18 0.72 0.42 1</ambient><diffuse>0.22 0.82 0.48 1</diffuse></material></visual>
      </link>""")
    for i, w in enumerate(g["walls"]):
        cx, cy = (w["x1"] + w["x2"]) / 2.0, (w["y1"] + w["y2"]) / 2.0
        L = math.hypot(w["x2"] - w["x1"], w["y2"] - w["y1"])
        yaw = math.atan2(w["y2"] - w["y1"], w["x2"] - w["x1"])
        links.append(f"""
      <link name='wall_{i}'>
        <pose>{round(cx,3)} {round(cy,3)} {w['h']/2.0} 0 0 {round(yaw,5)}</pose>
        <collision name='c'><geometry><box><size>{round(L,3)} {w['t']} {w['h']}</size></box></geometry></collision>
        <visual name='v'><geometry><box><size>{round(L,3)} {w['t']} {w['h']}</size></box></geometry>
          <material><ambient>0.17 0.35 0.5 1</ambient><diffuse>0.20 0.42 0.6 1</diffuse></material></visual>
      </link>""")
    return f"""<?xml version='1.0' ?>
<sdf version='1.6'>
  <world name='default'>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <model name='arena'>
      <static>true</static>{''.join(links)}
    </model>
  </world>
</sdf>
"""


def world_launch_xml(model_id: str) -> Optional[str]:
    """A roslaunch file that loads the generated world and spawns the robot at the
    origin. Mirrors turtlebot3_world.launch (reuses its description + spawn), only
    swapping the world file and the spawn pose."""
    if not uses_generated_world(model_id):
        return None
    return f"""<launch>
  <arg name='model' default='$(env TURTLEBOT3_MODEL)'/>
  <arg name='gui' default='false'/>
  <arg name='x_pos' default='0.0'/>
  <arg name='y_pos' default='0.0'/>
  <arg name='z_pos' default='0.0'/>
  <include file='$(find gazebo_ros)/launch/empty_world.launch'>
    <arg name='world_name' value='{ARENA_WORLD_PATH}'/>
    <arg name='paused' value='false'/>
    <arg name='use_sim_time' value='true'/>
    <arg name='gui' value='$(arg gui)'/>
    <arg name='headless' value='false'/>
    <arg name='debug' value='false'/>
  </include>
  <param name='robot_description' command='$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro' />
  <node pkg='gazebo_ros' type='spawn_model' name='spawn_urdf' args='-urdf -model turtlebot3_$(arg model) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -param robot_description' />
</launch>
"""


def get_spec(model_id: str) -> Optional[ModelSpec]:
    return CATALOG.get(model_id)


def is_valid(model_id: str) -> bool:
    return model_id in CATALOG


def is_arm(model_id: str) -> bool:
    """True for manipulator-arm models (joint control, no mobile base)."""
    s = CATALOG.get(model_id)
    return bool(s and "arm" in s.capabilities)


def catalog_public() -> List[dict]:
    """Compact, JSON-safe list for the client model picker."""
    return [
        {
            "id": s.id,
            "name": s.name,
            "description": s.description,
            "capabilities": list(s.capabilities),
            "world_geometry": world_geometry(s.id),
            # Arm models ship their joint list so the UI can build sliders.
            "joint_names": list(s.joint_names),
        }
        for s in CATALOG.values()
    ]


# ── ROS Navigation stack (move_base) support ──────────────────────────────────
# We bring up the real Navigation stack inside the client container: map_server
# serving a map we GENERATE from the arena geometry (no SLAM drive needed), a
# static map->odom transform (the robot spawns at the origin and Gazebo odom is
# reliable, so we skip AMCL — the heaviest piece — under emulation), and move_base
# with TurtleBot3's stock costmap + DWA params. Goals are published to
# /move_base_simple/goal; move_base plans a global path AROUND the pillars and a
# DWA local plan to follow it.
NAV_MAP_RES   = 0.3                       # m/cell (coarser = far less costmap work under emulation; pillars are big)
NAV_MAP_BOUND = 41.0                      # map spans [-B, +B] in x and y (arena ~±38)
_MAP_TAG = "r%02d" % int(round(NAV_MAP_RES * 100))   # changes with res -> forces a rebuild
NAV_LAUNCH_PATH  = ARENA_DIR + "/nav.launch"
NAV_GEOM_PATH    = ARENA_DIR + "/arena_geom.json"
NAV_BUILDER_PATH = ARENA_DIR + "/make_map.py"
NAV_MAP_YAML     = ARENA_DIR + "/arena_map_" + _MAP_TAG + ".yaml"
NAV_MAP_PGM      = ARENA_DIR + "/arena_map_" + _MAP_TAG + ".pgm"
NAV_LOG_PATH     = ARENA_DIR + "/nav.log"  # move_base/roslaunch output (tail to debug)
NAV_TAG          = "WEBUI_NAV"            # marker to find/kill the nav launch


def supports_navigation(model_id: str) -> bool:
    # Navigation needs the generated arena (a map) + a LIDAR for the costmap.
    s = CATALOG.get(model_id)
    return bool(s and uses_generated_world(model_id) and "lidar" in s.capabilities)


def nav_map_inputs(model_id: str) -> dict:
    """Geometry + map params the in-container builder rasterises into an
    occupancy grid. Same obstacle coords as the world + overlay."""
    g = WORLD_GEOMETRY[CATALOG[model_id].world]
    b = NAV_MAP_BOUND
    return {
        "res": NAV_MAP_RES,
        "bounds": [-b, -b, b, b],
        "cylinders": g["cylinders"],
        "walls": g["walls"],
    }


# Pure-Python (numpy if available) occupancy-grid generator, run inside the
# container. Reads NAV_GEOM_PATH, writes the .pgm + .yaml map_server expects.
MAP_BUILDER_PY = r'''
import json, math, os, sys
G = json.load(open(sys.argv[1]))
res = G["res"]; x0, y0, x1, y1 = G["bounds"]
W = int(round((x1 - x0) / res)); H = int(round((y1 - y0) / res))
cyl = G["cylinders"]; walls = G["walls"]
pgm = sys.argv[2]; yml = sys.argv[3]
try:
    import numpy as np
    xs = x0 + (np.arange(W) + 0.5) * res
    ys = y0 + (np.arange(H) + 0.5) * res
    WX, WY = np.meshgrid(xs, ys)            # H x W; row j -> ys[j] (ascending)
    occ = np.zeros((H, W), dtype=bool)
    for c in cyl:
        occ |= (WX - c["x"])**2 + (WY - c["y"])**2 <= c["r"]**2
    for w in walls:
        mx = (w["x1"] + w["x2"]) / 2.0; my = (w["y1"] + w["y2"]) / 2.0
        dx = w["x2"] - w["x1"]; dy = w["y2"] - w["y1"]; L = math.hypot(dx, dy) or 1.0
        ux, uy = dx / L, dy / L
        RX = WX - mx; RY = WY - my
        along = RX * ux + RY * uy; perp = -RX * uy + RY * ux
        occ |= (np.abs(along) <= L / 2.0) & (np.abs(perp) <= w["t"] / 2.0)
    img = np.where(occ, 0, 254).astype(np.uint8)
    img = np.flipud(img)                    # row 0 = max y, as map_server expects
    with open(pgm, "wb") as f:
        f.write(("P5\n%d %d\n255\n" % (W, H)).encode()); f.write(img.tobytes())
except Exception as e:
    sys.stderr.write("numpy path failed (%s), using pure python\n" % e)
    wseg = []
    for w in walls:
        mx = (w["x1"] + w["x2"]) / 2.0; my = (w["y1"] + w["y2"]) / 2.0
        dx = w["x2"] - w["x1"]; dy = w["y2"] - w["y1"]; L = math.hypot(dx, dy) or 1.0
        wseg.append((mx, my, dx / L, dy / L, L / 2.0, w["t"] / 2.0))
    def occ1(wx, wy):
        for c in cyl:
            if (wx - c["x"])**2 + (wy - c["y"])**2 <= c["r"]**2: return True
        for mx, my, ux, uy, hl, ht in wseg:
            rx = wx - mx; ry = wy - my
            if abs(rx*ux + ry*uy) <= hl and abs(-rx*uy + ry*ux) <= ht: return True
        return False
    row = bytearray(W)
    with open(pgm, "wb") as f:
        f.write(("P5\n%d %d\n255\n" % (W, H)).encode())
        for j in range(H):
            wy = y0 + (H - 1 - j) * res + res / 2.0
            for i in range(W):
                wx = x0 + i * res + res / 2.0
                row[i] = 0 if occ1(wx, wy) else 254
            f.write(bytes(row))
with open(yml, "w") as f:
    f.write("image: %s\nresolution: %g\norigin: [%g, %g, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
            % (os.path.basename(pgm), res, x0, y0))
print("map %dx%d res %g written" % (W, H, res))
'''


def nav_launch_xml(model_id: str) -> Optional[str]:
    """Custom navigation launch: robot_state_publisher (tf) + map_server +
    static map->odom + move_base. We load TurtleBot3's stock costmap/planner
    params but then drop the update rates: stock runs every costmap AND the
    DWA controller at 10 Hz, which saturates the CPU under emulation. The map
    is static, so 1 Hz global is plenty. No AMCL, no RViz."""
    if not supports_navigation(model_id):
        return None
    return f"""<launch>
  <arg name='model' default='$(env TURTLEBOT3_MODEL)'/>
  <arg name='map_file' default='{NAV_MAP_YAML}'/>
  <!-- tf for base_footprint -> base_scan etc. (needed by the costmaps) -->
  <include file='$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch'>
    <arg name='model' value='$(arg model)'/>
  </include>
  <!-- our generated arena map -->
  <node pkg='map_server' name='map_server' type='map_server' args='$(arg map_file)'/>
  <!-- AMCL: localise against the map with the LIDAR and publish a CORRECTED
       map->odom (this is what fixes odometry drift). Robot spawns at the origin,
       so the initial pose is (0,0,0). Particle counts trimmed for emulation, and
       laser_max_range matches the (scaled) LIDAR so it can use far returns. -->
  <node pkg='amcl' type='amcl' name='amcl'>
    <param name='min_particles' value='100'/>
    <param name='max_particles' value='800'/>
    <param name='kld_err' value='0.05'/>
    <param name='update_min_d' value='0.20'/>
    <param name='update_min_a' value='0.20'/>
    <param name='resample_interval' value='2'/>
    <param name='transform_tolerance' value='0.5'/>
    <param name='recovery_alpha_slow' value='0.0'/>
    <param name='recovery_alpha_fast' value='0.0'/>
    <param name='initial_pose_x' value='0.0'/>
    <param name='initial_pose_y' value='0.0'/>
    <param name='initial_pose_a' value='0.0'/>
    <param name='gui_publish_rate' value='10.0'/>
    <remap from='scan' to='scan'/>
    <param name='laser_max_range' value='{LIDAR_MAX_RANGE}'/>
    <param name='laser_max_beams' value='180'/>
    <param name='laser_z_hit' value='0.5'/>
    <param name='laser_z_rand' value='0.5'/>
    <param name='laser_sigma_hit' value='0.2'/>
    <param name='laser_likelihood_max_dist' value='2.0'/>
    <param name='laser_model_type' value='likelihood_field'/>
    <param name='odom_model_type' value='diff'/>
    <param name='odom_alpha1' value='0.1'/>
    <param name='odom_alpha2' value='0.1'/>
    <param name='odom_alpha3' value='0.1'/>
    <param name='odom_alpha4' value='0.1'/>
    <param name='odom_frame_id' value='odom'/>
    <param name='base_frame_id' value='base_footprint'/>
    <param name='global_frame_id' value='map'/>
  </node>
  <!-- the real planner: global plan + DWA local plan + costmaps. Stock TB3
       params, then emulation-friendly rate overrides (loaded last = they win). -->
  <node pkg='move_base' type='move_base' respawn='false' name='move_base' output='screen'>
    <param name='base_local_planner' value='base_local_planner/TrajectoryPlannerROS'/>
    <rosparam file='$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml' command='load' ns='global_costmap'/>
    <rosparam file='$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml' command='load' ns='local_costmap'/>
    <rosparam file='$(find turtlebot3_navigation)/param/local_costmap_params.yaml' command='load'/>
    <rosparam file='$(find turtlebot3_navigation)/param/global_costmap_params.yaml' command='load'/>
    <rosparam file='$(find turtlebot3_navigation)/param/move_base_params.yaml' command='load'/>
    <remap from='cmd_vel' to='/cmd_vel'/>
    <remap from='odom' to='odom'/>
    <!-- ↓↓↓ emulation rate overrides (loaded last = they win) ↓↓↓ -->
    <param name='controller_frequency' value='3.0'/>
    <param name='planner_frequency' value='0.5'/>
    <param name='global_costmap/update_frequency' value='1.0'/>
    <param name='global_costmap/publish_frequency' value='0.5'/>
    <param name='local_costmap/update_frequency' value='4.0'/>
    <param name='local_costmap/publish_frequency' value='2.0'/>
    <!-- local planner: base_local_planner (always installed) in DWA mode.
         dwa_local_planner is NOT in this image. 10x arena: faster, looser goal. -->
    <param name='TrajectoryPlannerROS/dwa' value='true' type='bool'/>
    <param name='TrajectoryPlannerROS/meter_scoring' value='true' type='bool'/>
    <param name='TrajectoryPlannerROS/holonomic_robot' value='false' type='bool'/>
    <param name='TrajectoryPlannerROS/max_vel_x' value='0.5'/>
    <param name='TrajectoryPlannerROS/min_vel_x' value='0.1'/>
    <param name='TrajectoryPlannerROS/max_vel_theta' value='1.82'/>
    <param name='TrajectoryPlannerROS/min_vel_theta' value='-1.82'/>
    <param name='TrajectoryPlannerROS/min_in_place_vel_theta' value='0.9'/>
    <param name='TrajectoryPlannerROS/acc_lim_x' value='2.5'/>
    <param name='TrajectoryPlannerROS/acc_lim_theta' value='3.2'/>
    <param name='TrajectoryPlannerROS/sim_time' value='1.5'/>
    <param name='TrajectoryPlannerROS/vx_samples' value='12'/>
    <param name='TrajectoryPlannerROS/vtheta_samples' value='20'/>
    <param name='TrajectoryPlannerROS/pdist_scale' value='0.6'/>
    <param name='TrajectoryPlannerROS/gdist_scale' value='0.8'/>
    <param name='TrajectoryPlannerROS/occdist_scale' value='0.02'/>
    <param name='TrajectoryPlannerROS/xy_goal_tolerance' value='0.25'/>
    <param name='TrajectoryPlannerROS/yaw_goal_tolerance' value='0.3'/>
  </node>
</launch>
"""
