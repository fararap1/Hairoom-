# Hairoom — operator runbook

Quick terminal commands for daily use. All paths are relative to the repo root
(`~/Desktop/Hairoom/v2`). Every command below assumes ROS is sourced:

```bash
cd ~/Desktop/Hairoom/v2
source /opt/ros/humble/setup.bash
source ~/hairoom_ws/install/setup.bash
```

---

## 1. Start EVERYTHING (arm driver + head + sink + points)

One launch brings up the driver, MoveIt, the head/sink scene, the head TF, and
the 26 landmark markers:

```bash
./bin/start_hairoom.sh          # headless (no RViz)
```

Then RViz on the NVIDIA path (this laptop needs the explicit display/auth):

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  rviz2 -d config/hairoom.rviz &
```

The launch code itself:
- `bin/start_hairoom.sh` — checks the robot answers (ports 20003/20005), then
  runs the bringup launch below.
- `~/hairoom_ws/src/frcobot_ros2/fairino5_v6_moveit2_config/launch/hairoom_bringup.launch.py`
  — the actual launch: driver + controllers + move_group, and it auto-starts
  `scripts/hairoom_scene.py` (head+sink), `scripts/hairoom_head_tf.py` (head TF),
  `scripts/show_landmarks.py` (the 26 points).

Check it came up:
```bash
ros2 control list_controllers          # fairino5_controller must be "active"
python3 scripts/check_goals.py          # read-only sanity, moves nothing
```

---

## 2. Kill ALL connections (before you reboot the arm)

The driver holds a live socket to the robot. **Before rebooting the arm, stop
the stack first** so it releases the connection — otherwise the driver keeps
retrying a dead socket (`ServoJ ...错误码:14`).

```bash
./bin/stop_hairoom.sh           # kills driver, move_group, scene, head_tf, points, rviz
pkill -x rviz2                  # (stop_hairoom already does this; harmless to repeat)
```

`stop_hairoom.sh` is self-safe: it never kills a shell, so it will not take your
terminal down. After it prints `clean`, reboot the arm, then go back to step 1.

Verify nothing is left holding the robot:
```bash
ros2 node list 2>/dev/null | grep -E "move_group|controller" || echo "clean"
```

---

## 3. Run a movement YOURSELF from the terminal

Motions are **saved joint paths** replayed deterministically. Always PREVIEW
first (no `--execute`), watch it in RViz "Planned Path", then add `--execute`.

`replay_path.py` refuses to execute unless the arm is already at the path's
first point (within 3°) — that guard prevents a silent jump. So which command
you run depends on where the arm currently is.

> **DDS note:** `start_hairoom.sh` now forces a UDPv4-only Fast DDS profile
> (`config/fastdds_profile.xml`). This fixes the intermittent "Failed to send
> goal response (timeout)" where the FIRST `/execute` hung and a retry worked —
> it was Fast DDS's shared-memory transport stalling on localhost. If you still
> see it, confirm the profile is loaded (the start log prints `DDS: ...
> UDPv4-only`).

**+H ↔ +K, plain 30 mm arc** (`routes/paths/plusK__plusH.yaml`, forward is +K→+H):

```bash
# preview first (nothing moves):
python3 scripts/replay_path.py routes/paths/plusK__plusH.yaml            # +K -> +H
python3 scripts/replay_path.py routes/paths/plusK__plusH.yaml --reverse  # +H -> +K

# then execute the one whose start = where the arm is now:
python3 scripts/replay_path.py routes/paths/plusK__plusH.yaml --execute            # if arm at +K
python3 scripts/replay_path.py routes/paths/plusK__plusH.yaml --reverse --execute  # if arm at +H
```

So a full **+H → +K → +H** round trip, starting with the arm at +H:
```bash
python3 scripts/replay_path.py routes/paths/plusK__plusH.yaml --reverse --execute  # +H -> +K
python3 scripts/replay_path.py routes/paths/plusK__plusH.yaml --execute            # +K -> +H
```

**Same pair WITH the spray skill** (j4 up/down nod, 20 cycles, ±7.4° ≈ 5 cm, vel 0.15):
```bash
python3 scripts/replay_path.py routes/ready/spray_plusK__plusH.yaml --execute            # +K -> +H, spray
python3 scripts/replay_path.py routes/ready/spray_plusK__plusH.yaml --reverse --execute  # +H -> +K, spray
```

**+C ↔ +H** is the same idea with `routes/paths/plusH__plusC.yaml` /
`routes/ready/spray_plusH__plusC.yaml`.

**+B ↔ +G, LONG hop — arc capped at 18 mm, spray is a j4+j5 nod** (not the
along-line scrub — on this steep arc j3 swings 54°, so scrubbing the path
parameter shook j3 hard; the j4+j5 nod keeps j3 monotone):
```bash
python3 scripts/replay_path.py routes/ready/plusB__plusG.yaml --execute            # +B -> +G, plain 18 mm arc
python3 scripts/replay_path.py routes/ready/spray_plusB__plusG.yaml --execute            # +B -> +G, j4+j5 spray
python3 scripts/replay_path.py routes/ready/spray_plusB__plusG.yaml --reverse --execute  # +G -> +B, j4+j5 spray
```

**+B ↔ +C, LONG hop (182 mm) — arc capped at 12 mm** (straight clears, any
bigger bulge cuts in): `routes/ready/plusB__plusC.yaml` (+B→+C),
`routes/ready/plusC__plusB.yaml` (+C→+B).

Check where the arm is (ground truth, over XML-RPC — not ROS):
```bash
python3 -c "import xmlrpc.client; s=xmlrpc.client.ServerProxy('http://192.168.58.2:20003/RPC2'); print([round(v,1) for v in s.GetActualJointPosDegree(0)[1:]], s.GetRobotErrorCode())"
```

### 3b. Run a whole saved route — ONE command (`bin/run_route.sh`)

For a long, non-stopping route (like the full hairdry loop) use the wrapper. It
sources ROS for you, leads the arm to the route start from wherever it is, sends
the trajectory **straight to the controller** (so MoveIt's duration monitor
can't false-abort a long route), and **verifies the arm really moved** (XML-RPC).

```bash
# 1) PREVIEW in RViz — nothing moves:
./bin/run_route.sh routes/hairdry/hairdry_spray.yaml

# 2) happy with it? RUN on the real arm:
./bin/run_route.sh routes/hairdry/hairdry_spray.yaml --go
```

The full hairdry loop is `+H…+K→+H` with spray (starts and ends at +H, so you
can run it back-to-back). A clean run prints `OK — route complete (arm really
traversed)` with `ACTUAL motion during run: max ~165 deg`. If it instead says
`ARM DID NOT MOVE` or `NOT SUCCESSFUL`, the arm did **not** do the route — do not
assume it worked (a loop ends where it starts, so the end pose alone can't tell).

A trailing `Aborted (core dumped)` after the result is a cosmetic rclpy-shutdown
message — ignore it; the result line above it is what counts.

Works for any saved route yaml, e.g. a single arc:
```bash
./bin/run_route.sh routes/ready/spray_plusK__plusH_horiz.yaml --go
```

---

## 4. If the arm was hand-dragged (drag-teach) — restart before moving

After a manual drag-teach the driver can't servo: `/execute` reports SUCCESS but
NOTHING moves (log fills with `ServoJ ...错误码:14`). Fix = restart the stack so
the driver reconnects:

```bash
./bin/stop_hairoom.sh && ./bin/start_hairoom.sh
```

Saving a taught pose still works even then (it only reads):
```bash
python3 scripts/teach_save.py +K        # writes current joints to config/approach_poses.yaml
```

---

## 5. Files & points inventory (what is saved where)

- `config/approach_poses.yaml` — taught joint poses: **+C, +H, +K** (drag-taught,
  live XML-RPC). Timestamped backups: `config/approach_poses.backup-*.yaml`.
- `config/head_landmarks.yaml` — the 26 landmark points, in `head_link` (they
  ride the head). Backups: `config/head_landmarks.backup-*.yaml`.
- `routes/paths/` — working saved trajectories (rebuilt as we tune).
- `routes/ready/` — FINALIZED skills, kept so rebuilds never clobber them:
  - `plusK__plusH.yaml` — plain 30 mm +K↔+H arc
  - `spray_plusK__plusH.yaml` — +K↔+H with the spray nod
  - `spray_plusH__plusC.yaml` — +C↔+H with the spray nod
  - `plusB__plusG.yaml` — +B↔+G 18 mm arc; `spray_plusB__plusG.yaml` — +B↔+G j4+j5 nod
  - `plusB__plusC.yaml` / `plusC__plusB.yaml` — +B↔+C 12 mm arc (long hop)
- `scripts/` — tools: `replay_path.py` (play a saved path), `spray.py` (build a
  spray path), `spray_blend.py` (j4+j5 level-vertical nod — use on steep arcs),
  `build_arc.py` (smallest 30 mm arc, short hops), `max_arc.py` (LARGEST
  landing-safe arc, long hops), `build_line.py` (straight line),
  `aim_correction.py` (propose a wrist aim tweak), `reseat.py` (joint move
  to a taught pose), `show_traj.py` (loop a path in RViz), `teach_save.py` (save
  a drag-taught pose), `glide.py`, `run_landmarks.py`.

---

## 6. Build a new path / skill (reference)

```bash
# 30 mm arc from the arm's current pose to a taught point (e.g. -> +H):
python3 scripts/build_arc.py +K +H            # saves routes/paths/plusK__plusH.yaml

# add the spray skill on top of a base arc:
python3 scripts/spray.py routes/paths/plusK__plusH.yaml     # -> routes/paths/spray_plusK__plusH.yaml
# spray knobs: SPRAY_J4_AMP_DEG (default 7.4 ≈ 5 cm), SPRAY_CYCLES (20), SPRAY_VEL (0.15)
```

## 7. Build a ROUTE from a sequence of points (no manual reverse)

`route.py` builds a continuous path/loop from a SEQUENCE of taught points. It
scans `routes/paths/` for the plain arc files, builds an undirected graph, and
for each hop uses the stored arc as-is **or reverses it on the fly** — so you
never hand-write a reverse file again (that used to duplicate every edge and
grow as N²). It then stitches with `concat_loop.py` and, optionally, sets a
physical tip speed (`retime_mms.py`) and overlays the j4+j5 spray
(`spray_blend.py`).

```bash
# base loop -> routes/paths/loop_FGBF.yaml (auto-reverses G->B and B->F):
python3 scripts/route.py F G B F

# + physical tip speed 30 mm/s, + j4+j5 spray on top (spray_plusF__plusF.yaml):
TIP_MM_S=30 SPRAY_CYCLES=14 python3 scripts/route.py F G B F --spray
```

Points may be "F" or "+F". Env: `TIP_MM_S` (retime), `LOOP_SCALE`/
`LOOP_JOIN_TOL_DEG` (concat), `SPRAY_MM_S`/`SPRAY_VERT_CM`/`SPRAY_VEL` (spray).

**Spray speed is mm/s too, decoupled from traverse.** `spray_blend.py` takes
`SPRAY_MM_S` (nod tip speed, default 90) and `SPRAY_VERT_CM` (amplitude p-p, 5).
The nod frequency is derived from those, so speeding the traverse up/down leaves
the wag identical (only the cycle count changes). A tukey window zeroes the nod
at both ends, so endpoints stay on the taught points at any frequency.

**Speed is physical now (mm/s), not an abstract scale.** `retime_mms.py` reports
the current tip speed of any path (no arg) and retimes it to `TIP_MM_S`
(constant tool-tip speed via FK arc length), refusing if a joint would exceed
`JOINT_SPEED_CAP_DEG` (default 120). The old `velocity_scale` was only relative
to the 0.10 teach build — e.g. "0.40" was really ~21 mm/s mean.

Golden rule: **preview in RViz first, execute only after you confirm the shape.**
