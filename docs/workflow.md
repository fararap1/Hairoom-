# Workflow — bring-up, teaching, running

## 1. Bring the stack up

```bash
./bin/start_hairoom.sh          # headless
./bin/start_hairoom.sh --rviz   # with RViz
```

It refuses to launch unless the robot answers and ports 20003 + 20005 are open,
then waits until `fairino5_controller` reports `active`.

Expect in the log:

```
[FairinoHardwareInterface]: 机械臂SDK连接成功！          SDK connected
[FairinoHardwareInterface]: 初始指令位置: ...            seeded from the REAL pose
[FairinoHardwareInterface]: 机械臂硬件启动成功!          hardware started
[controller_manager]: update rate is 100 Hz
Configured and activated fairino5_controller
Configured and activated joint_state_broadcaster
```

The "初始指令位置" line should match the robot's actual joints — that is the
driver seeding its command buffer from reality so nothing jumps on startup.

Sanity checks:

```bash
ros2 control list_controllers            # both active
ros2 control list_hardware_components    # plugin = fairino_hardware/FairinoHardwareInterface
ros2 topic echo /joint_states --once
```

Tear down with `./bin/stop_hairoom.sh`.

> Duplicate publishers are a recurring failure here — two `hairoom_head_tf.py`
> fighting over `base_link→head_link`, or an orphaned `robot_state_publisher`
> outliving its launch tree. `stop_hairoom.sh` sweeps both and warns if anything
> survives. If ROS logs *"nodes in the graph that share an exact name"*, stop and
> clean up before running anything.

## 2. Teach landmarks

```bash
python3 scripts/hairoom_teach.py
```

Read-only with respect to the robot — no publishers, no action clients, nothing
that can command motion. It only reads TF and writes YAML.

Hand-guide the arm so `tcp_tool` touches a point, press Enter, name it. The pose
is stored **in `head_link`**, so landmarks stay valid however the arm got there,
and survive the head being repositioned relative to the robot. Saved to
`config/head_landmarks.yaml`.

## 3. Check before moving (always)

```bash
python3 scripts/check_goals.py         # IK + collision validity per landmark
python3 scripts/dry_run_landmarks.py   # plan-only survey, robot never moves
```

`check_goals.py` answers "is this target itself reachable and collision-free".
`dry_run_landmarks.py` answers "can a path be planned to it right now" — bear in
mind RRTConnect is randomised, so its verdict is a snapshot, not a guarantee.

## 4. Move

Smallest possible real motion, to prove the chain end to end — j6 rotates about
the tool axis, so the TCP barely translates:

```bash
python3 scripts/demo_move.py 5     # +5°
python3 scripts/demo_move.py -5    # back
```

Traverse landmarks:

```bash
python3 scripts/run_landmarks.py A B C D E F G center I J
```

Per point it re-plans from the robot's real current state, re-validates the
start state, executes exactly the trajectory it checked, then confirms arrival
by reading the robot over XML-RPC. A planning failure skips that point and
continues (nothing moved); an execution failure or a non-zero robot error code
aborts the whole sequence.

`run_landmarks.py` runs unattended. `legacy/hairoom_run.py` is the same pipeline
but waits for Enter before each point — prefer it when a human is watching and
you want a gate on every motion.

## Gliding along the head

```bash
python3 scripts/glide.py A B                       # one hop
python3 scripts/glide.py --dwell 2 J I center G F A B C D E   # a route
```

Each hop is a Cartesian path that follows the scalp instead of lifting away:
a chord between the two taught poses, bulged outward along the surface normal
(taken from the tool axis at each end), orientation slerped. The standoff is
searched from 0 upward in 0.5 mm steps and the smallest collision-free value
wins, so the tool hugs the surface as closely as the geometry allows.

**Hop length is what decides feasibility.** A chord between distant points cuts
deep into the skull, and no amount of bulge fixes that — it stops being a
surface glide. Measured on the current landmark set:

| hop length | outcome |
|---|---|
| 46–77 mm | complete, 0–4 mm standoff |
| 118 mm | complete, but needs 18 mm standoff |
| 153–190 mm | **no collision-free glide at all** |

So **order the route by proximity**. The order landmarks happen to sit in the
YAML is not a route: running A→B→C→D→E→F→… fails at E→F (189 mm) and drops
three points. A nearest-neighbour tour keeps every hop under 80 mm:

```
J -> I -> center -> G -> F -> A -> B -> C -> D -> E
hops: 65, 70, 75, 74, 46, 72, 60, 69, 77 mm
```

That route completed all nine hops with `robot_err 0`, standoffs of 0–9 mm and
the elbow between +47° and +118°. Recompute the tour if landmarks change; the
pairwise distance table is trivial to regenerate from
`config/head_landmarks.yaml`.

A failed hop leaves the tool where it was and the route continues from there,
so one unreachable point does not abandon the rest.

## After pulling upstream `frcobot_ros2`

```bash
./bin/install_overlay.sh
```

Everything in `moveit_config_overlay/` is a local change to a vendored upstream
package. A pull silently reverts them — on 2026-07-18 the V3.9.5 update restored
the mock `ros2_control` stack, which published fake all-zeros `/joint_states` at
~100 Hz, outvoted the real feedback, and made every plan fail on a phantom
collision. Re-run the overlay after any pull.
