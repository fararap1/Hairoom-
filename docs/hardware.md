# Hardware and network

State as verified **2026-07-21**.

| | |
|---|---|
| Robot | Fairino FR5, 6 axis |
| Controller | `192.168.58.2`, firmware V3.9.5-QX / web v3.9.1 |
| Host | Ubuntu 22.04, ROS 2 Humble |
| Host NIC | `enp46s0` = `192.168.58.100/24` |
| ROS workspace | `~/hairoom_ws` (the only one) |

## Controller ports

Probed and confirmed open on 2026-07-21: **20003, 20004, 20005, 8083, 80**.

- `20003` — XML-RPC command channel (`/RPC2`). What the driver's `write()` and
  all read-only tooling in this repo use.
- `20005` — SDK state channel. `libfairino`'s `RPC()` needs it; the official
  `ros2_control` driver hangs without it.

Port 20005 being open is what makes the official driver viable here. It was
closed earlier in this project's history, which is why custom bridges were
written. If the official driver ever hangs on activation again, probe this port
first:

```bash
timeout 3 bash -c 'cat < /dev/null > /dev/tcp/192.168.58.2/20005' && echo open
```

## Read-only health check

None of this commands motion:

```python
import xmlrpc.client
s = xmlrpc.client.ServerProxy('http://192.168.58.2:20003/RPC2')
s.GetActualJointPosDegree(0)   # [err, j1..j6] in degrees
s.GetRobotErrorCode()          # [0, 0, 0] == healthy
s.GetSDKVersion()
```

Note `GetRobotState` and `GetRobotMode` are **not defined** on this firmware —
they return fault -506.

## Head placement decides whether anything works — moved closer 2026-07-21

The head used to sit at `x = -0.78` (926 mm from the base), which put the
landmarks ~886 mm out with the elbow at **`j3 = +8°`** — the arm working
essentially straight, at the end of its reach. That single fact caused most of
the trouble in this project:

- No Cartesian glide between adjacent landmarks was possible at all. A→B, two
  points **72 mm apart**, died at 21 % of the path with `NO_IK_SOLUTION`.
- The failure was identical with collision checking off, with the elbow
  constraint removed and with the jump threshold disabled — all four variants
  returned exactly 0.213, which is what ruled out every software explanation.
- Adding standoff made it *worse* (0.213 at 0 mm → 0.050 at 40 mm), because
  bulging away from the head pushes further out of reach.
- Searching 120 random seeds found **no solution at all** for the middle
  35–85 % of the straight line between A and B. Not a continuity problem: that
  volume was simply outside the workspace.
- Per-landmark `INVALID_MOTION_PLAN` failures came and went for the same
  reason — there was no margin to plan within.

`HEAD_BASE_POSE` is now `(-0.62, 0.00, 0.50)` — **130 mm closer**, 796 mm from
the base. Measured immediately afterwards:

| | before (x = −0.78) | after (x = −0.62) |
|---|---|---|
| A→B chord, no collisions | fraction 0.200 | **1.000** |
| A→B glide, collisions on | impossible at any standoff | **complete at 1.5 mm** |
| elbow on arrival at B | `j3 = +7°` | **`j3 = +53°`** |

So the glide was never a software problem. It was 130 mm of table.

**This constant must match where the head physically is.** It was moved with no
mannequin present, so the mesh is currently just a conservative planning
obstacle. The moment a real head goes back, re-measure and set this to the
truth — a wrong value means collision checking is protecting the wrong volume.
Landmarks live in `head_link` and move with it, so they do not need re-teaching
when this changes, but they *do* need re-teaching if the head moves physically
without this being updated.

One caveat from the move: landmarks **E** and **I** now report goal states in
collision (`forearm_link~tool_link` and `sink~upperarm_link`). Closer is not
uniformly better — it trades reach margin for tighter clearances against the
sink and the arm's own links.

## Safety

- Real hardware. Treat every motion as safety-critical.
- MoveIt velocity/acceleration scaling is **0.10** (raised from 0.05 on
  2026-07-21 at the operator's request, with no mannequin present).
  `scripts/run_landmarks.py` and `scripts/glide.py` share this value;
  `legacy/hairoom_run.py` hard-caps at 0.1 regardless. Put it back to 0.05
  before working next to anyone.
- Joint limits are asymmetric on J2/J4: **−265.1°…+85.0°**.
- A "jog exceeds limit" fault needs **physical** recovery — hold the drag-teach
  button and manually pull the joint off the limit. It cannot be cleared from
  software.
- Always verify arrival against the robot directly (XML-RPC), not only against
  ROS topics.

## Tool

`moveit_config_overlay/config/hairoom_tool.xacro` — a cylinder rigidly fixed to
`wrist3_link`, with `tcp_tool` as the tip frame. It is a **real URDF link**, so
MoveIt collision-checks it.

`TOOL_LENGTH_M = 0.20` and `TOOL_RADIUS_M = 0.02` are **placeholders pending a
real measurement.** This matters: this cylinder is what collided with the head
mesh during planning, so its true dimensions directly affect which landmarks are
reachable. Every consumer (visual, collision, inertial, and the `tcp_tool` tip
offset) derives from those two properties plus `FLANGE_FACE_Z` — retuning means
editing only those.

The +Z mounting axis was verified, not assumed: joint j6 has `axis="0 0 1"`, and
`wrist3_link`'s own mesh spans Z=[0.053, 0.099] in its local frame — entirely
positive, so +Z points out toward the flange face.
