# Architecture — how the robot is actually driven

Verified end-to-end on hardware **2026-07-21** (all 10 landmarks traversed,
zero controller errors).

The robot is driven by the **official `ros2_control` driver**. No custom
bridge is involved in the normal path. `legacy/hairoom_action_bridge.py` is
kept only as a fallback and is not started by anything here.

## The chain, top to bottom

```
scripts/run_landmarks.py          your code — plain ROS 2 node, no robot socket
   │
   │  landmark pose in head_link  (config/head_landmarks.yaml)
   ▼
TF: head_link → base_link         so points survive the head being moved
   │
   ▼
/compute_ik            (avoid_collisions=True, seeded from current state)
/check_state_validity  independent confirmation the solution is collision-free
   │
   │  one specific, validated joint configuration
   ▼
/move_action (MoveGroup)          plan_only=True — plans, moves nothing
   │  goal = JointConstraint on all 6 joints, tolerance 0.001 rad
   │  planner: OMPL RRTConnect · vel/acc scaling 0.05
   ▼
start-state re-validation         plan's first point vs live /joint_states
   │                              (>0.02 rad apart ⇒ plan is stale, abort)
   ▼
/execute_trajectory               executes exactly the trajectory just checked
   │
   ▼
move_group → MoveItSimpleControllerManager
   │
   ▼
/fairino5_controller/follow_joint_trajectory
   │  joint_trajectory_controller, spline interpolation
   ▼
controller_manager                100 Hz: read() → update() → write()
   │
   ▼
fairino_hardware/FairinoHardwareInterface
   │  read():  GetActualJointPosDegree  → /joint_states  (real measurement)
   │  write(): ServoJ                   → position servo stream
   ▼
XML-RPC  http://192.168.58.2:20003/RPC2
```

## Why joint-space goals, not pose goals

**This is the single most important thing in this repo.**

A pose goal carries tolerances (e.g. a 1 cm position sphere and 0.1 rad
orientation). Inside that region OMPL is free to sample **its own** goal
configuration — and on this setup it repeatedly picked an IK branch that
drives `tool_link` through the head mesh. MoveIt then correctly refused to
execute:

```
Computed path is not valid. Invalid states at index locations:
[ 270 ... 283 ] out of 309
Found a contact between 'head' (Object) and 'tool_link' (Robot link)
```

The invalid states are at the **end** of the trajectory — the goal itself.

The landmarks are not the problem: `scripts/check_goals.py` solves IK for all
10 and checks each with `/check_state_validity` — **all 10 are valid and
collision-free**. The fix is to remove the planner's freedom: solve IK first,
verify it, then name those joint values as the goal. The path in between is
still fully collision-checked.

`legacy/hairoom_run.py` still uses the pose-goal-with-tolerance form and is
therefore subject to this same failure.

## Choosing the IK branch — why "nearest", not "any"

A 6-DOF arm reaches the same tool pose in several configurations. **KDL returns
*a* valid solution, never the nearest one**, and the branch it lands in differs
between identical calls.

Measured on landmark G, seeded with landmark F's exact pose, 8 calls:

| result | calls | base swing |
|---|---|---|
| j1 = +67.1° | 4 / 8 | −0.1° |
| j1 = −92.2° | 4 / 8 | **−159.4°** |

A coin flip on the same request. Raising the solver timeout does not help — at
50 ms the far branch came back 5 times out of 6. Proximity is simply not a
criterion KDL has. (`kinematics.yaml`: `KDLKinematicsPlugin`, 5 ms.)

The first traversal accepted whatever came back, validated it for collisions
only, and so commanded a **159° base swing** on F→G and **119°** on center→J.
MoveIt was not at fault: it planned a correct path to a badly chosen goal.

`run_landmarks.py` now samples `IK_SEED_ATTEMPTS` seeds, ranks the distinct
solutions by weighted joint-space distance (j1 weighted heaviest — rotating the
base sweeps the largest volume, which is what matters beside a head), and takes
the nearest one that is collision-free and inside `MAX_BASE_SWING_DEG` /
`MAX_JOINT_TRAVEL_DEG`.

Confirmed on hardware 2026-07-21: worst base swing **62°**, both pathological
swings gone.

### Two costs of this, both real

- **Nearest is not always plannable.** Some far branches were *easier* to reach.
  Reached points went from 8/10 to 7/10 in one pass; F in particular now picks a
  near branch whose approach path collides.
- **Reachability depends on approach direction.** G is nearly free from F
  (−0.1° swing) but needs ≥96° from the J/I side, where the guard correctly
  refuses it. **The traversal order matters** and is currently arbitrary —
  ordering points so neighbours share a side is unfinished work.

Solution sampling is randomised and the RNG is not seeded, so branch choice
varies between runs. Seed it if you need reproducible sequences.

## Feedback is real, not echoed

`FairinoHardwareInterface::read()` calls `GetActualJointPosDegree` and converts
to radians — `/joint_states` is the robot's **measured** position, not a replay
of commanded values. Confirmed externally: ROS readings matched a direct
XML-RPC poll to 3 decimal places.

## Known rough edges

- **`ServoJ` command period is 0.008 s, `controller_manager` runs at 0.01 s.**
  A mismatch inside the vendor driver. No `ServoJ` errors were observed across
  a full session, but it is there.
- **RRTConnect is randomised.** The same landmark can plan on one attempt and
  fail with `INVALID_MOTION_PLAN` on the next. During the verified run, E and I
  failed first and both succeeded on a retry from a different arm pose. Planning
  failures are harmless (nothing has moved yet) — `run_landmarks.py` skips and
  continues, aborting only on an *execution* failure.
- **IK can pick a far branch.** Landmarks G and center resolved to
  configurations requiring a large reconfiguration (j1 swinging to −92° and
  −115°). It completed cleanly, but for work near a head this should be
  constrained.
- **`GetRobotMotionDone` flickers 0/1 while stationary** — the `ServoJ` stream
  is always active, so the controller intermittently reports "in motion". Judge
  motion by comparing joint positions, not by this flag.
- **The hardware component is named `FakeSystem`.** Cosmetic only: it is the
  `name` argument passed to the xacro macro. The loaded plugin really is
  `fairino_hardware/FairinoHardwareInterface` — confirm with
  `ros2 control list_hardware_components`.
