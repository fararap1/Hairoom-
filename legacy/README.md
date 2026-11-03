# legacy/ — superseded, kept deliberately

Nothing here runs in the normal path. Read `../docs/architecture.md` first.

## `hairoom_action_bridge.py` — fallback control path

Custom ROS 2 action server implementing `FollowJointTrajectory` and translating
each goal into blocking `MoveJ` calls over XML-RPC (port 20003), polling
`GetActualJointPosDegree` for `/joint_states`.

It works, and it is worth keeping: it needs only port 20003, whereas the
official driver also needs 20005. If 20005 ever closes again, this is the way
back.

**Never run it alongside the official driver** — they serve the same action name
and both publish `/joint_states`.

Its header comment claims port 20005 is "genuinely closed". That was true when
written; on 2026-07-21 all of 20003/20004/20005/8083/80 were confirmed open,
which is what made the official driver viable.

## `hairoom_run.py` — the earlier landmark runner

Same plan → re-validate → execute pipeline as `scripts/run_landmarks.py`, but it
waits for Enter before every motion. **Prefer it when a human is watching** and
you want a gate on each point.

One caveat: it builds pose goals with position/orientation tolerances, so it is
subject to the OMPL goal-sampling problem described in
`../docs/architecture.md` — planning can fail with `INVALID_MOTION_PLAN` from a
`head`↔`tool_link` contact even though the target itself is fine.

## `start_hairoom_OLD.sh` / `stop_hairoom_OLD.sh`

Bring-up for the **bridge** architecture: they start `hairoom_action_bridge.py`
plus `demo.launch.py` (which deliberately omits `ros2_control_node`). Superseded
by `../bin/start_hairoom.sh`, which launches the official driver and no bridge.

Kept because they document how the fallback path was wired, and `demo.launch.py`
remains the bridge's launch file.
