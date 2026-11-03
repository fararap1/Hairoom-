# Landmarks

`config/head_landmarks.yaml` — poses taught by hand-guiding `tcp_tool` to touch
a point on the head, recorded in the **`head_link`** frame.

Each entry: `name`, `region`, `position` [x,y,z] m, `orientation` [x,y,z,w]
quaternion, `standoff` (m, currently 0.0 everywhere — an approach offset along
the tool axis, not yet used), `tools` (currently empty everywhere).

## Current set — 10 points, all traversed on hardware 2026-07-21

| name | region | position (m) |
|---|---|---|
| A | — | −0.084, −0.060, 0.005 |
| B | — | −0.058, −0.090, 0.065 |
| C | top of the head | −0.003, −0.101, 0.085 |
| D | right eye side | 0.062, −0.099, 0.061 |
| E | right side | 0.088, −0.045, 0.013 |
| F | left side low 1 | −0.093, −0.065, −0.040 |
| G | left eye low 1 | −0.070, −0.132, −0.019 |
| center | Center | −0.006, −0.165, 0.003 |
| I | right eye low1 | 0.051, −0.147, −0.033 |
| J | right low1 | 0.072, −0.098, −0.072 |

All 10 pass `/compute_ik` + `/check_state_validity`. All 10 were reached with
`robot error code [0,0,0]` at every point. E and I needed a second attempt
(RRTConnect randomness), not a change to the points.

## Why `head_link` and not `base_link`

A landmark in base coordinates is meaningless the moment the head moves relative
to the robot. Stored in `head_link`, the same YAML stays correct — TF does the
conversion at run time. This is also why `hairoom_scene.py` and
`hairoom_head_tf.py` share one pose constant: `hairoom_head_tf.py` imports
`head_link_pose()` directly from `hairoom_scene.py`, so the TF frame cannot
drift away from where the collision mesh actually is.

## Scene alignment

`scripts/hairoom_scene.py` places the head + sink meshes. `SCENE_YAW_DEG`
rotates the whole assembly about the robot base so it faces the arm; both
objects rotate about the same pivot, so their relative geometry is preserved.
Tune by eye in RViz, re-run the script — no rebuild needed. Currently `45.0`.

Landmark coordinates are **not** affected by retuning this, since they live in
`head_link` — but where the head physically is obviously must match reality, or
collision checking is checking the wrong thing.

## Adding points

Run `scripts/hairoom_teach.py` (see `workflow.md`). Re-using an existing name
overwrites that entry. Then verify with `scripts/check_goals.py` before ever
including a new point in a run.

Note the historical set in `../head positions 1/` (A–M with joint angles and
base-frame coordinates) is an **older, different** recording in a different
frame — it is not this file and does not interchange with it. K, L and M have no
equivalent here yet.
