#!/usr/bin/env python3
"""Arc between two TAUGHT points, bulging RADIALLY outward from the head centre
(reliable "away from head" even for far-apart points on different sides -- the
mean-of-normals bulge in glide.arc_waypoints only works for near landmarks).

Built via /compute_cartesian_path from the taught A joints (offline, no motion),
so the joint path is CONTINUOUS and smoothly timed -- no per-point IK branch
flips, no lurching dt. Orientation slerps A->B (tip keeps looking at the head).
Sweeps the target standoff DOWN, keeps the largest that clears, is elbow-legal,
and lands on the taught B joints. Same output format as max_arc.py.

    A_FROM_TAUGHT=1 STANDOFF_MM=90 python3 scripts/arc_radial.py +IL +HL
"""
import math, os, sys, threading, time
import rclpy
from rclpy.duration import Duration
import yaml
from moveit_msgs.srv import GetPositionFK, GetCartesianPath
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

REPO = '/home/farrukh/Desktop/Hairoom/v2'
sys.path.insert(0, os.path.join(REPO, 'scripts'))
from glide import (qslerp, elbow_path_constraint, retime,  # noqa: E402
                   GROUP, TOOL_LINK, PLANNING_FRAME, JOINTS, MIN_FRACTION,
                   MAX_STEP_M, REVOLUTE_JUMP, VELOCITY_SCALE)
from hairoom_teach import HEAD_FRAME  # noqa: E402
from run_landmarks import check_path_elbow, path_min_j3_deg  # noqa: E402

A_NAME = sys.argv[1]; B_NAME = sys.argv[2]
TARGET_MM = float(os.environ.get('STANDOFF_MM', '90'))
STEP_MM = float(os.environ.get('ARC_STEP_MM', '5'))
VEL = float(os.environ.get('ARC_VEL', str(VELOCITY_SCALE)))
CENTER = [0.0025, -0.1872, 0.0232]          # head centre (centroid of + points), head_link
OUT = os.path.join(REPO, 'routes', 'paths',
                   f'{A_NAME.replace("+","plus")}__{B_NAME.replace("+","plus")}.yaml')


def call(cli, req, timeout=60.0):
    done = threading.Event(); box = {}
    cli.call_async(req).add_done_callback(
        lambda f: (box.__setitem__('r', f.result()), done.set()))
    return box.get('r') if done.wait(timeout) else None


def arc_waypoints_radial(p0, q0, p1, q1, standoff, n=60):
    pts = []
    for i in range(n + 1):
        t = i / n
        p = [(1 - t) * a + t * b for a, b in zip(p0, p1)]
        out = [p[j] - CENTER[j] for j in range(3)]
        L = math.sqrt(sum(c * c for c in out)) or 1.0
        out = [c / L for c in out]
        k = standoff * math.sin(math.pi * t)
        p = [p[j] + k * out[j] for j in range(3)]
        pts.append((p, qslerp(q0, q1, t)))
    return pts


def main():
    poses = yaml.safe_load(open(os.path.join(REPO, 'config', 'approach_poses.yaml')))['poses']
    a_joints = [float(v) for v in poses[A_NAME]]
    b_joints = [float(v) for v in poses[B_NAME]]

    rclpy.init()
    node = rclpy.create_node('arc_radial')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, node)
    fk_cli = node.create_client(GetPositionFK, '/compute_fk')
    cart_cli = node.create_client(GetCartesianPath, '/compute_cartesian_path')
    fk_cli.wait_for_service(timeout_sec=10.0); cart_cli.wait_for_service(timeout_sec=10.0)
    buf.can_transform(PLANNING_FRAME, HEAD_FRAME, rclpy.time.Time(), timeout=Duration(seconds=8.0))

    def fk_head(joints):
        req = GetPositionFK.Request(); req.header.frame_id = PLANNING_FRAME
        req.fk_link_names = [TOOL_LINK]
        rs = RobotState(); rs.joint_state = JointState(name=list(JOINTS), position=[float(x) for x in joints])
        req.robot_state = rs
        res = call(fk_cli, req)
        ph = buf.transform(res.pose_stamped[0], HEAD_FRAME, timeout=Duration(seconds=2.0))
        o = ph.pose.orientation
        return ((ph.pose.position.x, ph.pose.position.y, ph.pose.position.z),
                (o.x, o.y, o.z, o.w))

    def cartesian(wps_head, start_joints):
        poses_pf = []
        for p, q in wps_head:
            ps = PoseStamped(); ps.header.frame_id = HEAD_FRAME
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = p
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = q
            poses_pf.append(buf.transform(ps, PLANNING_FRAME, timeout=Duration(seconds=2.0)).pose)
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME; req.group_name = GROUP; req.link_name = TOOL_LINK
        req.waypoints = poses_pf[1:]; req.max_step = MAX_STEP_M
        req.jump_threshold = 0.0; req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = REVOLUTE_JUMP; req.avoid_collisions = True
        req.path_constraints = elbow_path_constraint()
        rs = RobotState(); rs.joint_state = JointState(name=list(JOINTS), position=[float(x) for x in start_joints])
        req.start_state = rs
        res = call(cart_cli, req, timeout=60.0)
        if res is None:
            return None, 0.0
        return res.solution, res.fraction

    pa, qa = fk_head(a_joints)
    pb, qb = fk_head(b_joints)
    print(f'{A_NAME}->{B_NAME}: chord {math.dist(pa, pb)*1000:.0f} mm; target standoff '
          f'{TARGET_MM:.0f} mm, sweeping DOWN for the largest that clears+lands')
    b_map = dict(zip(JOINTS, b_joints))
    best = None
    s = TARGET_MM
    while s >= -1e-9:
        wps = arc_waypoints_radial(pa, qa, pb, qb, s / 1000.0)
        traj, frac = cartesian(wps, a_joints)
        ok = False; db = None
        if traj is not None and frac >= MIN_FRACTION:
            elbow_ok, _ = check_path_elbow(traj)
            nm = list(traj.joint_trajectory.joint_names)
            last = {n: math.degrees(v) for n, v in zip(nm, traj.joint_trajectory.points[-1].positions)}
            db = max(abs(last[n] - math.degrees(b_map[n])) for n in JOINTS)
            ok = elbow_ok and db <= 5.0
        print(f'  {"OK " if ok else "   "}standoff {s:5.1f} mm -> fraction {frac:.3f}'
              + (f', land {db:.1f} deg' if db is not None else ''))
        if ok:
            best = (s, traj, frac); break
        s -= STEP_MM

    if best is None:
        print('\nNo standoff clears+lands. Nothing saved.'); return 1
    s_mm, traj, frac = best
    traj = retime(traj, VEL)
    jt = traj.joint_trajectory
    dur = jt.points[-1].time_from_start; dur_s = dur.sec + dur.nanosec*1e-9
    print(f'\nchosen standoff {s_mm:.0f} mm, fraction {frac:.3f}, {len(jt.points)} pts, '
          f'j3 min {path_min_j3_deg(traj):+.1f}, {dur_s:.1f}s')
    doc = {'from': A_NAME, 'to': B_NAME, 'shape': 'arc_radial', 'standoff_mm': round(s_mm, 1),
           'velocity_scale': VEL, 'symmetric': True,
           'note': f'radial-from-centre arc, taught {A_NAME}->{B_NAME}, cartesian (smooth), '
                   f'tip aims at head (slerp). play reverse for return.',
           'joint_names': list(jt.joint_names),
           'points': [{'positions': [float(x) for x in p.positions],
                       'velocities': [float(x) for x in p.velocities],
                       'accelerations': [float(x) for x in p.accelerations],
                       'time_from_start': p.time_from_start.sec + p.time_from_start.nanosec*1e-9}
                      for p in jt.points]}
    with open(OUT, 'w') as f:
        yaml.safe_dump(doc, f, default_flow_style=None, sort_keys=False)
    print(f'SAVED -> {OUT}  ({len(jt.points)} pts, {dur_s:.1f}s, standoff {s_mm:.0f} mm)')
    return 0


if __name__ == '__main__':
    rc = main(); sys.stdout.flush(); os._exit(rc)
