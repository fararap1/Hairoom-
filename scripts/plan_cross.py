#!/usr/bin/env python3
"""Let MoveIt/OMPL plan a collision-free crossover between two taught poses
whose elbow sign differs (e.g. +K j3=+140 -> +KR j3=-139). The standard
build_plan_goal keeps j3>=0 along the WHOLE path, which makes an elbow-flip
target unreachable; here we drop that path constraint so OMPL may straighten the
arm (j3 through 0) and fold to the far side -- exactly the hand motion "rise up,
turn, come down". The path is still fully collision-checked by the planner.

PREVIEW/BUILD ONLY unless you replay the saved file. Writes
routes/paths/<from>__<to>.yaml. Moves nothing.

    python3 scripts/plan_cross.py +K +KR
env:
    PLAN_ATTEMPTS  (default 20)
    PLAN_TIME_S    (default 15)
    PLAN_OUT       output yaml
"""
import math, os, sys, threading, time
import numpy as np
import rclpy, yaml
from rclpy.action import ActionClient
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetPositionFK

REPO = '/home/farrukh/Desktop/Hairoom/v2'
APPROACH = os.path.join(REPO, 'config', 'approach_poses.yaml')
ARM = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
GROUP = 'fairino5_v6_group'
TIP = 'tcp_tool'


def main():
    if len(sys.argv) < 3:
        print('usage: plan_cross.py <from> <to>'); return 1
    a_name, b_name = sys.argv[1], sys.argv[2]
    attempts = int(os.environ.get('PLAN_ATTEMPTS', '20'))
    ptime = float(os.environ.get('PLAN_TIME_S', '15'))
    out = os.environ.get('PLAN_OUT',
        os.path.join(REPO, 'routes', 'paths',
                     f"{a_name.replace('+','plus')}__{b_name.replace('+','plus')}.yaml"))
    poses = yaml.safe_load(open(APPROACH))['poses']
    for nm in (a_name, b_name):
        if nm not in poses:
            print(f'{nm} not in approach_poses.yaml'); return 1
    A = [float(v) for v in poses[a_name]]
    B = [float(v) for v in poses[b_name]]

    rclpy.init()
    node = rclpy.create_node('plan_cross')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    fk = node.create_client(GetPositionFK, '/compute_fk')
    fk.wait_for_service(timeout_sec=10)
    mg = ActionClient(node, MoveGroup, '/move_action')
    if not mg.wait_for_server(timeout_sec=10):
        print('move_action unavailable'); return 1

    def call_fk(q):
        req = GetPositionFK.Request()
        req.header = Header(frame_id='head_link'); req.fk_link_names = [TIP]
        req.robot_state.joint_state = JointState(name=ARM, position=[float(v) for v in q])
        f = fk.call_async(req)
        while not f.done():
            pass
        ps = f.result().pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) * 1000.0
        o = ps.orientation; x, y, z, w = o.x, o.y, o.z, o.w
        zax = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
        return p, zax/np.linalg.norm(zax)

    K, _ = call_fk(A)   # aim reference = +K tip

    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = GROUP
    req.num_planning_attempts = attempts
    req.allowed_planning_time = ptime
    req.max_velocity_scaling_factor = 0.05
    req.max_acceleration_scaling_factor = 0.05
    # explicit start state = A (NOT is_diff): plan the +K -> +KR segment offline
    req.start_state = RobotState()
    req.start_state.joint_state = JointState(name=ARM, position=A)
    req.start_state.is_diff = False
    c = Constraints()
    for name, pos in zip(ARM, B):
        jc = JointConstraint()
        jc.joint_name = name; jc.position = float(pos)
        jc.tolerance_above = 0.001; jc.tolerance_below = 0.001; jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)
    # NO path constraint: allow the elbow to flip through j3=0
    goal.planning_options.plan_only = True
    goal.planning_options.replan = False

    print(f'planning {a_name} -> {b_name} (no elbow constraint, {attempts} attempts, {ptime:.0f}s)...')
    fut = mg.send_goal_async(goal)
    while not fut.done():
        time.sleep(0.02)
    gh = fut.result()
    if not gh.accepted:
        print('goal rejected'); return 1
    rf = gh.get_result_async()
    while not rf.done():
        time.sleep(0.02)
    res = rf.result().result
    if res.error_code.val != 1:
        print(f'PLAN FAILED error_code={res.error_code.val}'); return 1
    jt = res.planned_trajectory.joint_trajectory
    idx = [jt.joint_names.index(j) for j in ARM]
    pts = [[p.positions[i] for i in idx] for p in jt.points]
    times = [p.time_from_start.sec + p.time_from_start.nanosec*1e-9 for p in jt.points]
    print(f'planned {len(pts)} points, duration {times[-1]:.1f}s')

    # report
    j3s = [math.degrees(q[2]) for q in pts]
    j1s = [math.degrees(q[0]) for q in pts]
    strays, aims = [], []
    jjerk, worstj = 0.0, ''
    prev = None
    for q in pts:
        p, z = call_fk(q)
        d = K - p; dist = np.linalg.norm(d); strays.append(dist)
        aims.append(math.degrees(math.acos(np.clip(z @ (d/(dist+1e-9)), -1, 1))))
        if prev is not None:
            diffs = [abs(math.degrees(q[i]-prev[i])) for i in range(6)]
            if max(diffs) > jjerk:
                jjerk = max(diffs); worstj = f'j{diffs.index(max(diffs))+1}'
        prev = q
    print(f'j1: {j1s[0]:.1f} -> {j1s[-1]:.1f} deg   j3: {j3s[0]:.1f} -> {min(j3s):.1f} .. -> {j3s[-1]:.1f}')
    print(f'tip standoff from {a_name}: {strays[0]:.0f} -> APEX {max(strays):.0f} -> {strays[-1]:.0f} mm')
    print(f'aim to {a_name} tip: mean {np.mean(aims):.0f} deg (planner does not enforce aim)')
    print(f'max joint step between waypoints: {jjerk:.1f} deg ({worstj})')

    doc = {'from': a_name, 'to': b_name, 'skill': 'crossover_planned',
           'note': f'OMPL collision-free crossover {a_name}->{b_name}, elbow allowed to flip through 0.',
           'velocity_scale': 0.05, 'joint_names': ARM,
           'points': [{'positions': [float(v) for v in q],
                       'velocities': [0.0]*6, 'accelerations': [0.0]*6,
                       'time_from_start': round(t, 4)}
                      for q, t in zip(pts, times)]}
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, 'w') as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)
    print(f'\nwrote {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
