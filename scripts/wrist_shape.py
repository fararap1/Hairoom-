#!/usr/bin/env python3
"""Shape the wrist of a crossover path to the operator's staged description:
  - j4 barely moves at the START; it does its big turn in the MIDDLE, while the
    arm is rising (a smootherstep sweep between fractions J4_A and J4_B).
  - j5 is solved to look toward the target point (+K) -- so on the way DOWN it
    points more downward at the point.
j1,j2,j3,j6 keep the lift/rotate/lower shape. Endpoints stay the taught poses.
Re-timed to joint-speed limits; every waypoint re-validated for collisions.

PREVIEW/BUILD ONLY -- writes the output yaml, moves nothing.

    python3 scripts/wrist_shape.py routes/paths/plusK__plusKR.planned.yaml +K
env:
    STRIDE     subsample step           (default 5)
    J4_A J4_B  j4 turn happens between these path fractions  (default 0.25 0.60)
    J5_CONT    j5 continuity, deg/deg   (default 0.15)
    SPEED_SCALE  joint-speed scaling    (default 0.05)
    AIM_OUT    output yaml
"""
import math, os, sys, threading
import numpy as np
import rclpy, yaml
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetStateValidity
from moveit_msgs.msg import RobotState

REPO = '/home/farrukh/Desktop/Hairoom/v2'
APPROACH = os.path.join(REPO, 'config', 'approach_poses.yaml')
ARM = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
GROUP = 'fairino5_v6_group'
TIP = 'tcp_tool'
LIM = {'j1':(-175,175),'j2':(-265,85),'j3':(-162,162),
       'j4':(-265,85),'j5':(-175,175),'j6':(-175,175)}
VLIM = {'j1':180,'j2':180,'j3':180,'j4':183,'j5':183,'j6':183}


def smootherstep(x):
    x = max(0.0, min(1.0, x))
    return x*x*x*(x*(x*6 - 15) + 10)


def main():
    if len(sys.argv) < 3:
        print('usage: wrist_shape.py PATH.yaml <target_point>'); return 1
    inp, target_name = sys.argv[1], sys.argv[2]
    STRIDE = int(os.environ.get('STRIDE', '5'))
    J4_A = float(os.environ.get('J4_A', '0.25'))
    J4_B = float(os.environ.get('J4_B', '0.60'))
    J5_CONT = float(os.environ.get('J5_CONT', '0.15'))
    SCALE = float(os.environ.get('SPEED_SCALE', '0.05'))
    out = os.environ.get('AIM_OUT', inp)
    d = yaml.safe_load(open(inp))
    assert d['joint_names'] == ARM
    allpts = [[float(v) for v in p['positions']] for p in d['points']]
    idxs = list(range(0, len(allpts), STRIDE))
    if idxs[-1] != len(allpts) - 1:
        idxs.append(len(allpts) - 1)
    pts = [allpts[i] for i in idxs]
    N = len(pts)
    poses = yaml.safe_load(open(APPROACH))['poses']
    if target_name not in poses:
        print(f'{target_name} not in approach_poses.yaml'); return 1

    rclpy.init()
    node = rclpy.create_node('wrist_shape')
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    fk = node.create_client(GetPositionFK, '/compute_fk')
    sv = node.create_client(GetStateValidity, '/check_state_validity')
    for c, n in ((fk, 'fk'), (sv, 'sv')):
        if not c.wait_for_service(timeout_sec=10):
            print(f'no {n}'); return 1

    def call(client, req):
        f = client.call_async(req)
        while not f.done():
            pass
        return f.result()

    def fkpose(q):
        req = GetPositionFK.Request()
        req.header = Header(frame_id='head_link'); req.fk_link_names = [TIP]
        req.robot_state.joint_state = JointState(name=ARM, position=[float(v) for v in q])
        ps = call(fk, req).pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) * 1000.0
        o = ps.orientation; x, y, z, w = o.x, o.y, o.z, o.w
        zax = np.array([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])
        return p, zax/np.linalg.norm(zax)

    def valid(q):
        req = GetStateValidity.Request(); req.group_name = GROUP
        req.robot_state = RobotState()
        req.robot_state.joint_state = JointState(name=ARM, position=[float(v) for v in q])
        return call(sv, req).valid

    K, _ = fkpose([float(v) for v in poses[target_name]])
    print(f'target {target_name} tip {K.round(1)}  j4 turn between {J4_A:.0%}-{J4_B:.0%}  {N} pts')

    def aim_of(q):
        p, z = fkpose(q)
        dd = K - p; dist = np.linalg.norm(dd)
        return 0.0 if dist < 1 else math.degrees(math.acos(np.clip(z@(dd/dist), -1, 1)))

    def clamp(q):
        for i, j in enumerate(ARM):
            lo, hi = LIM[j]
            q[i] = max(math.radians(lo), min(math.radians(hi), q[i]))
        return q

    def solve_j5(q, s5):
        cur = list(q); cur[4] = s5; cur = clamp(cur)
        def cost(c):
            return aim_of(c) + J5_CONT*abs(math.degrees(c[4]-s5))
        best = cost(cur)
        for step in (0.15, 0.06, 0.025, 0.01):
            improved = True
            while improved:
                improved = False
                for dq in (step, -step):
                    cand = list(cur); cand[4] += dq; cand = clamp(cand)
                    c = cost(cand)
                    if c < best - 1e-4:
                        cur, best = cand, c; improved = True
        return cur[4]

    J5_SPLIT = float(os.environ.get('J5_SPLIT', '0.5'))   # descent starts here
    j4_0, j4_1 = pts[0][3], pts[-1][3]
    j5_0 = pts[0][4]
    newpts = [list(pts[0])]
    seed5 = j5_0
    for k in range(1, N-1):
        s = k/(N-1)
        frac = smootherstep((s - J4_A) / (J4_B - J4_A)) if J4_B > J4_A else (s >= J4_A)
        j4t = j4_0 + (j4_1 - j4_0) * frac        # middle-loaded j4 turn
        q = list(pts[k]); q[3] = j4t
        if s < J5_SPLIT:
            q[4] = j5_0                          # rise: keep j5 calm
        else:
            q[4] = solve_j5(q, seed5)            # descent: j5 looks down at +K
        newpts.append(clamp(q)); seed5 = q[4]
    newpts.append(list(pts[-1]))

    # re-time to joint-speed limits
    times = [0.0]
    for a, b in zip(newpts, newpts[1:]):
        dt = max(abs(math.degrees(b[i]-a[i]))/(VLIM[ARM[i]]*SCALE) for i in range(6))
        times.append(times[-1] + max(dt, 0.05))

    # report
    aims = [aim_of(q) for q in newpts]
    j4c = [math.degrees(q[3]) for q in newpts]
    j5c = [math.degrees(q[4]) for q in newpts]
    bad, limviol, jjerk, worstj = 0, 0, 0.0, ''
    prev = None
    for q in newpts:
        for j, v in zip(ARM, q):
            lo, hi = LIM[j]
            if not (lo-0.5 <= math.degrees(v) <= hi+0.5): limviol += 1
        if prev is not None:
            diffs = [abs(math.degrees(q[i]-prev[i])) for i in range(6)]
            if max(diffs) > jjerk:
                jjerk = max(diffs); worstj = f'j{diffs.index(max(diffs))+1}'
        prev = q
        if not valid(q): bad += 1
    print('j4 by phase:  ' + '  '.join(f'{int(f*100)}%={j4c[int(f*(N-1))]:.0f}' for f in (0,.25,.5,.75,1)))
    print('j5 by phase:  ' + '  '.join(f'{int(f*100)}%={j5c[int(f*(N-1))]:.0f}' for f in (0,.25,.5,.75,1)))
    print(f'aim to {target_name}: mean {np.mean(aims):.0f} worst {max(aims):.0f} deg')
    print(f'max joint step: {jjerk:.1f} deg ({worstj})   duration {times[-1]:.1f}s')
    print(f'limit violations: {limviol}   collision-invalid: {bad}/{N}')

    d['points'] = [{'positions': [float(v) for v in q],
                    'velocities': [0.0]*6, 'accelerations': [0.0]*6,
                    'time_from_start': round(t, 4)}
                   for q, t in zip(newpts, times)]
    d['note'] = f'{target_name}: j4 turns mid-rise ({J4_A:.0%}-{J4_B:.0%}), j5 looks at {target_name} (down on descent).'
    with open(out, 'w') as f:
        yaml.safe_dump(d, f, sort_keys=False, default_flow_style=None)
    print(f'\nwrote {out}')
    return 0 if (bad == 0 and limviol == 0) else 2


if __name__ == '__main__':
    sys.exit(main())
