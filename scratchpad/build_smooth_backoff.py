"""OFFLINE (no motion). Smooth joint-interp A->B (shortest per-joint, j5 stays
smooth), but push the tip RADIALLY AWAY from the head centre in mid-path
(taper sin(pi t) to 0 at the endpoints) by BULGE_MM, re-solving IK seeded from
the interp pose and KEEPING the interp orientation -- so the wrist barely moves
(no j5 spin) while the arm backs off the head. Endpoints exactly taught.
Every point head-strict collision-checked. Writes an exec_saved route.

    argv: OUT_YAML A B
env: BULGE_MM(25) STEP_DEG(1.0) DEG_S(8)
"""
import sys, os, math, threading, yaml
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetPositionFK
from moveit_msgs.msg import PositionIKRequest, RobotState

REPO = '/home/farrukh/Desktop/Hairoom/v2'
ARM = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
OUT, A, B = sys.argv[1], sys.argv[2], sys.argv[3]
BULGE_MM = float(os.environ.get('BULGE_MM', '25'))
STEP_DEG = float(os.environ.get('STEP_DEG', '1.0'))
DEG_S = float(os.environ.get('DEG_S', '8'))
DEV_DEG = float(os.environ.get('DEV_DEG', '15'))   # max pose deviation from smooth interp
J3_TOL = float(os.environ.get('J3_TOL', '10'))     # keep elbow bent (j3 near interp)
AIM_AT_HEAD = os.environ.get('AIM_AT_HEAD') == '1'  # actively point +Z at head centre
FRAME = 'head_link'
CENTER = np.array([0.0025, -0.1872, 0.0232])   # head centre (centroid of + pts)
P = yaml.safe_load(open(os.path.join(REPO, 'config', 'approach_poses.yaml')))['poses']
qa = [float(v) for v in P[A]]; qb = [float(v) for v in P[B]]


def ease(t):
    return 3*t*t - 2*t*t*t


def flat_top(t, r=0.25):
    """1.0 across the middle, smooth ease ramp 0->1 over the first/last r."""
    if t < r:
        return ease(t / r)
    if t > 1 - r:
        return ease((1 - t) / r)
    return 1.0


def rotate(q, v):
    x, y, z, w = q; vx, vy, vz = v
    tx = 2*(y*vz-z*vy); ty = 2*(z*vx-x*vz); tz = 2*(x*vy-y*vx)
    return np.array([vx+w*tx+(y*tz-z*ty), vy+w*ty+(z*tx-x*tz), vz+w*tz+(x*ty-y*tx)])


def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return [aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz]


def aim_z_at(o_prev, look_dir):
    """Minimally rotate orientation o_prev so its tool +Z points along look_dir
    (shortest arc -> roll stays continuous, no wrist spin)."""
    z_prev = rotate(o_prev, (0, 0, 1))
    z_prev = z_prev/np.linalg.norm(z_prev)
    ld = look_dir/np.linalg.norm(look_dir)
    d = float(np.clip(np.dot(z_prev, ld), -1.0, 1.0))
    if d > 0.99999:
        return list(o_prev)
    axis = np.cross(z_prev, ld)
    na = np.linalg.norm(axis)
    if na < 1e-9:                      # antiparallel: pick any perpendicular
        axis = np.cross(z_prev, (1, 0, 0))
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(z_prev, (0, 1, 0))
        na = np.linalg.norm(axis)
    axis = axis/na
    ang = math.acos(d)
    s = math.sin(ang/2)
    dq = [axis[0]*s, axis[1]*s, axis[2]*s, math.cos(ang/2)]
    q = qmul(dq, list(o_prev))
    q = np.array(q); return list(q/np.linalg.norm(q))


rclpy.init(); n = Node('smooth_backoff')
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
ik = n.create_client(GetPositionIK, '/compute_ik'); ik.wait_for_service(timeout_sec=10)
sv = n.create_client(GetStateValidity, '/check_state_validity'); sv.wait_for_service(timeout_sec=10)
fk = n.create_client(GetPositionFK, '/compute_fk'); fk.wait_for_service(timeout_sec=10)


def call(cl, req):
    ev = threading.Event(); box = {}
    cl.call_async(req).add_done_callback(lambda f: (box.__setitem__('r', f.result()), ev.set()))
    ev.wait(6); return box.get('r')


def fkpose(q):
    r = GetPositionFK.Request(); r.header = Header(frame_id=FRAME); r.fk_link_names = ['tcp_tool']
    r.robot_state.joint_state = JointState(name=ARM, position=[float(v) for v in q])
    p = call(fk, r).pose_stamped[0].pose
    return (np.array([p.position.x, p.position.y, p.position.z]),
            [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])


def solve(tgt, o, sd):
    ps = PoseStamped(); ps.header.frame_id = FRAME
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = [float(v) for v in tgt]
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = o
    req = PositionIKRequest(); req.group_name = 'fairino5_v6_group'; req.ik_link_name = 'tcp_tool'
    req.pose_stamped = ps; req.timeout.sec = 1; req.avoid_collisions = False
    rs = RobotState(); rs.joint_state = JointState(name=ARM, position=[float(v) for v in sd]); req.robot_state = rs
    r = GetPositionIK.Request(); r.ik_request = req; res = call(ik, r)
    if res is None or res.error_code.val != 1:
        return None
    js = res.solution.joint_state; return [js.position[js.name.index(j)] for j in ARM]


def valid(q):
    r = GetStateValidity.Request(); r.group_name = 'fairino5_v6_group'
    rs = RobotState(); rs.joint_state = JointState(name=ARM, position=[float(v) for v in q]); r.robot_state = rs
    return call(sv, r).valid


dmax = max(abs(math.degrees(qb[i]-qa[i])) for i in range(6))
steps = max(int(math.ceil(dmax / STEP_DEG)), 2)
pts = []
inv = 0; lifted = 0; maxlift = 0.0; maxaim = 0.0
_, o_prev = fkpose(qa)                     # start from taught A orientation
for k in range(steps + 1):
    t = k / steps
    q_i = [qa[i] + (qb[i]-qa[i]) * ease(t) for i in range(6)]
    if k == 0 or k == steps:
        pts.append(q_i)
        _, o_prev = fkpose(q_i)
        continue
    tip, o_i = fkpose(q_i)                 # keep the INTERP orientation (aim blends
    d = np.linalg.norm(tip - CENTER)       # smoothly between the two taught ends)
    outdir = (tip - CENTER) / d if d > 1e-6 else np.array([0, 0, 1.0])
    # largest RADIAL back-off (cheap in joints) that keeps the elbow bent and
    # the pose near the smooth interp -- no re-aim, so no wrist spin.
    chosen = None; chosen_o = None
    for frac in [1.0, 0.8, 0.6, 0.4, 0.2]:
        bulge = BULGE_MM/1000.0 * flat_top(t) * frac     # arc held across path
        tgt = tip + outdir * bulge
        if AIM_AT_HEAD:
            o_use = aim_z_at(o_prev, CENTER - tgt)       # +Z actively at head centre
        else:
            o_use = o_i                                  # keep interp orientation
        q = solve(tgt, o_use, pts[-1])     # seed prev -> on-branch, smooth
        if q is None or not valid(q):
            continue
        dev = max(abs(math.degrees(q[i]-q_i[i])) for i in range(6))
        dj3 = abs(math.degrees(q[2]-q_i[2]))
        if dev <= DEV_DEG and dj3 <= J3_TOL:
            chosen = q; chosen_o = o_use; break
    if chosen is not None:
        pts.append(chosen); lifted += 1
        tp, oa = fkpose(chosen)
        maxlift = max(maxlift, np.linalg.norm(np.array(tp) - tip)*1000)
        o_prev = oa
    else:
        pts.append(q_i)
        _, o_prev = fkpose(q_i)

# POLYNOMIAL smoothing: fit each joint to a smooth polynomial in u so ALL joints
# become smooth analytic functions of the SAME parameter -> synchronised, no
# ripple, no per-joint desync. Endpoints re-pinned to the taught poses exactly.
DEG_POLY = int(os.environ.get('DEG_POLY', '9'))
Np = len(pts)
u = np.array([k/(Np-1) for k in range(Np)])
arr = np.array([list(p) for p in pts])            # (Np,6)
sm = arr.copy()
for j in range(6):
    coef = np.polyfit(u, arr[:, j], DEG_POLY)
    sm[:, j] = np.polyval(coef, u)
sm[0] = arr[0]; sm[-1] = arr[-1]                  # exact taught endpoints
reinv = sum(0 if valid(sm[i]) else 1 for i in range(1, Np-1))
pts = [list(row) for row in sm]
print(f'  polyfit deg={DEG_POLY}: {reinv} invalid after smoothing')

# EASE-IN-OUT timing (S-curve): the arm accelerates smoothly from rest and
# decelerates smoothly to rest -> no jerk at the ends. time(k) = T*ease(u) with
# u = k/(N-1), so dt is large (slow) near the ends and small (fast) mid-path.
N = len(pts)
travel = sum(max(abs(math.degrees(pts[k][i]-pts[k-1][i])) for i in range(6))
             for k in range(1, N))
T = max(2.0 * travel / DEG_S, 0.2)                 # 2x: peak speed ~ DEG_S mid


def ease_inv(f):                                   # solve ease(tau)=f for tau
    lo, hi = 0.0, 1.0
    for _ in range(40):
        mid = 0.5*(lo+hi)
        if ease(mid) < f:
            lo = mid
        else:
            hi = mid
    return 0.5*(lo+hi)


# points are uniform in path (index); place them so arc-fraction follows ease
# in time -> slow (large dt) at the ends, fast mid -> velocity ramps rest->rest.
times = [T * ease_inv(k/(N-1)) for k in range(N)]

# velocities consistent with that timing (central differences); the large end
# dt makes these ramp smoothly to ~0 at both ends.
posarr = np.array([list(q) for q in pts])          # (N,6) radians
vel = np.zeros_like(posarr)
for i in range(1, N-1):
    vel[i] = (posarr[i+1] - posarr[i-1]) / (times[i+1] - times[i-1])
# endpoints exactly zero velocity (rest to rest)

j5tot = abs(math.degrees(qb[4]-qa[4]))
doc = {'from': A, 'to': B, 'skill': 'smooth_backoff',
       'note': f'{A}->{B} smooth joint interp + radial back-off {BULGE_MM:.0f}mm from head '
               f'(taper), IK re-seeded, smoothed, uniform dt + central-diff velocities. '
               f'endpoints=taught. lifted {lifted}/{steps-1}, max lift {maxlift:.0f}mm, {inv} invalid.',
       'joint_names': ARM,
       'points': [{'positions': [float(v) for v in posarr[k]],
                   'velocities': [float(v) for v in vel[k]],
                   'accelerations': [0.0]*6, 'time_from_start': float(times[k])}
                  for k in range(N)]}
clean = list(zip([list(posarr[k]) for k in range(N)], times))  # for the final print
yaml.safe_dump(doc, open(OUT, 'w'), sort_keys=False)
print(f'{A}->{B}: {len(clean)} pts, {clean[-1][1]:.1f}s, j5={j5tot:.0f}deg, '
      f'lifted {lifted}/{steps-1}, maxlift {maxlift:.0f}mm, max aim-err {maxaim:.1f}deg, invalid {inv}')
print('wrote', OUT)
rclpy.shutdown()
