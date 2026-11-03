#!/usr/bin/env python3
# Hairoom action bridge v2 — official Fairino XML-RPC (port 20003) edition.
#
# 2026-07-16: complete rewrite. The old bridge drove the robot with per-joint
# StartJOG over the web UI's HTTP endpoint + websocket:9999 feedback, because
# every coordinated-motion path looked dead on this firmware. That conclusion
# was wrong: the official SDK's XML-RPC command channel on port 20003 IS open
# on this controller (V3.9.5-QX / web v3.9.1) and its MoveJ moves all six
# joints together, verified on hardware 2026-07-16 (J1+J6 simultaneous move,
# landed exactly on target). What is genuinely closed is port 20005 (the SDK
# state channel), which is why libfairino's RPC() — and therefore the official
# fairino_hardware ros2_control plugin, every version 2.1.4..2.3.5 — hangs and
# can never work here without a firmware upgrade. So this bridge speaks the
# same protocol the official SDK does, minus the dead port:
#   commands:  XML-RPC http://<robot>:20003/RPC2  (MoveJ / StopMotion / ...)
#   feedback:  XML-RPC GetActualJointPosDegree polling -> /joint_states
# No web login, no cookies, no websocket. requests/websocket deps gone.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import xmlrpc.client
import http.client
import math, time, threading

ROBOT_IP = '192.168.58.2'
RPC_URL = f'http://{ROBOT_IP}:20003/RPC2'

JOINT_NAMES = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']

# Per-joint (lower, upper) limits in degrees, from fairino_description's
# fairino5_v6.urdf <limit> tags. j2/j4 are mechanically asymmetric.
JOINT_LIMITS_DEG = {
    1: (-175.0, 175.0),
    2: (-265.1, 85.0),
    3: (-162.0, 162.0),
    4: (-265.1, 85.0),
    5: (-175.0, 175.0),
    6: (-175.0, 175.0),
}
JOINT_LIMIT_MARGIN_DEG = 3.0

MOVE_SPEED_PCT = 10.0        # keep 5-10% while the workcell is experimental
BLEND_MS = 150.0             # smoothing between waypoints, 0-500 ms
# 2026-07-16: the controller's SDK service (ports 20003/20004/8083) died once
# ~35 min into a 20Hz polling session — whole service, not just our socket
# (new connections refused until controller reboot). Cause unproven, but this
# QX firmware's SDK service is clearly fragile, so poll gently and close dead
# sockets properly instead of abandoning them in CLOSE-WAIT.
STATE_POLL_HZ = 10.0
# MoveJ with blendT=-1 blocks until the motion physically completes, so the
# HTTP socket for the command proxy must outlive the longest plausible move.
MOTION_TIMEOUT_S = 180.0
STATE_TIMEOUT_S = 3.0
# Execute the MoveIt trajectory as a sequence of blocking MoveJ waypoints.
# Denser than ~8deg adds stop-and-go jerk without buying collision fidelity
# at hairoom's speeds; the final point is always executed exactly.
WAYPOINT_MIN_STEP_DEG = 8.0
WAYPOINT_MAX_COUNT = 12


class _TimeoutTransport(xmlrpc.client.Transport):
    def __init__(self, timeout):
        super().__init__()
        self._timeout = timeout

    def make_connection(self, host):
        conn = http.client.HTTPConnection(host, timeout=self._timeout)
        self._connection = host, conn
        return conn


def make_proxy(timeout):
    """One ServerProxy per thread — xmlrpc.client proxies are not thread-safe."""
    return xmlrpc.client.ServerProxy(RPC_URL, transport=_TimeoutTransport(timeout))


class HairoomActionBridge(Node):
    def __init__(self):
        super().__init__('hairoom_action_bridge')
        self.real_joints = None
        self.cancel_requested = False
        self.lock = threading.Lock()
        self.motion_lock = threading.Lock()   # serialize command-proxy use
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.cmd = make_proxy(MOTION_TIMEOUT_S)     # executor thread only
        self.stop_proxy = make_proxy(STATE_TIMEOUT_S)  # cancel thread only

        try:
            err, *ver = self.cmd.GetSoftwareVersion()
            self.get_logger().info(f'✅ XML-RPC 20003 connected, robot: {ver}')
        except Exception as e:
            self.get_logger().error(f'❌ XML-RPC connection to {RPC_URL} failed: {e!r}')

        for call, args in (('Mode', (0,)), ('RobotEnable', (1,)), ('SetSpeed', (100,))):
            try:
                rtn = getattr(self.cmd, call)(*args)
                self.get_logger().info(f'✅ {call}{args} -> {rtn}')
            except Exception as e:
                self.get_logger().error(f'{call}{args} failed: {e!r}')

        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            '/fairino5_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        t = threading.Thread(target=self.state_loop, daemon=True)
        t.start()
        self.get_logger().info('✅ XML-RPC MoveJ Bridge готов!')

    def goal_callback(self, goal_request):
        self.cancel_requested = False
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.cancel_requested = True
        try:
            self.stop_proxy.StopMotion()
        except Exception as e:
            self.get_logger().error(f'StopMotion failed: {e!r}')
        return CancelResponse.ACCEPT

    # ---------------- feedback ----------------

    def state_loop(self):
        """Poll real joint positions over XML-RPC -> /joint_states.
        Keeps publishing during a blocking MoveJ (own proxy, own socket),
        so RViz shows live motion."""
        proxy = make_proxy(STATE_TIMEOUT_S)
        period = 1.0 / STATE_POLL_HZ
        warned = False
        while True:
            t0 = time.monotonic()
            try:
                res = proxy.GetActualJointPosDegree(1)
                if res[0] == 0:
                    with self.lock:
                        self.real_joints = list(res[1:7])
                    self.publish_joint_states()
                    warned = False
            except Exception as e:
                if not warned:
                    self.get_logger().warn(f'joint feedback poll failed: {e!r} — retrying')
                    warned = True
                proxy = make_proxy(STATE_TIMEOUT_S)
                time.sleep(1.0)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def get_joints(self):
        with self.lock:
            return list(self.real_joints) if self.real_joints else None

    def publish_joint_states(self):
        joints = self.get_joints()
        if joints is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [math.radians(v) for v in joints]
        self.joint_state_pub.publish(msg)

    # ---------------- motion ----------------

    def movej(self, target_deg, blend_ms):
        """Blocking coordinated MoveJ to target (degrees). Returns robot error
        code (0 = reached). Needs the cartesian pose too — get it from the
        controller's own forward kinematics so the pair can't disagree."""
        with self.motion_lock:
            fk = self.cmd.GetForwardKin([float(v) for v in target_deg])
            if fk[0] != 0:
                return fk[0]
            return self.cmd.MoveJ(
                [float(v) for v in target_deg], fk[1:7],
                0, 0,                       # tool, workpiece
                float(MOVE_SPEED_PCT), 0.0, 100.0,
                [0.0, 0.0, 0.0, 0.0],       # external axes
                float(blend_ms),            # -1.0=block, 0..500=smooth ms
                0, [0.0] * 6)               # no offset

    def pick_waypoints(self, trajectory):
        """Downsample the planned path: keep waypoints at least
        WAYPOINT_MIN_STEP_DEG apart (max joint delta), always keep the final
        point. Preserves the planned (collision-checked) route shape without
        stop-and-go on every densely-sampled point."""
        name_idx = [list(trajectory.joint_names).index(n) for n in JOINT_NAMES]
        pts = [[math.degrees(p.positions[i]) for i in name_idx]
               for p in trajectory.points]
        picked = []
        last = pts[0]
        for p in pts[1:-1]:
            if max(abs(a - b) for a, b in zip(p, last)) >= WAYPOINT_MIN_STEP_DEG:
                picked.append(p)
                last = p
        while len(picked) > WAYPOINT_MAX_COUNT:
            picked = picked[::2]
        picked.append(pts[-1])
        return picked

    def execute_callback(self, goal_handle):
        self.get_logger().info('🎯 Цель получена!')
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        if not trajectory.points:
            goal_handle.succeed()
            return result

        if set(trajectory.joint_names) != set(JOINT_NAMES):
            self.get_logger().error(
                f'❌ trajectory.joint_names {list(trajectory.joint_names)} does not '
                f'match expected {JOINT_NAMES} — aborting to avoid commanding the wrong joint')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f'unexpected joint_names: {list(trajectory.joint_names)}'
            return result

        waypoints = self.pick_waypoints(trajectory)
        target = waypoints[-1]
        self.get_logger().info(
            f'Цель: J1={target[0]:.1f}° J2={target[1]:.1f}° J3={target[2]:.1f}° '
            f'J4={target[3]:.1f}° J5={target[4]:.1f}° J6={target[5]:.1f}° '
            f'({len(waypoints)} waypoint(s), MoveJ @ {MOVE_SPEED_PCT:.0f}%)')

        # Refuse the whole move if any waypoint is close to a hardware limit.
        for wp in waypoints:
            for i, t in enumerate(wp):
                lower, upper = JOINT_LIMITS_DEG[i + 1]
                if not (lower + JOINT_LIMIT_MARGIN_DEG <= t <= upper - JOINT_LIMIT_MARGIN_DEG):
                    self.get_logger().error(
                        f'J{i+1}: waypoint {t:.1f}° is within {JOINT_LIMIT_MARGIN_DEG:.1f}° of '
                        f'the [{lower:.1f}, {upper:.1f}]° joint limit — refusing the whole goal')
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = f'J{i+1} waypoint exceeds safe joint limit'
                    return result

        for n, wp in enumerate(waypoints, 1):
            if self.cancel_requested:
                goal_handle.canceled()
                return result
            is_last = (n == len(waypoints))
            try:
                rtn = self.movej(wp, -1.0 if is_last else BLEND_MS)
            except Exception as e:
                self.get_logger().error(f'MoveJ RPC error on waypoint {n}: {e!r}')
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f'MoveJ RPC error: {e}'
                return result
            if rtn != 0:
                if self.cancel_requested:
                    goal_handle.canceled()
                    return result
                self.get_logger().error(f'❌ MoveJ waypoint {n} returned error code {rtn}')
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f'robot MoveJ error code {rtn}'
                return result
            self.get_logger().info(f'waypoint {n}/{len(waypoints)} ✅')

        time.sleep(0.3)  # let the last feedback poll catch the resting pose
        cur = self.get_joints()
        if cur is not None:
            worst = max(abs(a - b) for a, b in zip(cur, target))
            self.get_logger().info(
                f'final pose error (max joint): {worst:.2f}° — '
                + ' '.join(f'J{i+1}={v:.2f}°' for i, v in enumerate(cur)))

        self.get_logger().info('✅ Цель достигнута!')
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = HairoomActionBridge()
    # Multi-threaded so cancel_callback can run while execute_callback is
    # parked inside a blocking MoveJ.
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
