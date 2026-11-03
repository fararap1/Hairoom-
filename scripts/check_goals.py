#!/usr/bin/env python3
"""Read-only: for each landmark, solve IK then ask MoveIt whether that goal
state is collision-free. Distinguishes "the goal itself is inside the head"
from "the approach path happened to clip it". Nothing moves."""
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from moveit_msgs.msg import RobotState, PositionIKRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf2_ros
import tf2_geometry_msgs  # noqa: F401

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hairoom_teach import load_landmarks, HEAD_FRAME  # noqa: E402

GROUP = 'fairino5_v6_group'
TOOL_LINK = 'tcp_tool'
PLANNING_FRAME = 'base_link'


class C(Node):
    def __init__(self):
        super().__init__('hairoom_check_goals')
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.sv = self.create_client(GetStateValidity, '/check_state_validity')
        self.js = None
        self.create_subscription(JointState, '/joint_states', self._js, 10)

    def _js(self, m):
        self.js = m

    def call(self, client, req, timeout=15.0):
        done = threading.Event()
        box = {}

        def cb(f):
            box['r'] = f.result()
            done.set()
        client.call_async(req).add_done_callback(cb)
        return box.get('r') if done.wait(timeout) else None


def main():
    data = load_landmarks()
    rclpy.init()
    n = C()
    threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()

    n.buf.can_transform(PLANNING_FRAME, HEAD_FRAME, rclpy.time.Time(),
                        timeout=Duration(seconds=8.0))
    n.ik.wait_for_service(timeout_sec=10.0)
    n.sv.wait_for_service(timeout_sec=10.0)
    while n.js is None:
        pass

    print(f'{"name":<8}{"region":<20}{"IK":<14}{"goal state":<16}{"colliding pair"}')
    print('-' * 88)
    for lm in data['landmarks']:
        ps = PoseStamped()
        ps.header.frame_id = HEAD_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = lm['position']
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = lm['orientation']
        tgt = n.buf.transform(ps, PLANNING_FRAME, timeout=Duration(seconds=2.0))

        ikreq = GetPositionIK.Request()
        r = PositionIKRequest()
        r.group_name = GROUP
        r.ik_link_name = TOOL_LINK
        r.pose_stamped = tgt
        r.timeout.sec = 3
        r.avoid_collisions = False      # solve pure kinematics, judge separately
        rs = RobotState()
        rs.joint_state = n.js
        r.robot_state = rs
        ikreq.ik_request = r
        ikres = n.call(n.ik, ikreq)

        if ikres is None or ikres.error_code.val != 1:
            code = 'no response' if ikres is None else str(ikres.error_code.val)
            print(f'{lm["name"]:<8}{lm.get("region",""):<20}{"FAIL "+code:<14}{"-":<16}')
            continue

        svreq = GetStateValidity.Request()
        svreq.group_name = GROUP
        svreq.robot_state.joint_state = ikres.solution.joint_state
        svres = n.call(n.sv, svreq)
        if svres is None:
            print(f'{lm["name"]:<8}{lm.get("region",""):<20}{"ok":<14}{"no response":<16}')
            continue
        pairs = {f'{c.contact_body_1}~{c.contact_body_2}' for c in svres.contacts}
        verdict = 'VALID' if svres.valid else 'IN COLLISION'
        print(f'{lm["name"]:<8}{lm.get("region",""):<20}{"ok":<14}{verdict:<16}'
              f'{", ".join(sorted(pairs)) if pairs else ""}')
    return 0


if __name__ == '__main__':
    rc = main()
    import os
    sys.stdout.flush()
    os._exit(rc)
