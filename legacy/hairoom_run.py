#!/usr/bin/env python3
"""Executes saved landmarks as a sequence through MoveIt — real motion.

Usage: python3 hairoom_run.py A B C ...

For each named landmark, in order:
  1. Transform its pose from head_link into the planning frame (base_link).
  2. PLAN (collision-checked, plan_only — nothing moves yet) a MoveGroup
     goal targeting tcp_tool at that pose.
  3. Print a plan summary and WAIT for Enter before doing anything else.
     Ctrl+C at this point aborts with zero motion.
  4. Re-check the robot's actual current joint state against the plan's
     first waypoint. If the robot moved since planning (pendant jog,
     anything else), abort this point rather than execute a trajectory
     that no longer starts where the robot actually is.
  5. Execute *that exact planned trajectory* via /execute_trajectory —
     never a fresh plan-and-execute call, so what you approved is exactly
     what runs.

Every motion goes through MoveIt's collision-checked planning pipeline —
this script never sends raw joint/servo commands. Velocity/acceleration
scaling is hard-capped at MAX_VELOCITY_SCALING_CAP regardless of
VELOCITY_SCALING below. If any point fails to plan, fails re-validation, or
fails to execute, the sequence stops immediately — remaining points are not
attempted. Ctrl+C at any time cancels whatever goal is currently active
(plan or execute) before exiting.
"""
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
import tf2_geometry_msgs  # noqa: F401 -- registers PoseStamped with Buffer.transform()
from sensor_msgs.msg import JointState

sys.path.insert(0, '/home/farrukh/hairoom_ws')
from hairoom_teach import load_landmarks, HEAD_FRAME, LANDMARKS_PATH  # noqa: E402

GROUP = 'fairino5_v6_group'
TOOL_LINK = 'tcp_tool'
PLANNING_FRAME = 'base_link'
VELOCITY_SCALING = 0.05
ACCEL_SCALING = 0.05
MAX_VELOCITY_SCALING_CAP = 0.1   # hard ceiling, independent of the constants above
POSITION_TOLERANCE_M = 0.01
ORIENTATION_TOLERANCE_RAD = 0.1
STARTUP_TF_TIMEOUT_S = 8.0
JOINT_MISMATCH_TOLERANCE_RAD = 0.01
TF_LOOKUP_TIMEOUT_S = 2.0


class HairoomRun(Node):
    def __init__(self):
        super().__init__('hairoom_run')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.active_goal_handle = None
        self.latest_joint_state = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self.latest_joint_state = msg

    def verify_tf_ready(self):
        try:
            self.buffer.can_transform(
                PLANNING_FRAME, HEAD_FRAME, rclpy.time.Time(),
                timeout=Duration(seconds=STARTUP_TF_TIMEOUT_S))
            self.buffer.can_transform(
                PLANNING_FRAME, TOOL_LINK, rclpy.time.Time(),
                timeout=Duration(seconds=STARTUP_TF_TIMEOUT_S))
            return True
        except Exception as e:
            self.get_logger().error(f'TF not ready: {e}')
            return False

    def transform_to_planning_frame(self, position, orientation):
        ps = PoseStamped()
        ps.header.frame_id = HEAD_FRAME
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = position
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = orientation
        return self.buffer.transform(ps, PLANNING_FRAME, timeout=Duration(seconds=TF_LOOKUP_TIMEOUT_S))

    def build_plan_goal(self, pose_stamped):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        vel = min(VELOCITY_SCALING, MAX_VELOCITY_SCALING_CAP)
        acc = min(ACCEL_SCALING, MAX_VELOCITY_SCALING_CAP)
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.start_state.is_diff = True

        pc = PositionConstraint()
        pc.header.frame_id = pose_stamped.header.frame_id
        pc.link_name = TOOL_LINK
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [POSITION_TOLERANCE_M]
        bv = BoundingVolume()
        bv.primitives.append(prim)
        bv.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = pose_stamped.header.frame_id
        oc.link_name = TOOL_LINK
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        oc.absolute_y_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        oc.absolute_z_axis_tolerance = ORIENTATION_TOLERANCE_RAD
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        return goal

    def send_and_wait(self, client, goal):
        """Sends a goal, tracks it as the active goal (for Ctrl+C cancellation),
        blocks until result. Returns the result, or None if rejected."""
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return None
        self.active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        self.active_goal_handle = None
        result = result_future.result()
        return result.result if result is not None else None

    def cancel_active_goal(self):
        if self.active_goal_handle is not None:
            print('Cancelling active goal...')
            try:
                cancel_future = self.active_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
            except Exception as e:
                print(f'  (cancel request error, robot should still stop via action server: {e})')
            self.active_goal_handle = None


def error_name(val):
    from moveit_msgs.msg import MoveItErrorCodes
    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == val:
            return name
    return str(val)


def summarize_plan(name, trajectory, planning_time):
    points = trajectory.joint_trajectory.points
    duration_s = 0.0
    if points:
        d = points[-1].time_from_start
        duration_s = d.sec + d.nanosec * 1e-9
    print(f"  Plan for '{name}': {len(points)} waypoints, "
          f"~{duration_s:.2f}s estimated duration, {planning_time:.3f}s planning time.")


def check_start_state(node, trajectory, tolerance):
    names = trajectory.joint_trajectory.joint_names
    if not trajectory.joint_trajectory.points:
        return False, 'planned trajectory has no points'
    first = trajectory.joint_trajectory.points[0]
    current_msg = node.latest_joint_state
    if current_msg is None:
        return False, 'no /joint_states received yet'
    current = dict(zip(current_msg.name, current_msg.position))
    diffs = []
    for jname, planned_pos in zip(names, first.positions):
        if jname not in current:
            return False, f"joint '{jname}' missing from /joint_states"
        diff = abs(current[jname] - planned_pos)
        if diff > tolerance:
            diffs.append(f'{jname}: planned={planned_pos:.4f} actual={current[jname]:.4f} diff={diff:.4f}')
    if diffs:
        return False, 'robot moved since planning: ' + '; '.join(diffs)
    return True, None


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python3 {sys.argv[0]} NAME [NAME ...]")
        sys.exit(1)
    requested = sys.argv[1:]

    data = load_landmarks()
    by_name = {lm['name']: lm for lm in data['landmarks']}
    missing = [n for n in requested if n not in by_name]
    if missing:
        print(f"ERROR: unknown landmark name(s): {', '.join(missing)}")
        print(f"Loaded from {LANDMARKS_PATH}: {', '.join(by_name) if by_name else '(none)'}")
        sys.exit(1)

    rclpy.init()
    node = HairoomRun()

    if not node.verify_tf_ready():
        print(f"ERROR: TF frames '{HEAD_FRAME}' and/or '{TOOL_LINK}' (in {PLANNING_FRAME}) "
              "are not available. Nothing sent.")
        rclpy.shutdown()
        sys.exit(1)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    if not node.move_client.wait_for_server(timeout_sec=10.0):
        print('ERROR: /move_action server not available.')
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        sys.exit(1)
    if not node.exec_client.wait_for_server(timeout_sec=10.0):
        print('ERROR: /execute_trajectory server not available.')
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        sys.exit(1)

    exit_code = 0
    try:
        for name in requested:
            lm = by_name[name]
            print(f"\n--- {name} ({lm.get('region', '')}) ---")

            try:
                pose_stamped = node.transform_to_planning_frame(lm['position'], lm['orientation'])
            except Exception as e:
                print(f"FAILED at '{name}': could not transform pose from {HEAD_FRAME} "
                      f"to {PLANNING_FRAME}: {e}")
                exit_code = 1
                break

            plan_goal = node.build_plan_goal(pose_stamped)
            plan_result = node.send_and_wait(node.move_client, plan_goal)
            if plan_result is None or plan_result.error_code.val != 1:
                code = error_name(plan_result.error_code.val) if plan_result else 'goal rejected'
                print(f"FAILED at '{name}': plan did not succeed ({code}). Stopping sequence.")
                exit_code = 1
                break

            summarize_plan(name, plan_result.planned_trajectory, plan_result.planning_time)
            try:
                input(f"  Press Enter to execute this plan for '{name}' (Ctrl+C to abort): ")
            except (EOFError, KeyboardInterrupt):
                print(f"\nAborted before executing '{name}'. Nothing moved for this point.")
                exit_code = 1
                break

            ok, reason = check_start_state(node, plan_result.planned_trajectory,
                                            JOINT_MISMATCH_TOLERANCE_RAD)
            if not ok:
                print(f"FAILED at '{name}': {reason}. Plan is stale, not executing. "
                      "Stopping sequence — re-run to re-plan from the current position.")
                exit_code = 1
                break

            exec_goal = ExecuteTrajectory.Goal()
            exec_goal.trajectory = plan_result.planned_trajectory
            exec_result = node.send_and_wait(node.exec_client, exec_goal)
            if exec_result is None or exec_result.error_code.val != 1:
                code = error_name(exec_result.error_code.val) if exec_result else 'goal rejected'
                print(f"FAILED at '{name}': execution did not succeed ({code}). Stopping sequence.")
                exit_code = 1
                break

            print(f"Moving to {name}... done")

    except KeyboardInterrupt:
        print("\nCtrl+C received.")
        node.cancel_active_goal()
        exit_code = 1
    except Exception as e:
        print(f"\nERROR: {e}")
        node.cancel_active_goal()
        exit_code = 1

    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
