#!/usr/bin/env python3
"""Hairoom backend — Layer 2: REST/WS API over the working ROS stack.

One process: FastAPI (uvicorn, main thread) + rclpy node (executor in a
background thread). Talks ONLY to ROS interfaces: MoveIt's MoveGroup action
(/move_action) for motion, /joint_states and TF for telemetry. Never opens a
socket to 192.168.58.x, never imports the Fairino SDK, never bypasses
move_group. hairoom_action_bridge.py is untouched. Nothing moves on startup —
motion only on an explicit /move_to.

Run:
    source /opt/ros/humble/setup.bash
    python3 backend/main.py
"""

import logging
import os
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import uvicorn
from fastapi import FastAPI

# ---------------- configuration (single place) ----------------

BIND_HOST = "0.0.0.0"
BIND_PORT = 8000

JOINTS_STALE_S = 1.0            # /joint_states older than this -> ros_ok False

# Motion limits (used from step 4 on; defined here so there is exactly one
# place for the cap). The hard cap is a REJECT threshold, not a clamp.
VELOCITY_SCALING = 0.05
ACCELERATION_SCALING = 0.05
VELOCITY_SCALING_HARD_CAP = 0.1

LOG_FILE = os.environ.get(
    "HAIROOM_BACKEND_LOG",
    os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                 "hairoom_backend.log"))

# ---------------- logging ----------------

log = logging.getLogger("hairoom_backend")
log.setLevel(logging.INFO)
_fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
for _h in (logging.FileHandler(LOG_FILE), logging.StreamHandler()):
    _h.setFormatter(_fmt)
    log.addHandler(_h)

# ---------------- ROS side ----------------


class HairoomBackendNode(Node):
    """Telemetry subscriptions. The MoveGroup action client arrives in the
    /stop step; motion goal construction in the /move_to step."""

    def __init__(self):
        super().__init__("hairoom_backend")
        self._joints_lock = threading.Lock()
        self._last_joints = None            # sensor_msgs/JointState
        self._last_joints_mono = None       # time.monotonic() of reception
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)

    def _on_joint_states(self, msg: JointState):
        with self._joints_lock:
            self._last_joints = msg
            self._last_joints_mono = time.monotonic()

    def joints_age_ms(self):
        """Milliseconds since the last /joint_states message, or None if
        none was ever received."""
        with self._joints_lock:
            if self._last_joints_mono is None:
                return None
            return (time.monotonic() - self._last_joints_mono) * 1000.0


# ---------------- HTTP side ----------------

app = FastAPI(title="Hairoom Backend", version="0.1")
node: HairoomBackendNode = None


@app.get("/status")
def get_status():
    age_ms = node.joints_age_ms()
    return {
        "ros_ok": age_ms is not None and age_ms < JOINTS_STALE_S * 1000.0,
        "moving": False,        # becomes real with the MoveGroup client
        "last_goal": None,
        "last_result": None,
        "joints_age_ms": None if age_ms is None else round(age_ms, 1),
    }


# ---------------- process wiring ----------------


def main():
    global node
    rclpy.init()
    node = HairoomBackendNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(
        target=executor.spin, daemon=True, name="rclpy-executor")
    spin_thread.start()

    log.info("backend up: rclpy spinning, HTTP on %s:%d", BIND_HOST, BIND_PORT)
    try:
        uvicorn.run(app, host=BIND_HOST, port=BIND_PORT, log_level="info")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
