#!/usr/bin/env bash
# DRAG-VIEW bring-up — RViz visualization for hand drag-teaching, WITHOUT the
# real driver. Starts robot_state_publisher + move_group + head/sink scene +
# head TF + landmarks, and mirrors the REAL joints into /joint_states over
# XML-RPC (scripts/drag_view.py, read-only). The ros2_control driver is
# deliberately NOT started -- it would ServoJ and stale (错误码:14) the instant
# you drag the arm by hand. Stop with bin/stop_hairoom.sh (kills move_group,
# rviz, head_tf, landmarks) plus pkill -f drag_view.py.
# NB: no `set -u` -- ROS's setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES)
# and would abort sourcing under nounset.
set -o pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/hairoom_ws/install/setup.bash"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$REPO/config/fastdds_profile.xml}"
PKG=fairino5_v6_moveit2_config

echo "==> safety: the real driver must NOT be running (it would ServoJ during drag)"
# Match the actually-launched node ('ros2_control_node --ros-args'), not any
# stray shell whose command line merely contains the string 'ros2_control_node'
# (a bare `pgrep -f ros2_control_node` false-positives on this script's own
# helper shells). Also refuse if a live socket to the robot is still open.
if pgrep -f 'ros2_control_node --ros-args' >/dev/null 2>&1 \
   || ss -tn state established 2>/dev/null | grep -qE '192\.168\.58\.2:(20003|20004|20005)'; then
  echo "    REFUSING: the real driver (ros2_control_node) or a live robot socket is up."
  echo "    Run ./bin/stop_hairoom.sh first."
  exit 1
fi
echo "    clear -- no real driver, no robot socket"

echo "==> mirroring real joints -> /joint_states (read-only XML-RPC)"
python3 "$REPO/scripts/drag_view.py" &
echo "==> robot model + move_group (planning scene for the head/sink)"
ros2 launch "$PKG" static_virtual_joint_tfs.launch.py >/dev/null 2>&1 &
ros2 launch "$PKG" rsp.launch.py >/dev/null 2>&1 &
ros2 launch "$PKG" move_group.launch.py >/dev/null 2>&1 &

sleep 7
echo "==> head + sink scene, head TF, landmark points"
python3 "$REPO/scripts/hairoom_scene.py" || true      # one-shot
python3 "$REPO/scripts/hairoom_head_tf.py" >/dev/null 2>&1 &
python3 "$REPO/scripts/show_landmarks.py" >/dev/null 2>&1 &

# LAW: the sink mesh must never block/flag the arm. Head stays strict.
echo "==> applying sink-ACM (sink collision-free, head strict)"
python3 "$REPO/scripts/apply_sink_acm.py" || \
    echo "    WARN: sink-ACM failed — rerun scripts/apply_sink_acm.py"

echo "Ready. Drag the arm by hand -- RViz follows the REAL joints."
echo "  RViz:  DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority LIBGL_ALWAYS_SOFTWARE=1 rviz2 -d config/hairoom.rviz"
echo "  Save:  python3 scripts/teach_save.py +NAME"
wait
