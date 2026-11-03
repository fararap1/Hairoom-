#!/bin/bash
# Bring up the Hairoom stack on the OFFICIAL ros2_control driver path.
#
# This is not the old start_hairoom.sh: it starts no bridge at all. The robot
# is driven by fairino_hardware/FairinoHardwareInterface under
# controller_manager. See docs/architecture.md.
#
# Usage:
#   ./bin/start_hairoom.sh                     # headless (no RViz)
#   ./bin/start_hairoom.sh --rviz              # with RViz, hairoom layout
#   ./bin/start_hairoom.sh --rviz use_scene:=false   # extra launch args pass through
set -u

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${HAIROOM_WS:-$HOME/hairoom_ws}"
ROBOT_IP="${HAIROOM_ROBOT_IP:-192.168.58.2}"
NIC="${HAIROOM_NIC:-enp46s0}"

# The saved layout: MotionPlanning + the /hairoom_zone_markers display the
# show_*.py scripts publish to. Without it RViz opens the stock moveit.rviz and
# the landmarks are invisible until you add the display by hand.
RVIZ_CFG="${HAIROOM_RVIZ_CONFIG:-$REPO_ROOT/config/hairoom.rviz}"

USE_RVIZ=false
if [ "${1:-}" = "--rviz" ]; then
    USE_RVIZ=true
    shift
fi
# Anything left over is handed to ros2 launch as-is (name:=value), so a
# one-off override does not need this script edited.
EXTRA_ARGS=("$@")

echo "==> stopping anything already holding the robot"
"$REPO_ROOT/bin/stop_hairoom.sh"

# Laptop's address on the robot's subnet. Harmless if it is already set.
sudo -n ip addr add 192.168.58.100/24 dev "$NIC" 2>/dev/null

echo "==> checking the robot answers"
if ! ping -c1 -W2 "$ROBOT_IP" >/dev/null 2>&1; then
    echo "ERROR: $ROBOT_IP does not answer. Robot powered on? NIC $NIC configured?"
    exit 1
fi
# 20003 is the XML-RPC command channel; 20005 is the SDK state channel the
# official driver's RPC() needs. Both must be open or the driver hangs.
for port in 20003 20005; do
    if ! timeout 3 bash -c "cat < /dev/null > /dev/tcp/$ROBOT_IP/$port" 2>/dev/null; then
        echo "ERROR: port $port closed on $ROBOT_IP — the official driver cannot start."
        echo "       Check the pendant is in Remote mode and there are no alarms."
        exit 1
    fi
done
echo "    $ROBOT_IP reachable, ports 20003 + 20005 open"

# ROS's setup scripts read unset variables, which trips `set -u`.
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

# --- DDS transport: UDPv4 only (no shared memory) ---
# Fast DDS's SHM transport intermittently stalls action goal-response delivery
# on localhost -> move_group "Failed to send goal response (timeout)" and the
# first /execute hangs (retry then works). A UDP-only participant profile fixes
# it at the root. Respect an already-set profile; clear stale SHM segments.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$REPO_ROOT/config/fastdds_profile.xml}"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "    DDS: $RMW_IMPLEMENTATION, profile $(basename "$FASTRTPS_DEFAULT_PROFILES_FILE") (UDPv4-only)"

echo "==> launching (use_rviz:=$USE_RVIZ, rviz_config:=$RVIZ_CFG)"
nohup ros2 launch fairino5_v6_moveit2_config hairoom_bringup.launch.py \
      use_rviz:="$USE_RVIZ" rviz_config:="$RVIZ_CFG" \
      ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} > "$WS/hairoom_bringup.log" 2>&1 &
echo "    pid $!, log: $WS/hairoom_bringup.log"

echo "==> waiting for the driver and controllers"
for _ in $(seq 1 40); do
    if timeout 5 ros2 control list_controllers 2>/dev/null | grep -q "fairino5_controller.*active"; then
        echo "    controllers active"
        ros2 control list_controllers
        echo
        # LAW: the sink mesh must never block the arm. Re-arm the sink-ACM on
        # every bringup (ACM resets on restart). Head stays strict.
        echo "==> applying sink-ACM (sink collision-free, head strict)"
        sleep 3   # let hairoom_scene.py add the scene first
        python3 "$REPO_ROOT/scripts/apply_sink_acm.py" || \
            echo "    WARN: sink-ACM failed — rerun scripts/apply_sink_acm.py"
        echo
        echo "Ready. Sanity checks:"
        echo "  ros2 topic echo /joint_states --once"
        echo "  python3 $REPO_ROOT/scripts/check_goals.py     # read-only"
        exit 0
    fi
    sleep 2
done

echo "ERROR: controllers did not come up in time. See $WS/hairoom_bringup.log"
exit 1
