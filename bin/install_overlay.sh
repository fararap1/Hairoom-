#!/bin/bash
# Copy this repo's moveit_config_overlay/ over the vendored frcobot_ros2
# package in the ROS 2 workspace, then rebuild.
#
# Why this exists: every file under moveit_config_overlay/ is a local change to
# a package that comes from an upstream git repo. An upstream pull silently
# reverts them (this happened on 2026-07-18 with the V3.9.5 update, which put
# the mock ros2_control stack back and broke every plan). Re-run this after any
# pull of frcobot_ros2. See docs/architecture.md.
set -eu

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${HAIROOM_WS:-$HOME/hairoom_ws}"
PKG="$WS/src/frcobot_ros2/fairino5_v6_moveit2_config"

[ -d "$PKG" ] || { echo "ERROR: $PKG not found. Set HAIROOM_WS."; exit 1; }

echo "==> installing overlay into $PKG"
for f in "$REPO_ROOT"/moveit_config_overlay/config/*; do
    [ -f "$f" ] || continue          # skip __pycache__ and friends
    echo "    config/$(basename "$f")"
    cp "$f" "$PKG/config/"
done
for f in "$REPO_ROOT"/moveit_config_overlay/launch/*; do
    [ -f "$f" ] || continue
    echo "    launch/$(basename "$f")"
    cp "$f" "$PKG/launch/"
done

# Only one fairino_hardware variant may build: they all register the same
# pluginlib class name, so a second one makes the build ambiguous.
echo "==> disabling non-active fairino_hardware variants"
for d in "$WS"/src/frcobot_ros2/fairino_hardware_v3_8_2_11 \
         "$WS"/src/frcobot_ros2/fairino_hardware_v3_9_0 \
         "$WS"/src/frcobot_ros2/fairino_hardware_v3_9_0_1 \
         "$WS"/src/frcobot_ros2/fairino_hardware_v3_9_4; do
    [ -d "$d" ] && touch "$d/COLCON_IGNORE" && echo "    COLCON_IGNORE $(basename "$d")"
done

echo "==> building"
cd "$WS"
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --packages-select fairino5_v6_moveit2_config fairino_hardware_v3_9_5

echo
echo "Done. Now: source $WS/install/setup.bash && $REPO_ROOT/bin/start_hairoom.sh"
