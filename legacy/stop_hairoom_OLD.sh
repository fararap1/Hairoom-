#!/bin/bash
# Stops the Hairoom stack (bridge + RViz/MoveIt launch tree + scene loader).
#
# 2026-07-16: the bridge now talks official XML-RPC on 192.168.58.2:20003 and
# no longer owns websocket:9999 or the web session, so this script kills by
# process name only. It must NOT kill by connection: the user's browser on the
# robot's web UI (port 80/9999) can and should coexist with the bridge now.
#
# Usage: ./stop_hairoom.sh
set -u

# Catch any known bridge script by name (current and historical ones).
for pattern in hairoom_action_bridge.py ws_bridge.py fairino_bridge.py fairino_follow_bridge.py \
               fairino_http_bridge.py final_bridge.py jog_bridge.py servoj_bridge.py smart_jog_bridge.py; do
    if pkill -f "$pattern" 2>/dev/null; then
        echo "  killed stray $pattern process(es)"
    fi
done

# Also kill any leftover RViz2/MoveIt2 launch tree and scene loader. Killing
# only the individual children (move_group/rviz2/robot_state_publisher)
# isn't enough — `ros2 launch` is a supervisor that respawns children that
# die out from under it. Killing the launch process itself makes it tear
# down its whole managed tree cleanly. Without this, re-running
# start_hairoom.sh leaves an orphaned old launch tree running alongside the
# new one (two move_group nodes fighting over the same robot).
if pkill -f 'ros2 launch fairino5_v6_moveit2_config' 2>/dev/null; then
    echo "  killed stray RViz2/MoveIt2 launch tree(s)"
    sleep 1
fi
if pkill -f 'hairoom_scene.py' 2>/dev/null; then
    echo "  killed stray hairoom_scene.py process(es)"
fi

echo "Done."
