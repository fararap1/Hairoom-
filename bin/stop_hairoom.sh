#!/bin/bash
# Tear down the Hairoom stack.
#
# Kills the launch supervisor first: `ros2 launch` respawns children that die
# under it, so killing move_group/ros2_control_node individually is not enough.
#
# Also sweeps up orphans by name. Real cases seen on this machine:
#   - a robot_state_publisher outliving its killed launch tree, then fighting
#     the next run over /robot_description and /tf;
#   - two hairoom_head_tf.py (or two show_landmarks.py) publishing at once.
#
# SELF-SAFETY: every match is done with `pgrep -f` (whole command line), which
# also matches the CALLER's shell (and forked subshells of a pipeline) when its
# command line happens to contain the pattern text — e.g. an operator running
# `pkill -x rviz2` by hand, or this being invoked from a one-liner that mentions
# "rviz2"/"head_tf". A bare `pkill -f rviz2` then kills that shell (seen as
# exit 144, a signal to the shell). Guarding by process group was not enough (a
# pipeline subshell can land in a different pgid). The robust rule: NEVER kill a
# process whose comm is a shell. Every real stack process is python3 / rviz2 /
# ros2_control_node / move_group / robot_state_publisher — none are shells,
# while every caller of this script is bash. So skipping shells can never spare
# a real target and can never kill the operator's shell.
set -u

# kill_pat PATTERN LABEL — kill every process whose full command line matches
# PATTERN, except $$ and any process whose comm is a shell (or already gone).
kill_pat() {
    local pat="$1" label="$2" hit=0 pid comm
    for pid in $(pgrep -f "$pat" 2>/dev/null); do
        [ "$pid" = "$$" ] && continue
        comm=$(ps -o comm= -p "$pid" 2>/dev/null)
        case "$comm" in
            bash|sh|dash|zsh|ksh|fish|"") continue ;;  # never kill a shell / vanished pid
        esac
        kill "$pid" 2>/dev/null && hit=1
    done
    [ "$hit" = 1 ] && echo "  killed $label"
}

# The launch supervisor first (it respawns its children).
kill_pat 'ros2 launch fairino5_v6_moveit2_config' 'launch tree(s)' && sleep 2

# Long-running helper nodes started by the launch (or by hand).
for pat in hairoom_head_tf.py hairoom_scene.py show_landmarks.py \
           hairoom_action_bridge.py; do
    kill_pat "$pat" "stray $pat"
done

# Only orphans remain by now — the launch tree above is already gone.
for pat in ros2_control_node moveit_ros_move_group robot_state_publisher rviz2; do
    kill_pat "$pat" "orphaned $pat"
done

sleep 1
leftover=$(pgrep -af 'ros2_control_node|move_group|robot_state_publisher|rviz2|hairoom_|show_landmarks' 2>/dev/null \
           | grep -v 'stop_hairoom' | grep -vw "$$" | wc -l)
if [ "$leftover" -gt 0 ]; then
    echo "  WARNING: $leftover process(es) still alive:"
    pgrep -af 'ros2_control_node|move_group|robot_state_publisher|rviz2|hairoom_|show_landmarks' \
        | grep -v 'stop_hairoom' | grep -vw "$$"
else
    echo "  clean"
fi
