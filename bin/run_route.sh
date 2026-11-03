#!/bin/bash
# Play a saved route on the arm — one command, sources ROS for you.
#
#   ./bin/run_route.sh routes/hairdry/hairdry_spray.yaml         # PREVIEW in RViz (no motion)
#   ./bin/run_route.sh routes/hairdry/hairdry_spray.yaml --go    # RUN on the real arm
#
# The player (scripts/exec_saved.py):
#   - checks ROS matches the real robot (phantom guard),
#   - leads the arm to the route start if it is off-start,
#   - sends the trajectory straight to the controller (no false CONTROL_FAILED),
#   - verifies the arm REALLY moved (XML-RPC), then reports.
# NB: no `set -u` here — ROS setup.bash references unbound vars and would abort.
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROUTE="${1:-}"
GO="${2:-}"

if [ -z "$ROUTE" ]; then
    echo "usage: $0 <route.yaml> [--go]"
    echo "  no --go  = preview in RViz (nothing moves)"
    echo "  --go     = run on the real arm"
    exit 1
fi
if [ ! -f "$REPO/$ROUTE" ] && [ ! -f "$ROUTE" ]; then
    echo "route not found: $ROUTE"; exit 1
fi

source /opt/ros/humble/setup.bash 2>/dev/null
source "$HOME/hairoom_ws/install/setup.bash" 2>/dev/null

if [ "$GO" = "--go" ]; then
    echo "==> RUNNING $ROUTE on the arm"
    ROUTE="$ROUTE" EXEC=1 python3 "$REPO/scripts/exec_saved.py"
else
    echo "==> PREVIEW $ROUTE  (RViz /display_planned_path — nothing moves)"
    echo "    happy with it?  re-run with:  $0 $ROUTE --go"
    ROUTE="$ROUTE" python3 "$REPO/scripts/exec_saved.py"
fi
