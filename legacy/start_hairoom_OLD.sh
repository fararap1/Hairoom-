#!/bin/bash
echo "🤖 Starting Hairoom..."

# Always start from a clean slate: kill any previous bridge/RViz/scene
# processes before launching new ones. Makes this script safe to re-run anytime.
~/stop_hairoom.sh

# Fix network for robot (only needed if the laptop lost its 192.168.58.x addr)
sudo -n ip addr add 192.168.58.100/24 dev enp46s0 2>/dev/null

# Bridge: MoveIt Plan&Execute -> real robot via official Fairino XML-RPC
# (port 20003, coordinated MoveJ), also polls real joint feedback -> /joint_states.
# No websocket/cookies — the robot's web UI in a browser can stay open alongside.
gnome-terminal --title="Action Bridge" -- bash -c "source ~/.bashrc && python3 ~/hairoom_ws/src/hairoom_action_bridge.py 2>&1 | tee ~/hairoom_ws/hairoom_action_bridge.log; exec bash"

# Launch RViz2 + MoveIt2
gnome-terminal --title="RViz2" -- bash -c "source ~/.bashrc && ros2 launch fairino5_v6_moveit2_config demo.launch.py 2>&1 | tee ~/hairoom_ws/rviz_moveit.log; exec bash"

# Wait for MoveIt2 to start
sleep 8

# Load head + sink scene
gnome-terminal --title="Hairoom Scene" -- bash -c "source ~/.bashrc && python3 ~/hairoom_ws/src/hairoom_scene.py 2>&1 | tee ~/hairoom_ws/hairoom_scene.log && echo '✅ Head + Sink loaded!' && exec bash"

echo "✅ Hairoom started!"
