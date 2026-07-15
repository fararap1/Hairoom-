# Hairoom

Minimal ROS2/MoveIt2 control surface for the Hairoom head-care chair.

## Current behavior

`POST /wash`:

1. Loads the 12 joint waypoints from `data/points.csv`.
2. Adds `head_centered.stl` and `sink_centered.stl` to the MoveIt2 planning scene.
3. Plans and executes every waypoint as a joint-space MoveIt2 target.
4. Plans and executes a final move to `base`.

The XYZ values in `data/points.csv` are treated as reference metadata only. The
actual command targets are the `j1..j6` joint angles.

## Environment setup

Target machine:

- Ubuntu 24.04 + ROS2 Jazzy, or Ubuntu 22.04 + ROS2 Humble if the Fairino stack
  you use is pinned to Humble.
- ROS2 and MoveIt2 must be installed before starting this server.
- The Fairino FR5 MoveIt2 stack must work in RViz before calling `POST /wash`.

Example for ROS2 Jazzy:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-moveit \
  ros-jazzy-moveit-py \
  python3-rosdep \
  python3-colcon-common-extensions \
  python3-vcstool
```

Initialize `rosdep` once per machine:

```bash
sudo rosdep init
rosdep update
```

For every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
```

Clone the Fairino dependencies into this ROS2 workspace:

```bash
cd /Users/ivanlysov/Projects/Hairoom-
git clone https://github.com/FAIR-INNOVATION/frcobot_ros2.git src/frcobot_ros2
git clone https://github.com/FAIR-INNOVATION/fairino-cpp-sdk.git src/fairino-cpp-sdk
```

Install package dependencies:

```bash
cd /Users/ivanlysov/Projects/Hairoom-
rosdep install --from-paths src --ignore-src -r -y
```

Build the workspace:

```bash
cd /Users/ivanlysov/Projects/Hairoom-
colcon build
source install/setup.bash
```

Before running the HTTP server, verify the Fairino FR5 stack directly through
MoveIt2/RViz. The expected Fairino package name may vary by release, but for FR5
look for `fairino5_v6_moveit2_config` inside `src/frcobot_ros2`.

Useful inspection commands:

```bash
ros2 pkg list | grep -i fairino
ros2 pkg list | grep -i moveit
find src/frcobot_ros2 -maxdepth 3 -type f -name '*launch.py'
```

## ROS2 package

Package:

```text
src/hairoom_control
```

Build from the workspace root on the Ubuntu robot computer:

```bash
colcon build --packages-select hairoom_control
source install/setup.bash
```

Run:

```bash
ros2 launch hairoom_control wash_server.launch.py \
  planning_group:=fr5_arm \
  joint_names:=j1,j2,j3,j4,j5,j6 \
  base_degrees:=0,0,0,0,0,0
```

Call:

```bash
curl -X POST http://localhost:8080/wash
```

Health check:

```bash
curl http://localhost:8080/health
```

## Required integration values

These defaults are placeholders until the FR5 MoveIt config is connected:

- `planning_group`: MoveIt planning group name, default `fr5_arm`.
- `joint_names`: MoveIt joint names matching the six columns in `data/points.csv`.
- `base_degrees`: safe return pose after washing.
- `controllers`: optional comma-separated controller names passed to MoveIt2 execution.

The server expects the FR5 MoveIt2 configuration to provide `robot_description`,
`robot_description_semantic`, controller config, joint states, and a working
planning pipeline.
