# ROS 2 Room Exploration Robot (oda_kesif)

Autonomous frontier-based exploration for a mobile robot in ROS 2 Jazzy + Gazebo, using Nav2 for navigation. Written step by step while learning.

## How it works

1. **Map** — subscribes to the SLAM occupancy grid (`/map`).
2. **Frontiers** (`oda_kesif/sinirlar.py`, pure NumPy, no ROS) — free cells next to unknown cells, grouped into clusters with BFS, filtered by size and distance from obstacles (dilation).
3. **Goal selection** (`oda_kesif/kesif.py`) — the closest frontier to the robot (robot pose from TF `map → base_link`).
4. **Navigation** — the goal is sent to Nav2 with `nav2_simple_commander`; progress is logged until arrival.

## Status

- [x] Frontier detection + unit tests (`test/test_sinirlar.py`)
- [x] Pick the best frontier and drive there with Nav2 (step 4c)
- [ ] Explore in a loop until no frontiers remain
- [ ] Camera: detect a red target and save its position on the map
- [ ] Second run: go directly to the saved target

## Build & run

```bash
cd ~/kesif_ws
colcon build --packages-select oda_kesif
source install/setup.bash
ros2 run oda_kesif kesif
```

Tests:

```bash
colcon test --packages-select oda_kesif && colcon test-result --verbose
```
