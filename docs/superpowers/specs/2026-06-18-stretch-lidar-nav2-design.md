# Stretch Lidar + SLAM/Nav2 Design

Date: 2026-06-18
Status: Approved

## Goal

Add a 2D lidar to the Stretch robot in `examples/apartment.ipynb` and provide a
ROS 2 (Jazzy) Nav2 environment that does **full autonomous navigation with live
SLAM** (`slam_toolbox`). The Nav2/SLAM stack auto-launches from the notebook.
The ROS workspace build is baked into `binder/Dockerfile`.

## TF tree

SLAM requires `odom → base_link` instead of the current ground-truth
`map → base_link`. `slam_toolbox` supplies `map → odom`.

```
map  →  odom          slam_toolbox (live from /scan)
odom →  base_link     sim node (ground-truth pose as perfect odometry)
base_link → laser     static, from URDF joint_laser (xyz 0.004 0 0.1664, yaw pi)
base_link → <links>   sim node (unchanged)
```

`laser` is a fixed link merged out of the articulation on URDF import, so it does
not appear in `body_names`; the node publishes `base_link → laser` explicitly.

## A. examples/apartment.ipynb

1. New section "Add a 2D lidar (RTX)": enable `isaacsim.sensors.rtx`, create an
   RTX lidar prim under `/World/stretch/base_link` at the `joint_laser` offset,
   configured like an RPLidar A1 (360°, ~10 Hz, ~12 m). Attach the flat-scan
   annotator to read ranges/intensities each frame.
2. Extend `StretchROS`:
   - `/scan` (`sensor_msgs/LaserScan`, `frame_id=laser`) from the annotator.
   - `/odom` (`nav_msgs/Odometry`) — Nav2 controller reads the odom topic.
   - Base TF parent changed `map` → `odom`.
   - Explicit static `base_link → laser` TF from the URDF offset.
3. Sim loop calls `publish_scan()` + `publish_odom()` each iteration.
4. New auto-launch cell (subprocess, same pattern as the rviz2 cell) sources ROS
   + `nav2_ws/install` and runs `ros2 launch stretch_nav2 slam_nav2.launch.py`.
   RViz cell switched to `nav2.rviz`.

## B. nav2_ws/ (colcon workspace at repo root)

```
nav2_ws/src/stretch_nav2/
  package.xml, CMakeLists.txt        ament_cmake; installs launch/config/rviz
  launch/slam_nav2.launch.py         slam_toolbox (online_async) + nav2_bringup
  config/nav2_params.yaml            diff-drive; frames map/odom/base_link;
                                     scan /scan, odom /odom, cmd_vel→/stretch/cmd_vel
  config/slam_toolbox.yaml           online async, /scan, odom frame, base_link
  rviz/nav2.rviz                     scan + costmaps + robot model + 2D Goal Pose
```

Launch remaps Nav2 `/cmd_vel` → `/stretch/cmd_vel` so goals drive the existing
wheel controller.

## C. binder/Dockerfile

- `sudo apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox`
- After the repo is copied in:
  `cd ${REPO_DIR}/nav2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install`
