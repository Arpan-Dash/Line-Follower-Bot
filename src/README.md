# Line Follower Robot (ROS2 Jazzy + Gazebo + Camera)

A camera-based line follower robot that tracks a black line on a floor. The robot uses a downward-tilted camera, grayscale thresholding, and a PD controller to follow the line autonomously.

## How it works
1. A custom differential drive robot with a downward-facing camera is spawned in an indoor room with a tiled floor
2. A black line track is painted on the floor in a smooth closed loop (generated procedurally)
3. The camera captures the floor in front of the robot
4. The `line_follower` node:
   - Crops the bottom 35% of the camera image (where the line is closest)
   - Converts to grayscale and thresholds dark pixels (the line)
   - Computes the centroid x-offset from image center
   - Applies a filtered PD controller: `angular = -Kp * error - Kd * derivative`
   - Runs at 0.24 m/s on straight sections and slows progressively before turns
5. If the line is lost, the robot spins in the last known direction to re-acquire it
6. RViz displays the live camera feed so you can see what the robot sees
7. A live `Line Follower Error` window plots the signed deviation from the
   camera center. Values beyond ±20 pixels are marked as turns.

While following the line, the node reports the tracking deviation in its log:

- `deviation`: the current distance between the line and camera center, in pixels
- the percentage in parentheses: deviation relative to half the camera view
- `avg`: average absolute deviation since the node started
- `max`: largest absolute deviation seen since the node started

For example, `deviation=12px (4.7%)` means the detected line is 12 pixels
away from the center of the camera view. These are image measurements, not
physical centimeters; converting them to distance requires camera calibration.

The graph is started automatically by the launch file after the line follower.
The blue line is the current tracking error, red points indicate turns, and
the green band represents small error near the center line. The graph keeps
the most recent 600 samples.

## Requirements
- Ubuntu 24.04 + ROS 2 Jazzy
- Gazebo (gz-sim, ships with `ros-jazzy-ros-gz`)
- The following ROS packages:
  ```bash
  sudo apt update
  sudo apt install -y \
      ros-jazzy-ros-gz \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-rviz2
  ```

## Build

```bash
# 1. Create a workspace (skip if you already have one)
mkdir -p ~/line_follower_ws/src
cd ~/line_follower_ws/src

# 2. Clone this repo
git clone https://github.com/Ahmed-m-abbas/ros2_line_follower.git

# 3. Build the package
cd ~/line_follower_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ros2_line_follower --symlink-install

# 4. Source the workspace
source ~/line_follower_ws/install/setup.bash
```

## Run

```bash
bash run.sh
# or: ros2 launch ros2_line_follower line_follower.launch.py
```

This launches everything in one command:
- Gazebo with the indoor room world
- The robot on the line track
- ROS-Gazebo bridges (camera, cmd_vel, odom, scan, tf)
- RViz with camera image display
- The line follower node (after 8s delay for startup)

## Project layout
```
ros2_line_follower/
├── package.xml / setup.py / setup.cfg
├── resource/ros2_line_follower
├── urdf/my_robot.urdf                          # custom diff drive robot + camera + lidar
├── worlds/indoor_line_track.sdf                # world with the final grading track
├── models/meshes/final_grading.stl             # final grading track mesh
├── launch/line_follower.launch.py              # full launch (Gazebo + robot + RViz + follower)
├── config/line_follower.rviz                   # RViz config with camera display
├── ros2_line_follower_pkg/
│   └── line_follower.py                        # PD controller line tracking node
├── scripts/create_floor_texture.py             # generates the tiled floor + line track
├── run.sh
```

## Tech
- ROS 2 Jazzy
- URDF (custom differential drive robot)
- Gazebo Harmonic (gz-sim) + camera sensor + `DiffDrive` plugin
- `ros_gz_bridge` (Clock, Twist, Odometry, LaserScan, Image, CameraInfo)
- PD controller (numpy-based image processing, no OpenCV dependency)
- RViz2 (camera image display)
- PIL/Pillow (procedural floor texture generation)

