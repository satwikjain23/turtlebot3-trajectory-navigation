# Path Smoothing and Trajectory Control in 2D Space
This project implements a complete navigation pipeline for a differential-drive robot (TurtleBot3) using ROS2.

The assignment consists of **three main tasks:**

**1) Path Smoothing** 
Convert discrete waypoints into a smooth, continuous trajectory using B-splines.

**2) Trajectory Generation**
Time-parameterize the smoothed path and publish a ROS2 trajectory for control.

**3)Trajectory Tracking Controller**
Use a Pure Pursuit controller to compute /cmd_vel and follow the trajectory.

![demo](./video_90_deg.gif)

## How to Build & Run

### Install dependencies
```bash
sudo apt update
sudo apt install ros-humble-desktop ros-humble-turtlebot3* python3-scipy

```

### Create workspace
```bash
mkdir -p ~/assignment_ws/src
cd ~/assignment_ws/src
git clone https://github.com/satwikjain23/turtlebot3-trajectory-navigation.git
```

### Build
```bash
cd ~/assignment_ws
colcon build --symlink-install
source install/setup.bash
```

### Run TurtleBot3 Simulation
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Run the assignment pipeline
```bash
ros2 launch navigation_assignment all_nodes.launch.py
```

### Start RViz Visualization
```bash
rviz2
```
>Add the following topics:
>
>**/robot_path**
>
>**/visualized_trajectory**

## Test Automation
This repository includes automated test scripts for validating the **path smoothing** and **trajectory generation** algorithms.

### Run All Tests
From your workspace root:
```bash
bash src/run_all_tests.sh
```
All output plots are saved in:
```bash
test_results/
```
## Obstacle Avoidance
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 run navigation_assignment path_smoothing_node 
ros2 run navigation_assignment trajectory_generator_node 
ros2 run navigation_assignment trajectory_visualizer 
ros2 run navigation_assignment obstacle_avoidance 
```
![demo2](./video_obstacle_avoidance.gif)
