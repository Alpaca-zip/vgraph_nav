# vgraph_nav [![ROS-noetic Industrial CI](https://github.com/Alpaca-zip/vgraph_nav/actions/workflows/ci.yml/badge.svg)](https://github.com/Alpaca-zip/vgraph_nav/actions/workflows/ci.yml)
ROS package for a simple navigation system utilizing the Visibility Graph algorithm.  
It is compatible with occupancy grid maps generated by popular 2D SLAM such as [Cartographer](https://github.com/cartographer-project/cartographer_ros), [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox), and [slam_gmapping](https://github.com/ros-perception/slam_gmapping/blob/melodic-devel/gmapping/src/slam_gmapping.cpp)

<img src="https://github.com/Alpaca-zip/vgraph_nav/assets/84959376/0660bebb-f53d-4791-8978-7e13569291fc" width="500px">

## Setup ⚙
```bash
$ cd {ROS_WORKSPACE}/src
$ git clone -b noetic-devel https://github.com/Alpaca-zip/vgraph_nav.git
$ wstool merge vgraph_nav/.rosinstall
$ wstool update
$ python3 -m pip install -r vgraph_nav/requirements.txt
$ rosdep install -r -y -i --from-paths .
$ catkin build
```
## Run 🚀
Example of navigation using [TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3).
```
$ roslaunch vgraph_nav turtlebot3_vgraph_navigation.launch
```
### 1. 2D Pose Estimate
Before starting navigation, you can define the starting point of the robot.  
This is achieved using the <kbd>2D Pose Estimate</kbd> button on the RViz.

More info : [e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#estimate-initial-pose:~:text=Estimate%20Initial%20Pose)

### 2. 2D Nav Goal
After setting the initial pose, the next step is to specify the target position using <kbd>2D Nav Goal</kbd>.

More info : [e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/nav_simulation/#:~:text=nodes%20during%20Navigation.-,Set%20Navigation%20Goal,-Click%20the%202D)

**NOTE**: The first time after launch, optimization using the Visibility Graph algorithm takes few minutes.

## vgraph_planner_node 🤖
### Params
- `~map_file`: Path to the configuration file (yaml) for the occupancy grid map.
- `~test_folder`: A debug folder primarily used for debugging the Visibility Graph algorithm.
- `~down_scale_factor`: A factor for downscaling the occupancy grid map. Please reduce this value if optimization is taking too long.
- `~clearance`: The required clearance for the robot to navigate. It has to be at least half the maximum width of the robot. This is also used for obstacle detection.
- `~odom_topic`: The name of the odometry topic.
- `~scan_topic`: The name of the LaserScan topic.
- `~vel_linear`: Linear velocity.
- `~vel_theta`: Maximum angular velocity.
- `~angle_tolerance`: During navigation, the robot will attempt to align its direction with the goal if the angle difference exceeds this tolerance.
- `~goal_tolerance`: Distance to a waypoint at which it is considered reached.
- `~verbose`: If True, the results of the optimization are displayed.

### Topics
- Subscribed Topics:
  - `/initialpose` ([geometry_msgs/PoseWithCovarianceStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)) : Receives the robot's initial pose from RViz's <kbd>2D Pose Estimate</kbd>
  - `/move_base_simple/goal` ([geometry_msgs/PoseStamped](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseStamped.html)) : Receives navigation goals from RViz's <kbd>2D Nav Goal</kbd>
  - `/odom` ([nav_msgs/Odometry](https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html)) : Provides the robot's odometry data. Can be remapped via `~odom_topic`.
  - `/scan` ([sensor_msgs/LaserScan](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)) : Provides the 2D Lidar data of the robot. Can be remapped via `~scan_topic`.

- Published Topics:
  - `/cost_map` ([nav_msgs/OccupancyGrid](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html)) : Generates a path along the obstacles in this occupancy grid map. Mainly for visualization in RViz.
  - `/path` ([nav_msgs/Path](https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Path.html)) : The path that the robot will follow. Mainly for visualization in RViz.
  - `/cmd_vel` ([geometry_msgs/Twist](https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)) : Conveys the target velocity data of the robot.
