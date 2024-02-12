# cartographer_utils

A set of utilities for ROS 2 cartographer package.

## Installation

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to cartographer_utils
```

## Pose Twist Fusion
A node for fusing pose and twist data from different sources. It may be useful for localization pipelines, where
odometry topic type is required, since cartographer ROS 2 API provides only pose topic as an output.

### Usage

```bash
ros2 launch cartographer_initializer cartographer_initializer.launch.py configuration_directory:=/path/to/dir configuration_basename:=filename.lua
```

### API

#### Input

| Name                    | Type                                           | Description                        |
| ----------------------- | ---------------------------------------------- | ---------------------------------- |
| `pose`                  | geometry_msgs::msg::PoseStamped                | Input pose topic.                  |
| `pose_with_covariance`  | geometry_msgs::msg::PoseWithCovarianceStamped  | Input pose with covariance topic.  |
| `twist`                 | geometry_msgs::msg::TwistStamped               | Input twist topic.                 |
| `twist_with_covariance` | geometry_msgs::msg::TwistWithCovarianceStamped | Input twist with covariance topic. |


#### Output

| Name       | Type                    | Description            |
| ---------- | ----------------------- | ---------------------- |
| `odometry` | nav_msgs::msg::Odometry | Output odometry topic. |


#### Parameters

| Name                 | Type     | Description                                                    |
| -------------------- | -------- | -------------------------------------------------------------- |
| `odom_rate`          | double   | Output odometry rate.                                          |
| `frame_id`           | str      | Output odometry frame id.                                      |
| `child_frame_id`     | str      | Output odometry child frame id.                                |
| `pose.is_with_cov`   | bool     | Wheter input pose topic is WithCovariance msg type.            |
| `pose.override_cov`  | bool     | Wheter to override input pose topic covariance (if contains).  |
| `pose.cov`           | list[36] | Covariance matrix for output pose of odometry topic.           |
| `twist.is_with_cov`  | bool     | Wheter input twist topic is WithCovariance msg type.           |
| `twist.override_cov` | bool     | Wheter to override input twist topic covariance (if contains). |
| `twist.cov`          | list[36] | Covariance matrix for output twist of odometry topic.          |

## Pose Initializer
A node for setting up vehicle ego pose via `/initialpose` (rviz tool) topic using cartographer ROS 2 API.

### Usage

```bash
ros2 launch cartographer_utils pose_initializer.launch.py configuration_directory:=/path/to/dir configuration_basename:=filename.lua
```

### API

#### Input

| Name           | Type                                          | Description                                |
| -------------- | --------------------------------------------- | ------------------------------------------ |
| `/initialpose` | geometry_msgs::msg::PoseWithCovarianceStamped | Requested vehicle ego pose from rviz tool. |

#### Output

| Name                    | Type                                            | Description                                         |
| ----------------------- | ----------------------------------------------- | --------------------------------------------------- |
| `get_trajectory_states` | cartographer_ros_msgs::srv::GetTrajectoryStates | Service to get trajectory states from cartographer. |
| `finish_trajectory`     | cartographer_ros_msgs::srv::FinishTrajectory    | Service to finish cartographer's trajectory.        |
| `start_trajectory`      | cartographer_ros_msgs::srv::StartTrajectory     | Service to start cartographer's trajectory.         |

#### Parameters

| Name                      | Type | Description                   |
| ------------------------- | ---- | ----------------------------- |
| `configuration_directory` | str  | Path to lua config directory. |
| `configuration_basename`  | str  | File name of lua config.      |


## References / External links
<!-- Optional -->
* [Cartographer ROS 2](https://github.com/ros2/cartographer_ros)