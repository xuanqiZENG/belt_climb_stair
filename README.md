# NMPC+WBC deployed on Sirius
## Installation
Please refer to [here](README_original.md).
## Start
1. Run the simulation with selected robot type and z_initial_pos in gazebo:
```
roslaunch legged_controllers zero_shot_start.launch robot_type:=sirius_mini real_robot:=false z_pos:=0.3
```
Robot| Recommended z_pos       |
|------------|----------------|
sirius_mini|0.3|
sirius_belt|0.6|
sirius_mid |0.6|

Or on the robot hardware:

```
roslaunch legged_controllers zero_shot_start.launch robot_type:=sirius_mini real_robot:=true
```
2. Make sure that gazebo/real robot is online.
3. Load and start the controller:

```
roslaunch legged_controllers load_controller.launch robot_type:=sirius_mini use_js:=true
```
4. Joystick 


Button| Robot behavior       |
|------------|----------------|
LB|stand up|
LB+Left stick|forward/backward/left/right|

5. For Velodyne VLP16 Lidar simulation in gazebo
- Download and build Lidar ROS package: [VelodyneSim](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/).
-   use `add_laser:=true` when doing step1.
---
6. For Velodyne VLP16 driver(real laser)
- Download and build Velodyne ROS package.
- Start the velodyne driver
```bash
roslaunch velodyne_pointcloud VLP16_points.launch 
```


## Trouble Shooting
### Deploy on ROS melodic
The code is originally designed for ROS noetic. But it can also be deployed on ROS melodic through minor changes.
| ROS version        | publishFixedTransforms|publishTransforms |
|------------|----------------|---------|
|Melodic | `publishFixedTransforms`(const std::string &`tf_prefix`, bool use_tf_static=false)| `publishTransforms`(const std::map<std::string, double > &joint_positions, const ros::Time &time, const std::string &`tf_prefix`)
|Noetic|`publishFixedTransforms`(bool use_tf_static=false) |`publishTransforms`(const std::map< std::string, double > &joint_positions, const ros::Time &time)

