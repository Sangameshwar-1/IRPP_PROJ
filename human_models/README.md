# Person 1 Deliverables: World + Humans + /human_pose

## Files
- Gazebo world: ../gazebo_worlds/indoor_with_humans.world
- ROS package: ros_humans

## What it does
- Spawns static and moving humans (as cylinder models) in Gazebo world.
- Publishes `/human_pose` (PoseArray) and TF frames for each `human_*` model.
- Moves `human_moving_*` models along waypoint loops using `/gazebo/set_model_state`.

## Build (catkin)
```
cd ~/catkin_ws/src
ln -s /home/sangam/Documents/Acad/sem-4/IRPP/PROJ/scan_project/human_models/ros_humans ros_humans
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run
1) Launch Gazebo with the world:
```
roslaunch gazebo_ros empty_world.launch world_name:=/home/sangam/Documents/Acad/sem-4/IRPP/PROJ/scan_project/gazebo_worlds/indoor_with_humans.world
```

2) Start human motion:
```
roslaunch ros_humans move_humans.launch
```

3) Start human pose publisher:
```
roslaunch ros_humans human_pose_publisher.launch
```

## Topic
- `/human_pose` (geometry_msgs/PoseArray)
- TF frames: `human_static_1`, `human_static_2`, `human_moving_1`, `human_moving_2`
