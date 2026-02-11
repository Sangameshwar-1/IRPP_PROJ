#!/bin/bash

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Define workspace and project paths
WORKSPACE_DIR=~/ros2_ws
PROJECT_SOURCE_DIR=/home/sangam/Documents/Acad/sem-4/IRPP/PROJ/scan_project/human_models/ros_humans_ros2

# Check if workspace exists, if not create it
if [ ! -d "$WORKSPACE_DIR/src" ]; then
    echo "Creating workspace directory..."
    mkdir -p $WORKSPACE_DIR/src
fi

# Copy project to workspace if it's not already linked or there
TARGET_DIR=$WORKSPACE_DIR/src/ros_humans_ros2
if [ ! -d "$TARGET_DIR" ]; then
    echo "Copying project to workspace..."
    # We use cp -r to copy the project. 
    # Adjust the source path specifically to where the package.xml is located.
    # Based on previous context: /home/sangam/Documents/Acad/sem-4/IRPP/PROJ/scan_project/human_models/ros_humans_ros2
    cp -r $PROJECT_SOURCE_DIR $WORKSPACE_DIR/src/
else
    echo "Project already in workspace."
fi

# Build the workspace
echo "Building the workspace..."
cd $WORKSPACE_DIR
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation
echo "Launching the simulation with complex_grid_map..."
ros2 launch ros_humans_ros2 demo.launch.py
