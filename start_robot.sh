#!/bin/bash

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Function to run a command in a new terminal
run_in_terminal() {
    title=$1
    cmd=$2
    gnome-terminal --title="$title" -- bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && $cmd; exec bash"
}

# Start the robot state publisher and controllers
run_in_terminal "Robot Control" "ros2 launch so_arm100 so_arm100.launch.py use_mock_hardware:=false"

# Wait a bit for the controllers to start
sleep 5

# Start MoveIt
run_in_terminal "MoveIt" "ros2 launch so_arm100_moveit_config move_group.launch.py"

# Start RViz
run_in_terminal "RViz" "ros2 launch so_arm100_moveit_config moveit_rviz.launch.py"
