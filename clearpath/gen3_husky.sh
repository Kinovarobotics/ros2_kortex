#!/bin/bash

# Run the Python script directly in the current terminal and wait for it to finish
python3 rl_robot_config_init_script.py
# Check if the Python script was successful
if [ $? -ne 0 ]; then
    echo "The robot has reached the initial configuration , proceeding"
fi
sleep 5
# After the Python script finishes, open a new terminal for each of the remaining commands
gnome-terminal -- bash -c "ros2 launch clearpath_gz simulation.launch.py; exec bash"
sleep 5
gnome-terminal -- bash -c "ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10 gripper:=robotiq_2f_85 launch_rviz:=false; exec bash"
sleep 5
gnome-terminal -- bash -c "ros2 run clearpath_manipulators gazebo_to_real_robot_node; exec bash"
sleep 5
gnome-terminal -- bash -c "ros2 run clearpath_manipulators gazebo_to_real_gripper_node; exec bash"
