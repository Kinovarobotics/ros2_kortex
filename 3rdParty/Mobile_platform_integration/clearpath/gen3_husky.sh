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
sleep 30

# Function to retry a command until it succeeds
retry_command() {
    local cmd="$1"
    local max_retries=10
    local count=0
    until eval "$cmd"
    do
        count=$((count + 1))
        if [ $count -ge $max_retries ]; then
            echo "Command failed after $max_retries attempts: $cmd"
            return 1
        fi
        echo "Retry $count/$max_retries: $cmd"
        sleep 2
    done
}

# Reset the robot pose in Ignition Gazebo
echo "Resetting robot pose..."
retry_command "ign service -s /world/warehouse/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 5000 --req \"position: {x: 0.000925, y: 0.000067, z: 0.132280}, orientation: {x: 0, y: 0, z: 0.000007, w: 1}, name: 'a200_0000/robot'\""

# Reset the grip box pose in Ignition Gazebo
echo "Resetting grip box pose..."
retry_command "ign service -s /world/warehouse/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 5000 --req \"position: {x: 3.0, y: 0.0, z: 0.53}, orientation: {x: 0, y: 0, z: 0, w: 1}, name: 'grip_box'\""
sleep 5

# Publish the initial joint positions to the simulation
echo "Setting arm initial joint positions..."
ros2 topic pub /a200_0000/arm_0_joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: ''
  },
  joint_names: [
    'arm_0_joint_1',
    'arm_0_joint_2',
    'arm_0_joint_3',
    'arm_0_joint_4',
    'arm_0_joint_5',
    'arm_0_joint_6',
    'arm_0_joint_7'
  ],
  points: [
    {
      positions: [-0.23921483, 0.85088292, 0.1007404, 2.40223628, -0.07318166, -1.70646077, -1.63699667],
      velocities: [],
      accelerations: [],
      effort: [],
      time_from_start: {sec: 10, nanosec: 0}
    }
  ]
}" -1
echo "Setting gripper initial position..."
ros2 action send_goal /a200_0000/robotiq_85_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.7, max_effort: 100.0}}"
sleep 20
# Prompt user to check simulation status before proceeding
read -p "Check the simulation scene and the related terminal for any error or anomaly and check the roobt state using the Webapp then press Enter to continue if all is working well, otherwise relaunch the system again or eventually restart the computer"

gnome-terminal -- bash -c "ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.10 gripper:=robotiq_2f_85 launch_rviz:=false; exec bash"
sleep 5
gnome-terminal -- bash -c "ros2 run clearpath_manipulators gazebo_to_real_robot_node; exec bash"
sleep 5
gnome-terminal -- bash -c "ros2 run clearpath_manipulators gazebo_to_real_gripper_node; exec bash"

read -p "Wait until the entire system is up and running then press Enter to activate the Keyboard control"

gnome-terminal -- bash -c "python3 commanding_script.py; exec bash"
