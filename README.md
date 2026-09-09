# Dual Control with Moveit (2 Gen3 7DoF)

1. Make sure to set the IP address of the robotic arms as follows:
   1.  left side arm --> '192.168.1.10'
   2.  right side arm --> '192.168.2.10'
   
**P.S.** Please refer to the Gen3 user guide for more details if needed

2. In a terminal window, run the following to command to start Moveit in RVIZ:

```
  ros2 launch kinova_gen3_7dof_robotiq_2f_85_dual_moveit_config robot.launch.py \
    use_fake_hardware:=false \
    use_internal_bus_gripper_comm:=true \
    left_robot_ip:=192.168.1.10 \
    right_robot_ip:=192.168.2.10

```
This will activate the controllers for both robotic arms and an rviz scene will appear where the motion of both arms can be planned simultaneously while avoiding inter-collision scenarios.


3. Open a new terminal window then run the following python script:
```
cd ~/workspace/ros2_gen3_ws/src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_dual_moveit_config/scripts

./dual_arm_demo.py
```

