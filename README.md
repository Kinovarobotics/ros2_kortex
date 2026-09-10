# Dual Control with Moveit (2 Gen3 7DoF)

1. Make sure to set the IP address of the robotic arms as follows:
   1.  left side arm --> '192.168.1.10'
   2.  right side arm --> '192.168.2.10'
   
**P.S.** Please refer to the Gen3 user guide for more details if needed

2. Take each arm to 'home' position using the webapp

3. In a terminal window, run the following to command to start Moveit in RVIZ:

```
  ros2 launch kinova_gen3_7dof_robotiq_2f_85_dual_moveit_config robot.launch.py \
    use_fake_hardware:=false \
    use_internal_bus_gripper_comm:=true \
    left_robot_ip:=192.168.1.10 \
    right_robot_ip:=192.168.2.10

```
This will activate the controllers for both robotic arms and an rviz scene will appear where the motion of both arms can be planned simultaneously while avoiding inter-collision scenarios.

**P.S.** Out of the box that inter-arm check has **no margin at all** -- it is bare
mesh-against-mesh contact, so a plan is allowed to pass the other arm by a
fraction of a millimetre and any tracking error becomes a real collision. The
`robot_padding` launch argument does **not** help: MoveIt pads the robot only
against the *world*, and self-collision -- which is what arm-vs-arm is, since both
arms are links of one robot -- is checked unpadded. Step 5 is what adds the margin.

4. In a new terminal window, re-apply the realtime userspace tuning:

```
  sudo ~/workspace/ros2_gen3_ws/rt-tune/rt-fix-userspace.sh
```

**This must be done after EVERY launch, not once per boot.** The script pins the
Kortex transport threads -- the ones that carry each arm's reply back and unblock
`Refresh()` -- to the isolated P-cores at `SCHED_FIFO` 85, and raises the clock
floor on those cores. Thread IDs change every time `ros2_control_node` starts, so
a relaunch silently loses both, and the transport threads drop back to
`SCHED_OTHER` on the E-cores.

**P.S.** Skipping this is not a slow-but-working state, it is an intermittent
FAULT. A transport thread preempted on an E-core delivers the arm's feedback
late; measured position then drifts from the commanded path, and
`joint_trajectory_controller` aborts the goal with

```
  Controller 'left_arm_controller' failed with error PATH_TOLERANCE_VIOLATED:
  Aborted due to path tolerance violation
```

Because it is a tail-latency failure it does not show up straight away -- it
appears after a number of cycles, and the longer the demo runs the likelier it
gets. To confirm the tuning is actually in place, list the realtime threads:

```
  ps -L -o tid,cls,rtprio,psr,comm -p $(pgrep -f ros2_control_node | head -1) | awk 'NR==1 || $2=="FF"'
```

Expect the control loop and the two async component workers at `rtprio` 80 on
cpu 2, 6 and 7, **plus** the transport threads at 85. If 80 is the only priority
you see, the script has not been run for this launch.

5. In a new terminal window, give the arms a real keep-apart margin:

```
  ros2 run kinova_gen3_7dof_robotiq_2f_85_dual_moveit_config arm_padding.py \
    add --margin 0.005
```

This attaches invisible cylinders around one arm's links, sized to contain each
link plus 10 mm. MoveIt then refuses any configuration where the arms come
closer than that -- in RViz and in any script. Add `--verify` to see which named
poses still fit the margin.

**P.S.** The margin lives in the planning scene, so it is **lost whenever
`move_group` restarts** -- re-run this after every relaunch, before planning
anything. `arm_padding.py status` says whether it is active; `arm_padding.py
remove` takes it off. 10 mm is close to the most the current `demo` pose can
take: it has 10-15 mm of true clearance at the wrist/gripper.

6. In the same terminal, run the following python script:
```
cd ~/workspace/ros2_gen3_ws/src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_dual_moveit_config/scripts

./dual_arm_demo.py
```

The demo has its own conservatism controls -- `--sequential` moves one arm at a
time, `--velocity` slows everything down, and `--plan-only` previews in RViz
without touching the hardware. Run `./dual_arm_demo.py --help` for the rest.

