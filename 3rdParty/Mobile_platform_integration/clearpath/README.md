# Gen3 + Husky Integration

The Gen3 robotic arm from Kinova has been mounted on the Husky mobile platform from clearpath in Gazebo simulation environment. Both the arm and the mobile platform can be commanded simultaneously and the following explains in details what is needed to build the needed setup.
## Simulation Setup

1. Install Gazebo Fortress
```
sudo apt-get update && sudo apt-get install wget
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update && sudo apt-get install ignition-fortress
```

2. Install Clearpath Simulator
```
sudo apt-get update
sudo apt-get install ros-humble-clearpath-simulator
```

3. Place the `robot.yaml` file in the correct directory
```
mkdir ~/clearpath
cd ~/clearpath
cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/3rdParty/Mobile_platform_integration/clearpath/robot.yaml ~/clearpath/robot.yaml
```

4. Generate the `setup.bash` file
```
ros2 run clearpath_generator_common generate_bash -s ~/clearpath
```

5. Source the previous `setup.bash` file in your `~/.bashrc`
```
echo 'source ~/clearpath/setup.bash' >> ~/.bashrc
```
6. Modify the `gz_sim.launch.py` using the following command:
```
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/3rdParty/Mobile_platform_integration/clearpath/gz_sim.launch.py /opt/ros/humble/share/clearpath_gz/launch/gz_sim.launch.py
```
This will lead to the following modifications:

    a. Starting the simulation in a running state which will prevent controllers timeout

    b. Adding the keyboard press topic to the ROS2-Gazebo communication bridge

7. Modify the simulation world using the following command:
```
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/3rdParty/Mobile_platform_integration/clearpath/warehouse.sdf /opt/ros/humble/share/clearpath_gz/worlds/warehouse.sdf
```
This will lead to the following modifications:

    a. Decreasing the simulation step size down to `1 millisec` to allow for a high frequency publication of the joint states of the spawned robot

    b. Adding the table and grasping box to the simulation scene

    c. Simplifying the warehouse environment to avoid system's crashing (please note that more powerful computers can run more sophisticated simulation scenes)

8. Modify the simulation configuration using the following command:
```
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/3rdParty/Mobile_platform_integration/clearpath/gui.config /opt/ros/humble/share/clearpath_gz/config/gui.config
```
This will change the default topic name in the teleop plugin panel in the simulation window

9. Run the following command in **NEW** terminal window:
```
ros2 launch clearpath_gz simulation.launch.py
```

## Real-life Setup to establish a sim-to-real robotic arm & gripper synchronization

1. Follow the [instructions](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/tree/master/api_python/examples) to prepare the setup for the python kortex-api. **P.S.** Instead of directly installing the .whl file, please make sure to use the following commands:
```
pip install protobuf==3.20 deprecated==1.2.7
pip install <file.whl> --no-deps
```

2. Import the `~/workspace/ros2_kortex_ws/src/ros2_kortex/3rdParty/Mobile_platform_integration/clearpath/Gen3_Husky_Initial_Position.xml` file to the webapp actions.

3. It is recommended to use CycloneDDS for a more reliable communication at a high frequency. To use this DDS, make sure to use the following command:
```
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

4. Install control_msgs package if not already installed using the following command:

```
sudo apt install ros-humble-control-msgs
```

# Usage

1. If any simulation is running, make sure to close it before proceeding. Also, make sure to use the following command to stop any trace from previous sessions:
```
killall ruby
```

2. Open a new terminal in the following directory:`~/workspace/ros2_kortex_ws/src/ros2_kortex/3rdParty/Mobile_platform_integration/clearpath`

3. Run the following command to launch the entire system and follow closely the step by step instructions:
```
./gen3_husky.sh
```

4. Once the entire system is up and running, in the simulation window switch to the keyboard tab in the teleop panel on the right side, then command the robot using the keyboard. Please refer to the instructions included in `~/workspace/ros2_kortex_ws/src/ros2_kortex/3rdParty/Mobile_platform_integration/clearpath/instructions.txt` for more details about the available keyboard commands.

5. If the communication between the simulation and the real-life robot is lost, please restart the entire code.
