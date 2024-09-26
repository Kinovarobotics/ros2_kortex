# Gen3 + Husky Integration
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
cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/robot.yaml ~/clearpath/robot.yaml
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
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/gz_sim.launch.py /opt/ros/humble/share/clearpath_gz/launch/gz_sim.launch.py
```

7. Modify the simulation world using the following command:
```
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/warehouse.sdf /opt/ros/humble/share/clearpath_gz/worlds/warehouse.sdf
```

8. Modify the simulation configuration using the following command:
```
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/gui.config /opt/ros/humble/share/clearpath_gz/config/gui.config
```

9. Run the following command in **NEW** terminal window:
```
ros2 launch clearpath_gz simulation.launch.py
```

## Real-life Setup to establish a sim-to-real robotic arm & gripper synchronization

1. Follow the [instructions](https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/tree/master/api_python/examples) to prepare the setup for the python kortex-api 
2. If you are using Python >=3.10 (it will probably be the case), replace `import collections` by `import collections.abc as collections` in the following files: `~/.local/lib/python3.10/site-packages/google/protobuf/internal/containers.py` and `~/.local/lib/python3.10/site-packages/google/protobuf/internal/well_known_types.py` so that you will be able to use the python kortex-api included in the `commanding_script.py`

3. Import the `~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/Gen3_Husky_Initial_Position.xml` file to the webapp actions.

4. Install control_msgs package using the following command:

```
sudo apt install ros-humble-control-msgs
```

# Usage

1. If any simulation is running, make sure to close it before proceeding

2. Open a new terminal in the following directory:`~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath`

3. Run the following command to launch the entire system: 
```
./gen3_husky.sh
```

4. Wait until the entire system is launched then using the same terminal window, run the following command to launch to command script:
```
python3 commanding_script.py
```

5. In the simulation window switch to the keyboard tab in the teleop panel on the right side, then command the robot using the keyboard. Please refer to the instructions included in `~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/instructions.txt` for more details about the available keyboard commands.