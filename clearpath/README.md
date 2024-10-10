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
This will lead to the following modifications:

    a. Starting the simulation in a running state which will prevent controllers timeout

    b. Adding the keyboard press topic to the ROS2-Gazebo communication bridge

7. Modify the simulation world using the following command:
```
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/warehouse.sdf /opt/ros/humble/share/clearpath_gz/worlds/warehouse.sdf
```
This will lead to the following modifications:

    a. Decreasing the simulation step size down to `1 millisec` to allow for a high frequency publication of the joint states of the spawned robot

    b. Adding the table and grasping box to the simulation scene

    c. Simplifying the warehouse environment to avoid system's crashing (please note that more powerful computers can run more sophisticated simulation scenes)

8. Modify the simulation configuration using the following command:
```
sudo cp ~/workspace/ros2_kortex_ws/src/ros2_kortex/clearpath/gui.config /opt/ros/humble/share/clearpath_gz/config/gui.config
```
This will change the default topic name in the teleop plugin panel in the simulation window

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

**P.S.** Before restarting the simulation, make sure to use the following command to eliminate any leftovers from a previous iteration:
```
killall ruby
```

The following file "~/workspace/ros2_kortex_ws/install/clearpath_config/lib/python3.10/site-packages/clearpath_config/common/types/rmw_implementation.py" was modified to the following:
```
# Software License Agreement (BSD)
#
# @author    Luis Camero <lcamero@clearpathrobotics.com>
# @copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

class RMWImplementation:
    CONNEXT = "rmw_connext_cpp"
    CYCLONE_DDS = "rmw_cyclonedds_cpp"
    FAST_RTPS = "rmw_fastrtps_cpp"
    GURUM_DDS = "rmw_gurumdds_cpp"

    ALL_SUPPORTED = [CYCLONE_DDS]

    DEFAULT = CYCLONE_DDS

    def __init__(
            self,
            rmw: str = DEFAULT
            ) -> None:
        self.assert_valid(rmw)
        self.rmw = rmw

    def __eq__(self, other: object) -> bool:
        if isinstance(other, str):
            return self.rmw == other
        elif isinstance(other, RMWImplementation):
            return self.rmw == other.rmw
        else:
            return False

    def __str__(self) -> str:
        return self.rmw

    @classmethod
    def is_valid(cls, rmw: str) -> bool:
        return rmw in cls.ALL_SUPPORTED

    @classmethod
    def assert_valid(cls, rmw: str) -> None:
        assert cls.is_valid(rmw), ("\n".join[
            "RMW '%s' not supported." % rmw,
            "RMW must be one of: '%s'" % cls.ALL_SUPPORTED
        ])
```

Also check and push the modifications on the clearpath_common and ros2_kortex