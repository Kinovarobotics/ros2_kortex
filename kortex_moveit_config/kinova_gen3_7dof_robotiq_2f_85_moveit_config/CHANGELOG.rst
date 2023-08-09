^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kinova_gen3_7dof_robotiq_2f_85_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2023-08-09)
------------------
* specify planning pipelines to use (`#157 <https://github.com/Kinovarobotics/ros2_kortex/issues/157>`_)
* Refactor MoveIt Launch files (`#162 <https://github.com/Kinovarobotics/ros2_kortex/issues/162>`_)
* Add use_external_cable parameter to URDF (`#155 <https://github.com/Kinovarobotics/ros2_kortex/issues/155>`_)
* Contributors: Anthony Baker

0.2.1 (2023-07-26)
------------------
* remove stomp for Humble from ros packages (`#153 <https://github.com/PickNikRobotics/ros2_kortex/issues/153>`_)
  - STOMP is available if you build and use MoveIt from src but not
  if you have MoveIt installed from the ros distro
* Contributors: Alex Moriarty

0.2.0 (2023-07-17)
------------------
* Initial Public ROS 2 release of kinova_gen3_7dof_robotiq_2f_85_moveit_config
* Make Gripper velocity and force configurable through URDF (`#137 <https://github.com/PickNikRobotics/ros2_kortex/issues/137>`_)
* Fault and Twist controller from new repo (`#143 <https://github.com/PickNikRobotics/ros2_kortex/issues/143>`_)
* Add camera to 7dof gen3 ignition sim (`#141 <https://github.com/PickNikRobotics/ros2_kortex/issues/141>`_)
* add parameter to use sim time to move_group launch (`#134 <https://github.com/PickNikRobotics/ros2_kortex/issues/134>`_)
* Add MoveIt Config for 7dof arm with robotiq_2f_85 gripper (`#107 <https://github.com/PickNikRobotics/ros2_kortex/issues/107>`_)
* Contributors: Alex Moriarty, Anthony Baker

0.1.0 (2021-09-12)
------------------
