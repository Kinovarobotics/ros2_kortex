^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kortex_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.3 (2025-02-27)
------------------
* Separated the arm and gripper control for Gen3_Lite + Fixed bugs (`#252 <https://github.com/Kinovarobotics/ros2_kortex/issues/252>`_)
  * Separated the arm and gripper controllers for Gen3_Lite in Kortex_bringup
* Added the empty gripper option for gen3.launch.py (`#242 <https://github.com/Kinovarobotics/ros2_kortex/issues/242>`_)
* Fix mock_hardware and enable simulating gen3_lite (`#196 <https://github.com/Kinovarobotics/ros2_kortex/issues/196>`_)
  * Add additional guards if user sets use_fake_hardware and use_internal_bus_gripper_comm true
* Contributors: Marq Rasmussen, aalmrad, smoya23

0.2.2 (2023-08-09)
------------------

0.2.1 (2023-07-26)
------------------
* fix missing dependencies (`#152 <https://github.com/PickNikRobotics/ros2_kortex/issues/152>`_)
  This fixes missing dependencies which were available from source build
  but were missing from released binary
* Contributors: Alex Moriarty

0.2.0 (2023-07-17)
------------------
* Initial Public ROS 2 release of kortex_bringup
* Make Gripper velocity and force configurable through URDF (`#137 <https://github.com/PickNikRobotics/ros2_kortex/issues/137>`_)
* Fix License -> BSD and update maintainers (`#146 <https://github.com/PickNikRobotics/ros2_kortex/issues/146>`_)
* Contributors: Alex Moriarty, Anthony Baker

0.1.0 (2021-09-12)
------------------
