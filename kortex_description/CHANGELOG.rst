^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kortex_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2023-08-09)
------------------
* Refactor MoveIt Launch files (`#162 <https://github.com/Kinovarobotics/ros2_kortex/issues/162>`_)
* Add use_external_cable parameter to URDF (`#155 <https://github.com/Kinovarobotics/ros2_kortex/issues/155>`_)
* Contributors: Anthony Baker

0.2.1 (2023-07-26)
------------------
* fix missing dependencies (`#152 <https://github.com/PickNikRobotics/ros2_kortex/issues/152>`_)
  This fixes missing dependencies which were available from source build
  but were missing from released binary
* Contributors: Alex Moriarty

0.2.0 (2023-07-17)
------------------
* Initial Public ROS 2 release of kortex_description
* Make Gripper velocity and force configurable through URDF (`#137 <https://github.com/PickNikRobotics/ros2_kortex/issues/137>`_)
* rename kortex2 -> kortex (`#144 <https://github.com/PickNikRobotics/ros2_kortex/issues/144>`_)
* Fault and Twist controller from new repo (`#143 <https://github.com/PickNikRobotics/ros2_kortex/issues/143>`_)
* Add camera to 7dof gen3 ignition sim (`#141 <https://github.com/PickNikRobotics/ros2_kortex/issues/141>`_)
* Remove circular dependency between Description and bringup package (`#123 <https://github.com/PickNikRobotics/ros2_kortex/issues/123>`_)
* Add moveit config for 6dof gen 3 (`#109 <https://github.com/PickNikRobotics/ros2_kortex/issues/109>`_)
* Add MoveIt Config for 7dof arm with robotiq_2f_85 gripper (`#107 <https://github.com/PickNikRobotics/ros2_kortex/issues/107>`_)
* Convert package to use robotiq_description and update xacros (`#95 <https://github.com/PickNikRobotics/ros2_kortex/issues/95>`_)
  * add sim_isaac arg to gen3.xacro
* Update gen3 URDF mesh path (`#79 <https://github.com/PickNikRobotics/ros2_kortex/issues/79>`_)
* Support Gen3 lite Hardware (`#73 <https://github.com/PickNikRobotics/ros2_kortex/issues/73>`_)
  * Improve Protective Stop Reset and General Driver Robusteness (`#75 <https://github.com/PickNikRobotics/ros2_kortex/issues/75>`_)
* Add support for ros2_control of the kinova gen3 lite (`#72 <https://github.com/PickNikRobotics/ros2_kortex/issues/72>`_)
* Revert joint2 limit to factory values (`#70 <https://github.com/PickNikRobotics/ros2_kortex/issues/70>`_)
* Kortex fixes for simulations (`#64 <https://github.com/PickNikRobotics/ros2_kortex/issues/64>`_)
* Gripper controller adaptation (`#66 <https://github.com/PickNikRobotics/ros2_kortex/issues/66>`_)
* Remove ros1 files (`#62 <https://github.com/PickNikRobotics/ros2_kortex/issues/62>`_)
* Add conditional finger joint import. (`#57 <https://github.com/PickNikRobotics/ros2_kortex/issues/57>`_)
* Add default params for easier use in simulation. (`#56 <https://github.com/PickNikRobotics/ros2_kortex/issues/56>`_)
* Changes after debugging on real hardware (`#51 <https://github.com/PickNikRobotics/ros2_kortex/issues/51>`_)
* Additional parametrization (`#47 <https://github.com/PickNikRobotics/ros2_kortex/issues/47>`_)
* Use gripper through internal bus extension (`#38 <https://github.com/PickNikRobotics/ros2_kortex/issues/38>`_)
* Contributors: Alex Moriarty, Anthony Baker, Denis Štogl, Marq Rasmussen, livanov93

0.1.0 (2021-09-12)
------------------
* Limited shoulder to keep elbow up (`#22 <https://github.com/PickNikRobotics/ros2_kortex/issues/22>`_)
  * limited shoulder from +-2.41 to +-1.3
  * increase range to +-1.4 rad
* Updated the gripper range to 1 - 99 (`#18 <https://github.com/PickNikRobotics/ros2_kortex/issues/18>`_)
  * Updated the gripper range to 1 - 99
  * Fixed the gripper bug where it never successed
  * Added some better prunt statements
  * Added a gitignore to this project
* Added an action server for the gripper (`#14 <https://github.com/PickNikRobotics/ros2_kortex/issues/14>`_)
  * Fixed a error with an arg
  * Updated the name of open / close gripper group to be the same as the Ur5
  * Added hand_controller to movit controllers.yaml
  * Added a gripper driver for the 85 gripper
  * cleanup some code
  * Have the gripper opening
  * Updated the srdf so that open / close are the correct value
  * Updated the config so that it's range is 0 - 100
  * Working gripper action server
  * Working gripper action server
  * Removed an unused comment
  * Updated the copy write
  * linting
  * linting
  * removed un-used testing files
  * swapped the open / close the other way around
* Add gripper to hw_interface (`#5 <https://github.com/PickNikRobotics/ros2_kortex/issues/5>`_)
  * add robotiq driver
  * Fix hand controller
  * remove hand_controller from gen3_move_it_config
  * Combined launch file for kortex\_(moveit/control)
  * Fix joint value for left_inner_finger_joint
  * Run pre-commit
  * Revert "Merge pull request `#1 <https://github.com/PickNikRobotics/ros2_kortex/issues/1>`_ from PickNikRobotics/pr-robotiq_driver"
  This reverts commit 77d7e71b6cc7eea06eccff2cbcf3862cfbd5f2df, reversing
  changes made to 5ed5a8d1dde4ffadd0132043c894c1832153da47.
  * Send gripper commands, too
  * Launch the gripper controller
  Initialize gripper to position mode
  Copy gripper command from their action server
  * MoveIt does not manage gripper controller since it's not an action server anymore
  * Cannot check for 3 interfaces when gripper joints are included
  * Add missing deps
  Add more deps
  Add even more deps
  Restore default Kinova IP address
  Initialize info\_
  * Move session initialization
  * Hard-code robot ip
  Comment the gripper write(), for now
  Fix gripper xacro
  Comment the writing of joint values
  Debug statement
  To test gripper, take out of Servo mode
  * Do not write() anything whatsoever
  Copy example directly
  Set to BYPASS_SERVOING before gripper cmd
  Try low-level gripper command
  Initialize gripper position
  * Only include <ros2_control> xacro stuff once
  * Cleanup and better comments
  * working arm and gripper interface, changed ip back to ..0.10
  * file cleanup
  * fix format
  * add documentation, revert ros-control files for non-working configurations
  Co-authored-by: Marq <marq.razz@gmail.com>
  Co-authored-by: JafarAbdi <cafer.abdi@gmail.com>
* Merged in andyz/fix_arg_parsing_again (pull request `#12 <https://github.com/PickNikRobotics/ros2_kortex/issues/12>`_)
  Fix arg parsing (again)
  Approved-by: brennand
* Fix arg parsing (again)
* Merged in andyz/standardize_arg_parsing (pull request `#11 <https://github.com/PickNikRobotics/ros2_kortex/issues/11>`_)
  Standardize arg handling for 7dof gen3 xacros
  * Standardize arg handling for 7dof gen3 xacros
  Approved-by: Marq Rasmussen
* Merged in pr-fix_precommit (pull request `#9 <https://github.com/PickNikRobotics/ros2_kortex/issues/9>`_)
  Remove broken package dependencies and fix CI
  * Remove broken package dependencies and fix CI
  * pre commit fixes
  Approved-by: Andy Zelenak
* Merged in pr-fix_ip_address (pull request `#7 <https://github.com/PickNikRobotics/ros2_kortex/issues/7>`_)
  Add ip_address argument to all gen3 xacros
  * add ip_address argument to all gen3 xacros
  * revert hardware_interface changes
  Approved-by: Andy Zelenak
* Merged in pr-fix_precommit (pull request `#8 <https://github.com/PickNikRobotics/ros2_kortex/issues/8>`_)
  clang and pre-commit fixes
  Approved-by: Andy Zelenak
* clang and pre-commit fixes
* Merged in pr-ci_setup (pull request `#3 <https://github.com/PickNikRobotics/ros2_kortex/issues/3>`_)
  Add CI and pre-commit and apply relevant formatting fixes
  * Add CI, pre commit and clang-format
  * pre-commit fixes
  * Add pre-commit docs
  Approved-by: Andy Zelenak
* Add ros2_control hardware interface
  Approved-by: Andy Zelenak
* Merged in pr-ros_control (pull request `#2 <https://github.com/PickNikRobotics/ros2_kortex/issues/2>`_)
  * add ros2_control hardware interface
  Approved-by: Andy Zelenak
* Merged in pr-update_urdf (pull request `#1 <https://github.com/PickNikRobotics/ros2_kortex/issues/1>`_)
  Update URDF
  * enable parent argument for gen3
  Approved-by: Andy Zelenak
* updated readme with ported ROS2 packages
* port robot_description to ros2
* Contributors: Andy Zelenak, AndyZe, Brennand Pierce, Marq Rasmussen, Vatan Aksoy Tezer, marqrazz
