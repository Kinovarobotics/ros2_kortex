^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kortex_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2 (2023-08-09)
------------------
* cxx: remove unused-but-set-parameter (`#164 <https://github.com/Kinovarobotics/ros2_kortex/issues/164>`_)
* Contributors: Alex Moriarty

0.2.1 (2023-07-26)
------------------
* Change kortex_api header and library install locations (`#156 <https://github.com/PickNikRobotics/ros2_kortex/issues/156>`_)
  This commit does several two main things:
  1) kortex_api now only installs the header files and they now do not pollute include
  2) kortex_driver gets the binary libKortexApiCpp.a itself
  kortex_driver gets the headers from kortex_api this avoids protobuf errors because
  those header files and the libKortexApiCpp.a were generated using non-system protobuf
  kortex_api cannot install the libKortexApiCpp.a because CMake does not allow installing library which was IMPORTED
  This should fix the debian packages which are currently released
* fix missing dependencies (`#152 <https://github.com/PickNikRobotics/ros2_kortex/issues/152>`_)
  This fixes missing dependencies which were available from source build
  but were missing from released binary
* Contributors: Alex Moriarty

0.2.0 (2023-07-17)
------------------
* Initial Public ROS 2 release of kortex_driver
* cxx: fixup compiler warnings (`#148 <https://github.com/PickNikRobotics/ros2_kortex/issues/148>`_)
* Make Gripper velocity and force configurable through URDF (`#137 <https://github.com/PickNikRobotics/ros2_kortex/issues/137>`_)
* Fix License -> BSD and update maintainers (`#146 <https://github.com/PickNikRobotics/ros2_kortex/issues/146>`_)
* Contributors: Alex Moriarty, Anthony Baker, Denis Štogl

0.1.0 (2021-09-12)
------------------
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
* port robot_description to ros2
* Contributors: Vatan Aksoy Tezer, marqrazz
