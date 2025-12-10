# Detailed Build Statuses

This package follows build jobs conventions of ros2 control to keep ahead of future breaking changes.

A RED job might not mean that this package is currently broken but that an dependency is currently broken in a future version or that this package might need to be updated in the future.

To avoid overloading users viewing the main README page a full list of build statues can be kept here for package maintainers

ROS2 Distro | Jazzy
:---------: | :---:
| **Branch** | [`jazzy`](https://github.com/Kinovarobotics/ros2_kortex/tree/jazzy)
| **Build Status** | [![Jazzy Binary Build](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=jazzy)](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-binary-build.yml?branch=jazzy) <br /> [![Jazzy Source Build](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-source-build.yml/badge.svg?branch=jazzy)](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-source-build.yml?branch=jazzy)

### Explanation of different build types

**NOTE**: For the jazzy branch, there are two build stages checking current and future compatibility of the package.

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

2. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
