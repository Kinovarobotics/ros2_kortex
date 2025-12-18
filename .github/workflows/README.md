# Detailed Build Statuses

This package follows build jobs conventions of ros2 control to keep ahead of future breaking changes.

A RED job might not mean that this package is currently broken but that an dependency is currently broken in a future version or that this package might need to be updated in the future.

To avoid overloading users viewing the main README page a full list of build statues can be kept here for package maintainers

ROS2 Distro | Jazzy | Rolling
:---------: | :---: | :-----:
| **Branch** | [`main`](https://github.com/Kinovarobotics/ros2_kortex/tree/main) | [`main`](https://github.com/Kinovarobotics/ros2_kortex/tree/main)
| **Build Status** | [![Jazzy Binary Build](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=main)](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-binary-build.yml?branch=main) <br /> [![Jazzy Source Build](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-source-build.yml/badge.svg?branch=main)](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/jazzy-source-build.yml?branch=main) | [![Rolling Binary Build](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/rolling-binary-build.yml/badge.svg?branch=main)](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/rolling-binary-build.yml?branch=main) <br /> [![Rolling Semi-Binary Build](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=main)](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/rolling-semi-binary-build.yml?branch=main) <br /> [![Rolling Source Build](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/rolling-source-build.yml/badge.svg?branch=main)](https://github.com/Kinovarobotics/ros2_kortex/actions/workflows/rolling-source-build.yml?branch=main)

### Explanation of different build types

**NOTE**: For the main branch:
- **Jazzy** has two build stages (binary and source)
- **Rolling** has three build stages (binary, semi-binary, and source)

These stages check current and future compatibility of the package:

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `src/$NAME$/$NAME$-not-released.<ros-distro>.repos`

2. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `src/$NAME$/$NAME$.repos`

3. Source build - also core ROS packages are build from source. It shows potential issues in the mid future.
