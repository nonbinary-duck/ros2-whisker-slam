# ROS2 Whisker SLAM

This is the beginning of an application of tactile SLAM.

To use & compile, a Linux machine is likely required. This toolchain works: PopOS (an Ubuntu/Debian Distro), GCC & CMake with ROS2 iron installed and a working ROS2-compatible CoppeliaSim.

This project is a work in progress and the current state of the project is perhaps incompatible with a generic system, particularly as a working ROS2-compatible [CoppeliaSim](https://www.coppeliarobotics.com/) is required to run the Coppelia demo scene. The simple-controller node provided by this early prototype is simple enough to work on other ROS2 simulators or even a real robot (perhaps with constants changed...), however it is designed for the provided Coppelia scene (`bubble.ttt`), and Coppelia requires a bit of setup to work with ROS2, see [ROS2 Interface plugin for CoppeliaSim](https://github.com/CoppeliaRobotics/simROS2) and the [official documentation](https://manual.coppeliarobotics.com/en/ros2Tutorial.htm) where much of the provided scene originates from.

However, compilation might be done in the standard CMake way (i.e. `mkdir bin && cd bin && cmake .. && make && ./ros2_whisker_slam`) or through a "Makefile" (i.e.`make && ./build/release/ros2_whisker_slam`, must be in project root before running `make` this way here) which essentially executes the before CMake command. A VSCode configuration is also provided where the standard build and launch options should work (provided the system has the prerequisite ROS2-iron and typical GNU & cmake dev packages and runs Linux). The build and launch configurations use the Makefile script in the project root.
