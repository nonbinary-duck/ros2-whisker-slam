# ROS2 Whisker SLAM

This is the beginning of an application of tactile SLAM.

To use a Linux machine is likely required, and the developer uses the following toolchain: PopOS (an Ubuntu Distro), GCC/CMake with ROS2 iron installed.

This project is a work in progress and this state of the project is not exactly targeted for compatibility. However, compilation can be done in the standard CMake way (i.e. `mkdir bin && cd bin && cmake .. && make`) or through the "Makefile" which essentially executes those commands. A VSCode configuration is also provided where the standard build and launch options should work (provided the system has the prerequisite ROS2-iron and standard GNU & cmake dev packages and runs Linux). The build and launch configurations use the make script.
