cmake_minimum_required(VERSION 3.12)

set(PROJ_NAME ros2_whisker_slam)

# This is my standard cmake file from https://github.com/nonbinary-duck/hacky-cmake-cpp-template

# Set the compiler flags
# Use Wall to give good feedback and use pthread so we can use threads
# We need POSIX Threads (Pthreads), a standard for threads, since C++ doesn't have a native implementation of it
# Wno-reorder disables the reorder warning (because it doesn't seem to affect this program)
if (WIN32) # WIN32 is set to true when the target system is windows, including 64-bit according to the CMake docs
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-reorder")
endif()

# Set the project name
project(${PROJ_NAME})

# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Give directories where header files are located
# Technically not needed as we are an executable application and main links to everything we need for us
include_directories(
    ${PROJECT_SOURCE_DIR}/src
)

# Add main.cpp to the executable
add_executable(${PROJ_NAME} main.cpp)
ament_target_dependencies(${PROJ_NAME} rclcpp std_msgs)

# Inlcude source files from the src dir
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

# Use c++ 23 if supported
set_property(TARGET ${PROJ_NAME} PROPERTY CXX_STANDARD 23)


install(TARGETS
    ${PROJ_NAME}
    DESTINATION lib/${PROJ_NAME}
)



ament_package()
