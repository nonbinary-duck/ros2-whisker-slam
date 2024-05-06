# Version requirement explanation:
# New in version 3.11: The source files can be omitted if they are added later using target_sources().
# New in version 3.20 cxx_std_23
# New in version 3.21 c_std_23
cmake_minimum_required(VERSION 3.21)

project(whisker_utils)

# Add warnings to GCC and clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# We also depend upon our interfaces package, which needs to be built first
find_package(interfaces REQUIRED)

#
# Executables and libraries
#

# Add our nodes for ros2 run...
add_executable(reactive_controller src/reactive_controller.cpp)
ament_target_dependencies(reactive_controller rclcpp std_msgs sensor_msgs interfaces)

# Add the include directory to our executables
target_include_directories(reactive_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/whisker_utils>
    $<INSTALL_INTERFACE:include>)

# Require some basic minimum level of C
target_compile_features(reactive_controller PUBLIC c_std_23 cxx_std_23)

install(TARGETS reactive_controller
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
