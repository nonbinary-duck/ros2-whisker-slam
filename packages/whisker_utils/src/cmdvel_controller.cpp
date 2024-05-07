#include <iostream>

using std::cout, std::cin, std::endl;

#include "rclcpp/rclcpp.hpp"

#include "utils.hpp"
#include "cmdvel_controller.hpp"


/**
 * @brief Called when the program is launched
 * 
 * @param argc Count of command-line arguments
 * @param argv Args, zero is the name of the program
 * @return int An error code
 */
int main(int argc, char *argv[])
{
	// cout << "Hello, world!! I'm simple-ros2-test :)" << endl;

	// init spin and shutdown statements taken from: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
	rclcpp::init(argc, argv);

	// Print to regular stdout that the application might be running
	endl( cout << "cmd_vel to 4 diff drive command controller is likely working now" );

	// rclcpp::spin( std::make_shared<ros2_whisker_slam::SimpleMap>() );
	// Give ROS2 our node
	rclcpp::spin( std::make_shared<ros2_whisker_slam::CMDVelController>() );
	rclcpp::shutdown();

	
	return 0;
}