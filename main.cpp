#include <iostream>

using std::cout, std::cin, std::endl;

#include "rclcpp/rclcpp.hpp"

#include "src/utils.hpp"
#include "src/simple_controller.hpp"
#include "src/simple_mapper.hpp"


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
	// rclcpp::spin( std::make_shared<simple_ros2_test::SimpleMap>() );
	// Give ROS2 our node
	rclcpp::spin( std::make_shared<simple_ros2_test::SimpleController>() );
	rclcpp::shutdown();

	
	return 0;
}