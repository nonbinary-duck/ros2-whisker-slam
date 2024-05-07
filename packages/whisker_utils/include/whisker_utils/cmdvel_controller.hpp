#pragma once
#ifndef H_599955_SRC_CMD_VEL_CONTROLLER
#define H_599955_SRC_CMD_VEL_CONTROLLER 1


///
///
/// Uses code taken from the demos minimal_subscriber lambda
/// Found here:
/// https://github.com/ros2/examples/blob/iron/rclcpp/topics/minimal_subscriber/lambda.cpp
///
///


#include <iostream>
#include <random>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/f64vel4.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace ros2_whisker_slam
{
    /**
     * @brief An extremely rudimentary controller for our little bubble robot
     */
    class CMDVelController : public rclcpp::Node
    {
        public:

            inline CMDVelController() : Node("cmd_vel_controller")
            {
                // Setup the listener for cmdVel
                this->cmdVelSub = this->create_subscription< geometry_msgs::msg::Twist >(
                    // The topic to subscribe to
                    "cmd_vel",
                    // qos?
                    10,
                    // The callback
                    [this](geometry_msgs::msg::Twist::UniquePtr msg)
                    {
                        interfaces::msg::F64vel4 motorControl;

                        // First figure out how fast we should be going
                        motorControl.fl = CMDVelController::LIN_VEL_COEFFICIENT * msg->linear.x;
                        // For moving (only) forwards all the wheels should have the same velocity
                        motorControl.fr = motorControl.fl;
                        motorControl.rl = motorControl.fl;
                        motorControl.rr = motorControl.fl;

                        // Then figure out how fast we should be spinning
                        double spinFac = CMDVelController::ANG_VEL_COEFFICIENT * msg->angular.z;
                        // And add that speed to the velocity command
                        motorControl.fl -= spinFac;
                        motorControl.fr += spinFac;
                        motorControl.rl -= spinFac;
                        motorControl.rr += spinFac;
                        
                        // Publish our approximate model
                        this->motorVelPub->publish(motorControl);
                    }
                );

                // Setup a publisher to the motor control topic
                this->motorVelPub = this->create_publisher<interfaces::msg::F64vel4>("/motorControl", 10);
            }

        private:

            /**
             * @brief The relationship between the speed we subscribe to the speed we publish
             */
            constexpr static double LIN_VEL_COEFFICIENT = 2.067343303359;
            constexpr static double ANG_VEL_COEFFICIENT = 2.22816920328653470077;

            /**
             * @brief Our subscription to the cmd_vel topic
             */
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
        
            /**
             * @brief The publisher to our 4-diff-drive robot
             */
            rclcpp::Publisher<interfaces::msg::F64vel4>::SharedPtr motorVelPub;
    };


} // End namespace ros2_whisker_slam

#endif // H_599955_SRC_CMD_VEL_CONTROLLER