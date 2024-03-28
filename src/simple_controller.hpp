#pragma once
#ifndef H_599955_SRC_SIMPLE_CONTROLLER
#define H_599955_SRC_SIMPLE_CONTROLLER 1


///
///
/// Uses code taken from the demos minimal_subscriber lambda
/// Found here:
/// https://github.com/ros2/examples/blob/iron/rclcpp/topics/minimal_subscriber/lambda.cpp
///
///


#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "interfaces/msg/f64vel4.hpp"

namespace ros2_whisker_slam
{
    /**
     * @brief An extremely rudimentary controller for our little bubble robot
     */
    class SimpleController : public rclcpp::Node
    {
        public:
            // Do all of the setup for our node in a nice and untidy inline constructor,
            // entirely defeating the organisational power provided by objects
            inline SimpleController() : Node("simple_controller")
            {
                // Setup the listener for the simulation time
                this->simTimeSubscription = this->create_subscription< std_msgs::msg::Float32 >(
                    // The topic to subscribe to
                    "simTime",
                    // qos?
                    10,
                    // The callback
                    [this](std_msgs::msg::Float32::UniquePtr msg)
                    {
                        // Simply store the state of the simulation time
                        this->simTime = msg->data;
                    }
                );

                // Setup the listener for the sensor
                this->whiskerLidar = this->create_subscription< sensor_msgs::msg::LaserScan >(
                    // The topic to subscribe to
                    "scan",
                    // qos?
                    10,
                    // The callback
                    [this](sensor_msgs::msg::LaserScan::UniquePtr msg)
                    {
                        interfaces::msg::F64vel4 cmdVel;

                        cmdVel.fl = 0.5; cmdVel.fr = 0.5;
                        cmdVel.rr = 0.5; cmdVel.rl = 0.5;

                        // If we've not detected an impact
                        if (this->simTime - this->previousImpact > SimpleController::IMPACT_ADJUST_TIME)
                        {
                            // Check if we're colliding with something
                            for (size_t i = 0; i < msg->ranges.size(); i++)
                            {
                                if (msg->ranges.at(i) != 0.0)
                                {
                                    // If we've bumped into left, correct right
                                    this->correctLeft = i > 2;
                                    // Record this impact
                                    this->previousImpact = this->simTime;
                                    
                                    RCLCPP_INFO(this->get_logger(), "Bumped into the %s side", (this->correctLeft)? "right" : "left" );
                                    RCLCPP_INFO(this->get_logger(), "Reversing for %f seconds", (this->previousImpact + SimpleController::IMPACT_ADJUST_TIME * 0.25) - this->simTime );
                                    
                                    // Begin reversing
                                    cmdVel.fl = -0.5; cmdVel.fr = -0.5;
                                    cmdVel.rr = -0.5; cmdVel.rl = -0.5;
                                    break;
                                }
                                // If not, continue to move forward
                            }
                        }
                        else
                        {
                            // If we've in the first half of our impact time, move backward
                            if (this->simTime - this->previousImpact < (SimpleController::IMPACT_ADJUST_TIME * 0.25))
                            {
                                RCLCPP_INFO(this->get_logger(), "Reversing for %f seconds", (this->previousImpact + SimpleController::IMPACT_ADJUST_TIME * 0.25) - this->simTime );

                                cmdVel.fl = -0.5; cmdVel.fr = -0.5;
                                cmdVel.rr = -0.5; cmdVel.rl = -0.5;
                            }
                            // Then in the latter half, rotate
                            else
                            {
                                RCLCPP_INFO(this->get_logger(), "Turning %s for %f seconds", (this->correctLeft)? "left" : "right", (this->previousImpact + SimpleController::IMPACT_ADJUST_TIME) - this->simTime );

                                if (this->correctLeft)
                                {
                                    cmdVel.fl = -0.6; cmdVel.fr =  0.6;
                                    cmdVel.rr =  0.6; cmdVel.rl = -0.6;
                                }
                                else
                                {
                                    cmdVel.fl =  0.6; cmdVel.fr = -0.6;
                                    cmdVel.rr = -0.6; cmdVel.rl =  0.6;
                                }
                            }
                        }
                        
                        

                        this->motorVelPub->publish(cmdVel);
                        // // If we've collided with something, turn left
                        // // ... and continue to rotate left for IMPACT_ADJUST_TIME
                        // if (msg->data || this->simTime - this->previousImpact < SimpleController::IMPACT_ADJUST_TIME)
                        // {
                        //     // Check if this impact is after the previous impact time
                        //     if (this->previousImpact + SimpleController::IMPACT_ADJUST_TIME < this->simTime)
                        //     {
                        //         // If we're dealing with a new impact, record the incident
                        //         this->previousImpact = this->simTime;
                        //         // Record to the console for debugging purposes
                        //         RCLCPP_INFO(this->get_logger(), "New collision detected...");
                        //     }


                        //     // Move left for IMPACT_ADJUST_TIME
                        //     // This truly is a simple controller, it's just to get the robot moving to test other elements
                        //     RCLCPP_INFO(this->get_logger(), "Rotating for '%f' seconds", SimpleController::IMPACT_ADJUST_TIME - (this->simTime - this->previousImpact));
                        //     // Set the speed of the left motor to -4.5
                        //     std_msgs::msg::Float32 msg; msg.data = -4.5;
                        //     this->leftMotorPub->publish(msg);
                        //     // Set the speed of the right motor to 4.5
                        //     // Seemingly we publish a package which contains a complete copy of the message rather than a reference to it
                        //     msg.data = 4.5;
                        //     this->rightMotorPub->publish(msg);
                        // }
                        // else
                        // {
                        //     // Business as usual
                        //     // More debug logging
                        //     RCLCPP_INFO(this->get_logger(), "Moving forward");
                        //     // Maintain a default state of moving forward by this fixed amount
                        //     std_msgs::msg::Float32 msg; msg.data = 3.0;
                        //     this->leftMotorPub->publish(msg);
                        //     this->rightMotorPub->publish(msg);
                        // }
                        
                    }
                );

                // (this is unbelievably still the constructor btw)
                // Give us a publisher to send updates to the motor
                // this->leftMotorPub  = this->create_publisher<std_msgs::msg::Float32>("leftMotorSpeed", 10);
                // this->rightMotorPub = this->create_publisher<std_msgs::msg::Float32>("rightMotorSpeed", 10);
                this->motorVelPub = this->create_publisher<interfaces::msg::F64vel4>("/motorControl", 10);

                // And give an inital state of zero speed for our motors
                // std_msgs::msg::Float32 msg; msg.data = 0.0;

                // // Publish this inital state
                // this->leftMotorPub->publish(msg);
                // this->rightMotorPub->publish(msg);

            }

        private:
            // Assign our subscriptions and publishers on the class's stack since this is just a testing dummy controller
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr whiskerLidar;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr simTimeSubscription;
            rclcpp::Publisher<interfaces::msg::F64vel4>::SharedPtr motorVelPub;

            // Initialise some things
            float simTime = 0;
            float previousImpact = 0;
            // Should we turn left or right?
            bool  correctLeft = false;
            // Make sure we evaluate this at compile time
            constexpr static float IMPACT_ADJUST_TIME = 4.0;
    };


} // End namespace ros2_whisker_slam

#endif // H_599955_SRC_SIMPLE_CONTROLLER