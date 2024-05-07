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
#include <random>
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
     * @brief A reactive controller for our little bubble robot
     */
    class SimpleController : public rclcpp::Node
    {
        public:

            inline SimpleController() : Node("simple_controller")
            {
                // Seed our random number generator with non-deterministic random values
                std::random_device ndRand;
                this->rng.seed(ndRand());
                
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
                this->cmdVelSub = this->create_subscription< sensor_msgs::msg::LaserScan >(
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
                        if (this->simTime - this->previousImpact > this->random_IMPACT_ADJUST_TIME)
                        {
                            // Check if we're colliding with something
                            for (size_t i = 0; i < msg->ranges.size(); i++)
                            {
                                if (msg->ranges.at(i) != 0.0)
                                {
                                    // If we've bumped into left, correct right
                                    this->correctLeft = i < 2;
                                    // Record this impact
                                    this->previousImpact = this->simTime;
                                    
                                    RCLCPP_INFO(this->get_logger(), "Bumped into the %s side", (this->correctLeft)? "right" : "left" );
                                    RCLCPP_INFO(this->get_logger(), "Reversing for %f seconds", (this->previousImpact + this->random_IMPACT_ADJUST_TIME * 0.25) - this->simTime );

                                    // Randomise our impact adjust time
                                    this->random_IMPACT_ADJUST_TIME = SimpleController::IMPACT_ADJUST_TIME - 1.0 + ((float)(this->rng() % 200) / 200.0);
                                    
                                    // Begin reversing
                                    cmdVel.fl = -0.5; cmdVel.fr = -0.5;
                                    cmdVel.rr = -0.5; cmdVel.rl = -0.5;
                                    break;
                                }
                                // If not, continue to move forward
                            }
                        }
                        else if (this->simTime - this->previousImpact < SimpleController::IMPACT_FREEZE_TIME)
                        {
                            // Do nothing...
                        }
                        else if (this->simTime - this->previousImpact < SimpleController::IMPACT_DELAY_TIME)
                        {
                            RCLCPP_INFO(this->get_logger(), "Pausing %f seconds", (this->previousImpact + SimpleController::IMPACT_DELAY_TIME) - this->simTime );
                            cmdVel.fl = 0.0; cmdVel.fr = 0.0;
                            cmdVel.rr = 0.0; cmdVel.rl = 0.0;
                        }
                        else
                        {
                            // If we've in the first half of our impact time, move backward
                            if (this->simTime - this->previousImpact < (this->random_IMPACT_ADJUST_TIME * 0.1667))
                            {
                                RCLCPP_INFO(this->get_logger(), "Reversing for %f seconds", (this->previousImpact + this->random_IMPACT_ADJUST_TIME * 0.25) - this->simTime );

                                cmdVel.fl = -0.5; cmdVel.fr = -0.5;
                                cmdVel.rr = -0.5; cmdVel.rl = -0.5;
                            }
                            // Then in the latter half, rotate
                            else
                            {
                                RCLCPP_INFO(this->get_logger(), "Turning %s for %f seconds", (this->correctLeft)? "left" : "right", (this->previousImpact + this->random_IMPACT_ADJUST_TIME) - this->simTime );

                                if (this->correctLeft)
                                {
                                    cmdVel.fl = -0.7; cmdVel.fr =  0.7;
                                    cmdVel.rr =  0.7; cmdVel.rl = -0.7;
                                }
                                else
                                {
                                    cmdVel.fl =  0.7; cmdVel.fr = -0.7;
                                    cmdVel.rr = -0.7; cmdVel.rl =  0.7;
                                }
                            }
                        }
                        
                        // Publish our motor commands
                        this->motorVelPub->publish(cmdVel);
                    }
                );

                // Setup our publisher for speed
                this->motorVelPub = this->create_publisher<interfaces::msg::F64vel4>("/motorControl", 10);
            }

        private:
            // Assign our subscriptions and publishers on the class's stack since this is just a testing dummy controller
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr cmdVelSub;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr simTimeSubscription;
            rclcpp::Publisher<interfaces::msg::F64vel4>::SharedPtr motorVelPub;

            // Initialise some things
            float simTime = 0;
            float previousImpact = 0;
            // Should we turn left or right?
            bool  correctLeft = false;
            // Make sure we evaluate this at compile time
            constexpr static float IMPACT_FREEZE_TIME  = 0.15;
            constexpr static float IMPACT_DELAY_TIME   = 0.5 + IMPACT_FREEZE_TIME;
            constexpr static float IMPACT_ADJUST_TIME  = 4.0 + IMPACT_DELAY_TIME;
            float random_IMPACT_ADJUST_TIME = IMPACT_ADJUST_TIME;

            // The random engine
            std::mt19937 rng;
    };


} // End namespace ros2_whisker_slam

#endif // H_599955_SRC_SIMPLE_CONTROLLER