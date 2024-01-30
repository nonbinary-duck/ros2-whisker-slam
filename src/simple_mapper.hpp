#pragma once
#ifndef H_599955_SRC_SIMPLE_MAPPER
#define H_599955_SRC_SIMPLE_MAPPER 1


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
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

namespace ros2_whisker_slam
{
    
    /**
     * @brief A simple map builder. This does not work.
     */
    class SimpleMap : public rclcpp::Node
    {
        public:
            // Do all of the setup for our node in a nice and untidy inline constructor,
            // entirely defeating the organisational power provided by objects
            inline SimpleMap() : Node("simple_mapper")
            {
                // Initialise the subscribers for this node
                // We publish the X, Y, and Z coordinates from Coppelia as individual topics
                // which is a mistake. Using 3 different packets to represent the same state
                // gives us a race condition since the oder in which we receive X,Y,Z and
                // collision states is potentially non-deterministic
                this->simXSubscription = this->create_subscription< std_msgs::msg::Float32 >(
                    // The topic to subscribe to
                    "robo_x",
                    // qos?
                    10,
                    // The callback
                    [this](std_msgs::msg::Float32::UniquePtr msg)
                    {
                        this->roboPos[0] = msg->data;
                    }
                );

                this->simYSubscription = this->create_subscription< std_msgs::msg::Float32 >(
                    // The topic to subscribe to
                    "robo_y",
                    // qos?
                    10,
                    // The callback
                    [this](std_msgs::msg::Float32::UniquePtr msg)
                    {
                        this->roboPos[1] = msg->data;
                    }
                );

                this->simZSubscription = this->create_subscription< std_msgs::msg::Float32 >(
                    // The topic to subscribe to
                    "robo_z",
                    // qos?
                    10,
                    // The callback
                    [this](std_msgs::msg::Float32::UniquePtr msg)
                    {
                        this->roboPos[2] = msg->data;
                    }
                );

                // Setup the listener for the sensor
                this->sensorSubscription = this->create_subscription< std_msgs::msg::Bool >(
                    // The topic to subscribe to
                    "sensorTrigger",
                    // qos?
                    10,
                    // The callback
                    [this](std_msgs::msg::Bool::UniquePtr msg)
                    {
                        if (!msg->data) return;
                        
                        // If we've collided with something, mark its location on the map
                        // This isn't a well defined mapper, and our determineMapCoords
                        // can throw exceptions so we catch them, print them and ignore them (for debugging)
                        try
                        {
                            // Note that we're making a point on the map
                            // More or less all of the logging is related to debugging
                            RCLCPP_INFO(this->get_logger(), "Map");
                            // Get the coordinates of our collision
                            auto coords = this->determineMapCoords(this->roboPos[0], this->roboPos[1]);
                            
                            // Set the point in the map to true
                            map->at(coords[1]).at(coords[0]) = 1;

                            // Print our updated map
                            RCLCPP_INFO(this->get_logger(), this->printMap().c_str());
                        }
                        catch(const std::exception& e)
                        {
                            // Print to stderr the error (hopefully not) returned by determineMapCoords(F;F)obj;
                            std::cerr << e.what() << std::endl;
                        }
                        
                    }
                );

                // Init map
                this->map = new std::vector<std::vector<bool>>( SimpleMap::MAP_HEIGHT );

                // Perhaps there is a better way of making a two-dimensional vector?
                for (size_t j = 0; j < SimpleMap::MAP_HEIGHT; j++)
                {
                    this->map->at(j) = std::vector<bool>(SimpleMap::MAP_WIDTH);
                }
                
                // These should really be constants evaluated at compile time
                this->mapSize[0]   = SimpleMap::MAP_WIDTH * SimpleMap::MAP_SCALE;
                this->mapSize[1]   = SimpleMap::MAP_HEIGHT * SimpleMap::MAP_SCALE;
                this->mapOrigin[0] = SimpleMap::MAP_CENTRE[0] - (this->mapSize[0] / 2);
                this->mapOrigin[1] = SimpleMap::MAP_CENTRE[1] - (this->mapSize[1] / 2);
            }

            inline ~SimpleMap()
            {
                // Remove our "map" from the heap
                delete this->map;
            }

            inline std::array<size_t, 2> determineMapCoords(float x, float y) const
            {
                // Figure out where these coords are relative to the origin
                float relX = x - this->mapOrigin[0];
                float relY = y - this->mapOrigin[1];
                
                // Write to the console where we've placed a point on the map, for debugging
                RCLCPP_INFO(this->get_logger(), (std::to_string(x) + ',' + std::to_string(y)).c_str());

                // These should both be non-negative
                if (relX < 0 || relY < 0) throw "Tried to map coordinate outside predefined map";

                // Return coordinates relative to the map
                // This statement is flawed in perhaps both logic and in C++ conventions
                // Using explicit narrowing would fix the C++ part but debugging this is irrelevant as
                // a different (using ROS2 properly) approach will be taken
                return { relX * SimpleMap::MAP_SCALE, relY * SimpleMap::MAP_SCALE};
            }

            inline const std::string printMap() const noexcept
            {
                std::string mOut = "\n";
                // Initially we tried to preallocate the output for our map
                // But the map wasn't working so we tried a simpler approach
                // However that seems to not work also
                // std::string buffer;
                // buffer.reserve(SimpleMap::MAP_WIDTH);

                for (size_t j = 0; j < SimpleMap::MAP_HEIGHT; j++)
                {
                    for (size_t i = 0; i < SimpleMap::MAP_WIDTH; i++)
                    {
                        // buffer[i] = (this->map->at(j).at(i))? '+' : '-';
                        // Mark a plus if there is a point, or a minus if none
                        mOut += (this->map->at(j).at(i))? '+' : '-';
                    }

                    // mOut += buffer + '\n';
                    mOut += '\n';
                }
                
                return mOut;
            }

        private:
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sensorSubscription;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr simXSubscription;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr simYSubscription;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr simZSubscription;

            float roboPos[3]                     = { 0.0, 0.0, 0.0 };
            float mapSize[2];
            float mapOrigin[2];
            std::vector<std::vector<bool>> *map;

            constexpr static float MAP_CENTRE[2] = { -0.7, 0.0 };
            constexpr static size_t MAP_HEIGHT   = 50;
            constexpr static size_t MAP_WIDTH    = 50;
            constexpr static float MAP_SCALE     = 0.1;
    };


} // End namespace ros2_whisker_slam

#endif // H_599955_SRC_SIMPLE_MAPPER