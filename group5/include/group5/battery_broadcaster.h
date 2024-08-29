/**
 * @file battery_broadcaster.h
 * @brief Header file for the BatteryBroadcaster class.
 */
#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include "utils.h"
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"

using namespace std::chrono_literals;

namespace turtle{
/**
 * @class BatteryBroadcaster
 * @brief BatteryBroadcaster class responsible for broadcasting battery transforms.
 */
    class BatteryBroadcaster : public rclcpp::Node
    {
        public:
        /**
         * @brief Constructor for the BatteryBroadcaster class.
         *
         * @param node_name The name of the node.
         */
            BatteryBroadcaster(std::string node_name) : Node(node_name)
            {

                RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");

                battery_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/advanced_logical_camera/image",rclcpp::SensorDataQoS(),
                std::bind(&BatteryBroadcaster::battery_sub_cb , this , std::placeholders::_1));

                tf_broadcaster_battery_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
                tf_buffer_battery_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_buffer_battery_->setUsingDedicatedThread(true);
            }


        private:
            /**
             * @brief Callback function for the battery subscription.
             *
             * @param msg The received message from the logical camera.
             */
            void battery_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg);


            rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription_;

            //<! Buffer that stores several seconds of transforms for easy lookup by the listener. 
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_battery_;

            //<! Broadcaster object 
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_battery_;

            //<! Wall timer object for the broadcaster
            rclcpp::TimerBase::SharedPtr tf_timer_battery_;

            //<! x-coordinate of battery position.
            float bx_; 

            //<! y-coordinate of battery position. 
            float by_; 

            //<! z-coordinate of battery position.
            float bz_;  

            //<! x of battery orientation quaternion.
            float bq_x_;   

            //<! y of battery orientation quaternion. 
            float bq_y_; 

            //<! z of battery orientation quaternion.
            float bq_z_;  

            //<! w of battery orientation quaternion. 
            float bq_w_; 


    };
}