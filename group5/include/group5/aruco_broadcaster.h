/**
 * @file aruco_broadcaster.h
 * @brief Header file for the ArucoBroadcaster class.
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

using namespace std::chrono_literals;

namespace turtle{
/**
 * @class ArucoBroadcaster
 * @brief ArucoBroadcaster class responsible for broadcasting Aruco marker transforms.
 */
class ArucoBroadcaster : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the ArucoBroadcaster class.
     *
     * @param node_name The name of the node.
     */
    ArucoBroadcaster(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");
        subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",10,
        std::bind(&ArucoBroadcaster::aruco_sub_cb , this , std::placeholders::_1));


        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);

        broadcast_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&ArucoBroadcaster::broadcast_timer_cb_, this));
    }


private:
    /**
     * @brief Callback function for handling Aruco markers subscription.
     *
     * @param msg The received Aruco markers message.
     */    
    void aruco_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg);
    /**
     * @brief Timer callback for broadcasting the transform.
     */
    void broadcast_timer_cb_();


    //<! Subscripition to Aruco Markers Topic
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_;
    
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    /*!< Broadcaster object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    /*!< Wall timer object for the broadcaster*/
    rclcpp::TimerBase::SharedPtr broadcast_timer_;

    //<! x-coordinate of Aruco marker position. 
    float x_; 

    //<! y-coordinate of Aruco marker position.
    float y_;   

    //<! z-coordinate of Aruco marker position.
    float z_;  

    //<! x-coordinate of Aruco marker orientation quaternion.
    float q_x_;  

    //<! y-coordinate of Aruco marker orientation quaternion.
    float q_y_; 

    //<! z-coordinate of Aruco marker orientation quaternion.
    float q_z_; 

    //<! w-coordinate of Aruco marker orientation quaternion.
    float q_w_; 
};
}