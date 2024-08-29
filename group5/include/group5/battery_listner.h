/**
 * @file battery_listener.h
 * @brief Header file for the BatteryListener class.
 */
#pragma once

#include <cmath>
#include "utils.h"
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace turtle{

/**
 * @class BatteryListener
 * @brief BatteryListener class responsible for listening to battery transforms.
 */
class BatteryListener : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the BatteryListener class.
     *
     * @param node_name The name of the node.
     */
    BatteryListener(std::string node_name) : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), "Listener demo started");

        // load a buffer of transforms
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        
        battery_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/advanced_logical_camera/image",rclcpp::SensorDataQoS(),
        std::bind(&BatteryListener::battery_sub_cb_two_, this , std::placeholders::_1));

        end_subscription_ = this->create_subscription<std_msgs::msg::Bool>("end_topic",10,
        std::bind(&BatteryListener::end_sub_cb , this , std::placeholders::_1));
        }
        


private:

    /**
     * @brief Listen to a transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void listen_transform(const std::string &source_frame, const std::string &target_frame);
 

     /**
     * @brief Callback function for handling Logical Camera subscription.
     *
     * @param msg The received message from the logical camera.
     */
    void battery_sub_cb_two_(mage_msgs::msg::AdvancedLogicalCameraImage msg);
    /**
     * @brief Callback function for handling subscription to know if the robot has reached the end point.
     *
     * @param msg The received message indicating the end.
     */
    void end_sub_cb(std_msgs::msg::Bool msg);  

    //<! Buffer that stores several seconds of transforms for easy lookup by the listener
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    //<! Transform listener object 
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};

    //<! Wall timer object
    rclcpp::TimerBase::SharedPtr listen_timer_;

    //<! Advanced logical camera subscription object
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr battery_subscription_;

    //<! End message subscription object   
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr end_subscription_;

    //<! X coordinate of the battery
    float battery_x_; 

    //<! Y coordinate of the battery 
    float battery_y_; 

    //<! Z coordinate of the battery 
    float battery_z_; 
 
    //<! Roll angle the battery
    double battery_roll_; 

    //<! Pitch angle the battery
    double battery_pitch_; 

    //<! Yaw angle the battery
    double battery_yaw_; 

    //<!Variable to store the quaternion z value
    double battery_qz_; 

    //<! Variable to store the object type
    float object_type_; 

    //<!Variable to store the object colour
    float object_color_;

    //<! Vectors to store detected part information  
    std::vector<std::vector<double>> type_vector_;
    std::vector<double> object_vector_;
    std::array<double,3> rollpitchyaw;

    //<! Utils object   
    Utils utils_;
    
    //<! Arrtibute to store part type name
    std::string part_type_;

    //<! Arrtibute to store part color name
    std::string part_color_;
    
};
}