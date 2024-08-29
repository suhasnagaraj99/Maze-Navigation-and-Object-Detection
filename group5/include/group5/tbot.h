/**
 * @file turtle_ps.h
 * @brief Header file for the TurtlePS class.
 */
#pragma once
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "utils.h"
#include <rosgraph_msgs/msg/clock.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;



namespace turtle{
    /**
     * @class TurtlePS
     * @brief A ROS2 node for controlling a turtlebot 
     */
class TurtlePS: public rclcpp::Node{
    public:
    /**
     * @brief Constructor for the TurtlePS class.
     *
     * @param node_name The name of the ROS2 node.
     */
    TurtlePS(std::string node_name):
    
    Node(node_name){

        //<! Declaring the parameters
        this->declare_parameter("aruco_marker_0","right_90");
        this->declare_parameter("aruco_marker_1","left_90");
        this->declare_parameter("aruco_marker_2","end");

        //<! Geting the parameters and storing it in attributes
        aruco_zero_=this->get_parameter("aruco_marker_0").as_string();
        aruco_one_=this->get_parameter("aruco_marker_1").as_string();
        aruco_two_=this->get_parameter("aruco_marker_2").as_string();

        turtlebot_timer_ = this->create_wall_timer(std::chrono::seconds(1) , std::bind(&TurtlePS::motion_cb, this));

        publisher_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
     
        aruco_subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(),
        std::bind(&TurtlePS::aruco_sub_cb , this , std::placeholders::_1));
      
        end_publisher_ = create_publisher<std_msgs::msg::Bool>("end_topic", 10);
        
       
        aruco_tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        aruco_transform_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*aruco_tf_buffer_);


        aruco_listen_timer_ = this->create_wall_timer(100ms, std::bind(&TurtlePS::aruco_listen_timer_cb_, this));

        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("clock",rclcpp::SensorDataQoS(),
        std::bind(&TurtlePS::clock_cb , this , std::placeholders::_1));

        turtle::TurtlePS::decision_vector={1.2,1.0,0.25,0.6,1.0,1.2,1.0,1.5};

    }

    private:

    // ==================== methods ====================
    /**
     * @brief Callback function for handling Aruco markers subscription.
     *
     * @param msg The received Aruco markers message.
     */
    void aruco_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg);

    /**
     * @brief Callback function for handling motion timer.
     */
    void motion_cb();  
    /**
     * @brief Callback function for handling clock subscription.
     *
     * @param msg The received clock message.
     */
    void clock_cb(const rosgraph_msgs::msg::Clock msg);
    /**
     * @brief Listen to a transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void aruco_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Timer to listen to the transform
     *
     */
    void aruco_listen_timer_cb_();


    // ==================== attributes ====================

    //<! Store the detected aruco marker type 
    int aruco_id_;

    //<! For indexing
    int aruco_index_=0;

    //<! flag variable to check if the robot is rotating 
    bool is_rotating_=false;

    //<! Twist message for giving linear velocity
    geometry_msgs::msg::Twist linear_;

    //<! Twist message for giving angular velocity
    geometry_msgs::msg::Twist angular_;

    //<! Bool message object
    std_msgs::msg::Bool bool_message_;

    //<! Distance between tutlebot and detected aruco marker
    float listner_distance_;

    //<! Attribute stroting aruco marker zero id
    std::string aruco_zero_;

    //<! Attribute stroting aruco marker one id
    std::string aruco_one_;

    //<! Attribute stroting aruco marker two id
    std::string aruco_two_;

    //<! Attribute for comparing aruco marker ids
    std::string aruco_decision_;

    //<! Attibute defing the rotation time
    int angle_time_;

    //<! Flag Variable for rotating right 
    bool is_rotating_right=false;

    //<! Flag Variable for rotating left
    bool is_rotating_left=false;

    //<! Attibute for storing simulation time
    float sim_time_=0;

    //<! Attibute for indexinf decision vector
    int decicon_index_=0;

    //<! Callback timer for controlling the turtlebot motion
    rclcpp::TimerBase::SharedPtr turtlebot_timer_;

    //<! Listner objects for listening to Aruco Markers Transformation 
    std::unique_ptr<tf2_ros::Buffer> aruco_tf_buffer_;

    //<! Transform listener object for kistening to aruco marker transform
    std::shared_ptr<tf2_ros::TransformListener> aruco_transform_listener_{nullptr};

    //<! timer to listen to the aruco transforms
    rclcpp::TimerBase::SharedPtr aruco_listen_timer_;

    //<! To set the decision before turning 
    //(hard coded upon professor's permission)
    std::vector<float> decision_vector;

    //<! Publisher for publishing velocity values to turtlebot
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    //<! Publisher for publishing when the maze navigation ends  
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr end_publisher_;

    //<! Subscription for Aruco Markers topic   
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;

    //<! Subscription for Clock topic to get simulation time
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;
};
}
