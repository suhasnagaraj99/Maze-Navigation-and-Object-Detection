#include "aruco_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// needed for the listener
#include <tf2/exceptions.h>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

void turtle::ArucoBroadcaster::broadcast_timer_cb_()
{   // Create a TransformStamped message
    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;

    // Set the header information
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "camera_rgb_optical_frame";
    dynamic_transform_stamped.child_frame_id = "aruco_marker_frame";

    // Set the translation and rotation values
    dynamic_transform_stamped.transform.translation.x = x_;
    dynamic_transform_stamped.transform.translation.y = y_;
    dynamic_transform_stamped.transform.translation.z = z_;
    dynamic_transform_stamped.transform.rotation.x = q_x_;
    dynamic_transform_stamped.transform.rotation.y = q_y_;
    dynamic_transform_stamped.transform.rotation.z = q_z_;
    dynamic_transform_stamped.transform.rotation.w = q_w_;
    // Send the transform
    tf_broadcaster_->sendTransform(dynamic_transform_stamped);
}

void turtle::ArucoBroadcaster::aruco_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg){
    x_=msg.poses[0].position.x;
    y_=msg.poses[0].position.y;
    z_=msg.poses[0].position.z;
    q_x_=msg.poses[0].orientation.x;
    q_y_=msg.poses[0].orientation.y;
    q_z_=msg.poses[0].orientation.z;
    q_w_=msg.poses[0].orientation.w;
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtle::ArucoBroadcaster>("aruco_broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}