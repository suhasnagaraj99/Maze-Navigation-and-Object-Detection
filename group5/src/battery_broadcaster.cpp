#include "battery_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// needed for the listener
#include <tf2/exceptions.h>

// allows to use, 50ms, etc
using namespace std::chrono_literals;
// Callback function for the battery subscription.
void turtle::BatteryBroadcaster::battery_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    
    // Check if the received message has part poses
    if(!(msg.part_poses.empty())){

    geometry_msgs::msg::TransformStamped dynamic_transform_stamped;

    // Extract battery pose information from the message
    bx_=msg.part_poses[0].pose.position.x;
    by_=msg.part_poses[0].pose.position.y;
    bz_=msg.part_poses[0].pose.position.z;
    bq_x_=msg.part_poses[0].pose.orientation.x;
    bq_y_=msg.part_poses[0].pose.orientation.y;
    bq_z_=msg.part_poses[0].pose.orientation.z;
    bq_w_=msg.part_poses[0].pose.orientation.w;

    // Set header information for the transform
    dynamic_transform_stamped.header.stamp = this->get_clock()->now();
    dynamic_transform_stamped.header.frame_id = "logical_camera_link";
    dynamic_transform_stamped.child_frame_id = "battery_frame";

    // Set translation and rotation values for the transform
    dynamic_transform_stamped.transform.translation.x = bx_;
    dynamic_transform_stamped.transform.translation.y = by_;
    dynamic_transform_stamped.transform.translation.z = bz_;
    dynamic_transform_stamped.transform.rotation.x = bq_x_;
    dynamic_transform_stamped.transform.rotation.y = bq_y_;
    dynamic_transform_stamped.transform.rotation.z = bq_z_;
    dynamic_transform_stamped.transform.rotation.w = bq_w_;
    // Send the transform
    tf_broadcaster_battery_->sendTransform(dynamic_transform_stamped);
}
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  // Create an instance of the BatteryBroadcaster class
  auto node = std::make_shared<turtle::BatteryBroadcaster>("battery_broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}