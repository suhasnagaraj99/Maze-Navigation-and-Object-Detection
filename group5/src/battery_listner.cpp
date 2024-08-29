#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "battery_listner.h"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <std_msgs/msg/float64.hpp>
#include <tf2/exceptions.h>
using namespace std::chrono_literals;



void turtle::BatteryListener::battery_sub_cb_two_(mage_msgs::msg::AdvancedLogicalCameraImage msg){
    // Check if the received message contains battery information    
    if (!msg.part_poses.empty()) {

    // Update the TF listener for the battery frame
    listen_transform("odom", "battery_frame");

    object_type_ = int(msg.part_poses[0].part.type);

    object_color_ = int(msg.part_poses[0].part.color);

    if (type_vector_.empty()) {
        // If the type_vector_ is empty, add the battery information directly
        if(battery_x_){

        object_vector_.clear();

        object_vector_.push_back(object_type_);

        object_vector_.push_back(object_color_);

        object_vector_.push_back(battery_x_);

        object_vector_.push_back(battery_y_);

        object_vector_.push_back(battery_z_);

        object_vector_.push_back(battery_roll_);

        object_vector_.push_back(battery_pitch_);

        object_vector_.push_back(battery_yaw_);

        type_vector_.push_back(object_vector_);

        }
    } else {
        // If type_vector_ is not empty, check if the battery type and color are already present
        bool found = false;

        for (size_t i = 0; i < type_vector_.size(); ++i) {
            if (type_vector_[i][0] == object_type_ && type_vector_[i][1] == object_color_) {

                found = true;

                break;
            }
        }

        // If not found, add the battery information
        if (!found) {

            object_vector_.clear();

            object_vector_.push_back(object_type_);

            object_vector_.push_back(object_color_);

            object_vector_.push_back(battery_x_);

            object_vector_.push_back(battery_y_);

            object_vector_.push_back(battery_z_);

            object_vector_.push_back(battery_roll_);

            object_vector_.push_back(battery_pitch_);

            object_vector_.push_back(battery_yaw_);

            type_vector_.push_back(object_vector_);

        }
    }
}
}

void turtle::BatteryListener::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped battery_t_stamped;
    try
    {   // Try to look up the transform from source_frame to target_frame
        battery_t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 1ms);
    }
    catch (const tf2::TransformException &ex)
    {   // Handle exceptions (e.g., waiting for the transform)
        RCLCPP_INFO_STREAM(this->get_logger(), "waiting" );
    }
    
    // Extract Euler angles from the quaternion
    rollpitchyaw = utils_.set_euler_from_quaternion(tf2::Quaternion(battery_t_stamped.transform.rotation.x, battery_t_stamped.transform.rotation.y,battery_t_stamped.transform.rotation.z, battery_t_stamped.transform.rotation.w));

    // Update member variables with the latest battery pose
    battery_x_=float(battery_t_stamped.transform.translation.x);

    battery_y_=float(battery_t_stamped.transform.translation.y);

    battery_z_=float(battery_t_stamped.transform.translation.z);

    battery_roll_=float(rollpitchyaw[0]);
    
    battery_pitch_=float(rollpitchyaw[1]);

    battery_yaw_=float(rollpitchyaw[2]);
}

void turtle::BatteryListener::end_sub_cb(std_msgs::msg::Bool msg){
    // Check if the end signal is received
    if(msg.data==true){

        // Loop through the detected objects and print information
        for (auto x:type_vector_){    
            if (x[0]==mage_msgs::msg::Part::BATTERY){
                part_type_="Battery ";
            }
            if (x[0]==mage_msgs::msg::Part::PUMP){
                part_type_="Pump ";
            }
            if (x[0]==mage_msgs::msg::Part::SENSOR){
                part_type_="Sensor ";
            }
            if (x[0]==mage_msgs::msg::Part::REGULATOR){
                part_type_="Regulator ";
            }
            if (x[1]==mage_msgs::msg::Part::RED){
                part_color_="Red ";
            }
            if (x[1]==mage_msgs::msg::Part::BLUE){
                part_color_="Blue ";
            }
            if (x[1]==mage_msgs::msg::Part::GREEN){
                part_color_="Green ";
            }
            if (x[1]==mage_msgs::msg::Part::ORANGE){
                part_color_="Orange ";
            }
            if (x[1]==mage_msgs::msg::Part::PURPLE){
                part_color_="Purple ";
            }
            
            RCLCPP_INFO_STREAM(this->get_logger(), part_color_ << part_type_ << " detected at xyz=["<<x[2]<<","<<x[3]<<","<<x[4]<<"] "<<"rpy=["<<x[5]<<","<<x[6]<<","<<x[7]<<"]"<<'\n');
        }
        }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<turtle::BatteryListener>("battery_listner");
  rclcpp::spin(node);
  rclcpp::shutdown();
}