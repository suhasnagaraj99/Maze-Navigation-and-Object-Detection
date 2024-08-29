#include <rclcpp/rclcpp.hpp>
#include "tbot.h"
#include <tf2/exceptions.h>
#include "utils.h"


void turtle::TurtlePS::motion_cb(){


    // Check if the robot is not rotating        
    if (!is_rotating_ ){

        linear_.linear.x=0.2;

        publisher_->publish(linear_);

        // Check if the robot has reached the decision point
        if (((listner_distance_<=decision_vector[decicon_index_]))  && listner_distance_!=0){
         
            angle_time_=sim_time_+10;

            is_rotating_=true;
    
            if(aruco_decision_=="end"){
                // Robot reached the goal
                RCLCPP_INFO_STREAM(this->get_logger(), "Reached Goal");

                bool_message_.data = true;

                end_publisher_->publish(bool_message_);         

                linear_.linear.x=0.0;

                publisher_->publish(linear_);             
            }
        }
    }
    // Check if the robot is rotating
    if (is_rotating_){

        linear_.linear.x=0.0;

        publisher_->publish(linear_);
        // Check if the robot needs to rotate right
        if(aruco_decision_=="right_90" || is_rotating_right==true){

            if(is_rotating_left==false){

                is_rotating_right=true;

                angular_.angular.z=-0.15708;

                publisher_->publish(angular_);

                if (sim_time_>=angle_time_){

                    angular_.angular.z=0.0;

                    publisher_->publish(angular_);

                    decicon_index_=decicon_index_+1;

                    is_rotating_=false;

                    is_rotating_right=false;

                }
            }
        }
        // Check if the robot needs to rotate left
        if(aruco_decision_=="left_90" || is_rotating_left==true){

            if(is_rotating_right==false){

                is_rotating_left=true;

                angular_.angular.z=0.15708;

                publisher_->publish(angular_);

                // Check if the rotation time has elapsed
                if (sim_time_>=angle_time_){

                    angular_.angular.z=0.0;

                    publisher_->publish(angular_);

                    decicon_index_=decicon_index_+1;

                    is_rotating_=false;

                    is_rotating_left=false;

                }
            }
        }
    }
    }

void turtle::TurtlePS::aruco_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers msg){

    aruco_id_=msg.marker_ids[aruco_index_];

    // Check the Aruco marker ID and set the decision accordingly
    if (aruco_id_==0 && listner_distance_<1.5){
        aruco_decision_=aruco_zero_;
    }
    else if (aruco_id_==1 && listner_distance_<1.5){
        aruco_decision_=aruco_one_;
    }
    else if (aruco_id_==2  && listner_distance_<1.5){
        aruco_decision_=aruco_two_;
    }
}

void turtle::TurtlePS::aruco_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped aruco_t_stamped;

    geometry_msgs::msg::Pose aruco_pose_out;

    try
    {
        aruco_t_stamped = aruco_tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 10ms);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // Extract translation and rotation
    aruco_pose_out.position.x = aruco_t_stamped.transform.translation.x;
    aruco_pose_out.position.y = aruco_t_stamped.transform.translation.y;
    aruco_pose_out.position.z = aruco_t_stamped.transform.translation.z;
    aruco_pose_out.orientation = aruco_t_stamped.transform.rotation;

    // Update the distance
    listner_distance_=aruco_pose_out.position.x;
}

void turtle::TurtlePS::aruco_listen_timer_cb_()
{
    aruco_listen_transform("camera_rgb_frame", "aruco_marker_frame");
}

void turtle::TurtlePS::clock_cb(const rosgraph_msgs::msg::Clock msg) {
    sim_time_=msg.clock.sec;
}


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtle::TurtlePS>("turtleps");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
