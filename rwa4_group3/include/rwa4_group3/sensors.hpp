#pragma once
#include <map>
#include <rclcpp/rclcpp.hpp>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include "rwa4_group3/utils.hpp"

class Sensors : public rclcpp::Node {
 public:
   Sensors(std::string node_name);


   void LeftBinCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

   void RightBinCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
   
   void KitsTray1CameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

   void KitsTray2CameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

   geometry_msgs::msg::Pose multiply_kdl_frames(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2);


 private:

   /*!< Utils object to access utility functions*/
   std::shared_ptr<Utils> utils_ptr_;

   rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bin_subscriber_;

   rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bin_subscriber_;
   
   rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kits_tray1_subscriber_;

   rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr kits_tray2_subscriber_;

   rclcpp::Publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_publisher_;


   std::map<int, std::string> color = {
       {0, "RED"},
       {1, "GREEN"},
       {2, "BLUE"},
       {3, "ORANGE"},
       {4, "PURPLE"}
     };
   
   std::map<int, std::string> type = {
       {10, "BATTERY"},
       {11, "PUMP"},
       {12, "SENSOR"},
       {13, "REGULATOR"}
     };

   bool left_bin_read_ = false;
   bool right_bin_read_ = false;
   bool kits_tray1_read_ = false;
   bool kits_tray2_read_ = false;

};