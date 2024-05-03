#pragma once
#include <map>
#include <rclcpp/rclcpp.hpp>

#include <ariac_msgs/msg/advanced_logical_camera_image.hpp>
#include <ariac_msgs/msg/basic_logical_camera_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/kit_tray_pose.hpp>
#include <ariac_msgs/msg/part.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <vector>

#include "cv_bridge/cv_bridge.h"  // Included for cv_bridge (OPENCV TO ROS)
#include <opencv2/highgui.hpp>  // Included for highgui  (OPENCV)

#include "rwa4_group3/utils.hpp"

class Sensors : public rclcpp::Node {
 public:
   Sensors(std::string node_name);


   void LeftBinCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

   void RightBinCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
   
   void KitsTray1CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

   void KitsTray2CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

   void KitsTray1LogicalCameraCallback(const ariac_msgs::msg::BasicLogicalCameraImage::SharedPtr msg);

   void KitsTray2LogicalCameraCallback(const ariac_msgs::msg::BasicLogicalCameraImage::SharedPtr msg);

   geometry_msgs::msg::Pose multiply_kdl_frames(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2);

   int ImageToID(cv::Mat &image);

   void PublishPartsAndTrays();


 private:

   /*!< Utils object to access utility functions*/
   std::shared_ptr<Utils> utils_ptr_;

   rclcpp::TimerBase::SharedPtr timer_;

   rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr left_bin_subscriber_;

   rclcpp::Subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr right_bin_subscriber_;
   
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kits_tray1_subscriber_;

   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr kits_tray2_subscriber_;

   rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr kits_tray1_logical_subscriber_;

   rclcpp::Subscription<ariac_msgs::msg::BasicLogicalCameraImage>::SharedPtr kits_tray2_logical_subscriber_;

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
   bool kits_tray1_logical_read_ = false;
   bool kits_tray2_logical_read_ = false;


   std::vector<cv::Mat> aruco_dict = {
    // Define all the IDs as cv::Mat objects
    (cv::Mat_<uchar>(4, 4) << 1, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0),
    (cv::Mat_<uchar>(4, 4) << 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0),
    (cv::Mat_<uchar>(4, 4) << 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1),
    (cv::Mat_<uchar>(4, 4) << 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 1, 1, 0),
    (cv::Mat_<uchar>(4, 4) << 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0),
    (cv::Mat_<uchar>(4, 4) << 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1),
    (cv::Mat_<uchar>(4, 4) << 1, 1, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0),
    (cv::Mat_<uchar>(4, 4) << 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0),
    (cv::Mat_<uchar>(4, 4) << 1, 1, 0, 0, 0, 1, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0),
    (cv::Mat_<uchar>(4, 4) << 1, 0, 0, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0),
   };



   std::vector<int> detected_ids_kts1;
   std::vector<int> detected_ids_kts2;

   std::vector<geometry_msgs::msg::Pose> detected_poses_kts1;
   std::vector<geometry_msgs::msg::Pose> detected_poses_kts2;

   bool kts1_published = false;
   bool kts2_published = false;

};