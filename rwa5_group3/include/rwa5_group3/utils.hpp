#pragma once

#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>

// create a utility class
class Utils
{
public:
    Utils(){}
    ~Utils(){}

    /**
     * @brief Get the quaternion from euler object
     *
     * @param roll  roll angle in radians
     * @param pitch  pitch angle in radians
     * @param yaw  yaw angle in radians
     * @return geometry_msgs::msg::Quaternion  Quaternion from rpy
     */
    geometry_msgs::msg::Quaternion set_quaternion_from_euler(double roll, double pitch, double yaw);
    
    /**
     * @brief Get the euler from quaternion object
     *
     * @param quaternion
     * @param roll
     * @param pitch
     * @param yaw
     * @return array[roll, pitch, yaw]
     */
    std::array<double, 3> set_euler_from_quaternion(tf2::Quaternion quaternion);

    geometry_msgs::msg::Quaternion get_quaternion_from_euler(double roll, double pitch, double yaw);

    std::array<double, 3> get_euler_from_quaternion(tf2::Quaternion quaternion);

    geometry_msgs::msg::Pose multiply_poses(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2);

    geometry_msgs::msg::Pose build_pose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation);

    std::string log_pose_to_console(geometry_msgs::msg::Pose pose);

    double get_yaw_from_pose_(geometry_msgs::msg::Pose pose);

};


