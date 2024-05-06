#include "final_group3/utils.hpp"


geometry_msgs::msg::Quaternion Utils::set_quaternion_from_euler(double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    geometry_msgs::msg::Quaternion q_msg;

    q.setRPY(roll, pitch, yaw);

    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();
    q_msg.w = q.w();

    return q_msg;
}

std::array<double, 3> Utils::set_euler_from_quaternion(tf2::Quaternion quaternion)
{
    double roll;
    double pitch;
    double yaw;
    tf2::Matrix3x3 matrix(quaternion);
    matrix.getRPY(roll, pitch, yaw);
    // create an empty array of doubles to store the roll, pitch, yaw
    std::array<double, 3> rpy;
    rpy.at(0) = roll;
    rpy.at(1) = pitch;
    rpy.at(2) = yaw;

    return rpy;
}


geometry_msgs::msg::Pose Utils::build_pose(double x, double y, double z, geometry_msgs::msg::Quaternion orientation)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation = orientation;
        return pose;
    }


std::string Utils::log_pose_to_console(geometry_msgs::msg::Pose pose)
    {
        auto qx = pose.orientation.x;
        auto qy = pose.orientation.y;
        auto qz = pose.orientation.z;
        auto qw = pose.orientation.w;

        auto px = pose.position.x;
        auto py = pose.position.y;
        auto pz = pose.position.z;

        tf2::Quaternion q(qx, qy, qz, qw);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        roll *= 180 / M_PI;
        pitch *= 180 / M_PI;
        yaw *= 180 / M_PI;

        std::string output = "";
        output += "xyz: [" + std::to_string(px) + ",";
        output += std::to_string(py) + ",";
        output += std::to_string(pz) + "]\n";
        output += "rpy: [" + std::to_string(roll) + ",";
        output += std::to_string(pitch) + ",";
        output += std::to_string(yaw) + "]\n";
        output += "xyzw: [" + std::to_string(qx) + ",";
        output += std::to_string(qy) + ",";
        output += std::to_string(qz) + ",";
        output += std::to_string(qw) + "]\n";

        return output;
    }


    double Utils::get_yaw_from_pose_(geometry_msgs::msg::Pose pose)
    {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        return yaw;
    }


    geometry_msgs::msg::Pose Utils::multiply_poses(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2)
    {
        KDL::Frame frame1;
        KDL::Frame frame2;

        tf2::fromMsg(pose1, frame1);
        tf2::fromMsg(pose2, frame2);

        KDL::Frame frame3 = frame1 * frame2;

        return tf2::toMsg(frame3);
    }

    std::array<double, 3> Utils::get_euler_from_quaternion(tf2::Quaternion quaternion)
    {
        double roll;
        double pitch;
        double yaw;
        tf2::Matrix3x3 matrix(quaternion);
        matrix.getRPY(roll, pitch, yaw);
        // create an empty array of doubles to store the roll, pitch, yaw
        std::array<double, 3> rpy;
        rpy.at(0) = roll;
        rpy.at(1) = pitch;
        rpy.at(2) = yaw;

        return rpy;
    }

    geometry_msgs::msg::Quaternion Utils::get_quaternion_from_euler(double roll, double pitch, double yaw)
    {
        tf2::Quaternion q;
        geometry_msgs::msg::Quaternion q_msg;

        q.setRPY(roll, pitch, yaw);

        q_msg.x = q.x();
        q_msg.y = q.y();
        q_msg.z = q.z();
        q_msg.w = q.w();

        return q_msg;
    }