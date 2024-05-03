#include "rwa5_group3/sensors.hpp"
#include <string>
#include <vector>

Sensors::Sensors(std::string node_name)
    : Node(node_name)
{   


    // Set QoS profile
    rclcpp::QoS qos_profile(10);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
    qos_profile.keep_last(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::SystemDefault);

    left_bin_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/left_bins_camera/image", qos_profile, std::bind(&Sensors::LeftBinCameraCallback, this, std::placeholders::_1));

    right_bin_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/right_bins_camera/image", qos_profile, std::bind(&Sensors::RightBinCameraCallback, this, std::placeholders::_1));

    kits_tray1_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts1_camera/image", qos_profile, std::bind(&Sensors::KitsTray1CameraCallback, this, std::placeholders::_1));

    kits_tray2_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/ariac/sensors/kts2_camera/image", qos_profile, std::bind(&Sensors::KitsTray2CameraCallback, this, std::placeholders::_1));

    part_publisher_ = this->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/rwa_group3/detected_parts", qos_profile);
}


void Sensors::LeftBinCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{   
    std::vector<ariac_msgs::msg::PartPose> detected_parts;
    if (!left_bin_read_){
    geometry_msgs::msg::Pose sensor_pose = msg->sensor_pose;
    for(auto part_pose: msg->part_poses){
        ariac_msgs::msg::Part part = part_pose.part;
        geometry_msgs::msg::Pose pose = multiply_kdl_frames(sensor_pose,part_pose.pose);
        ariac_msgs::msg::PartPose detected_part;
        detected_part.part = part;
        detected_part.pose = pose;
        detected_parts.push_back(detected_part);
    }
    ariac_msgs::msg::AdvancedLogicalCameraImage detected_parts_msg;
    detected_parts_msg.part_poses = detected_parts;
    part_publisher_->publish(detected_parts_msg);
    left_bin_read_ = true;
    
    }
}

void Sensors::RightBinCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{   

    std::vector<ariac_msgs::msg::PartPose> detected_parts;
    if (!right_bin_read_){
        geometry_msgs::msg::Pose sensor_pose = msg->sensor_pose;
        for(auto part_pose: msg->part_poses){
        ariac_msgs::msg::Part part = part_pose.part;
        geometry_msgs::msg::Pose pose = multiply_kdl_frames(sensor_pose,part_pose.pose);
        ariac_msgs::msg::PartPose detected_part;
        detected_part.part = part;
        detected_part.pose = pose;
        detected_parts.push_back(detected_part);
    }
    ariac_msgs::msg::AdvancedLogicalCameraImage detected_parts_msg;
    detected_parts_msg.part_poses = detected_parts;
    part_publisher_->publish(detected_parts_msg);
    right_bin_read_ = true;
    }
}


void Sensors::KitsTray1CameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{   
    std::vector<ariac_msgs::msg::KitTrayPose> detected_trays;
    if (!kits_tray1_read_){
    geometry_msgs::msg::Pose sensor_pose = msg->sensor_pose;
    for(auto tray_pose: msg->tray_poses){
        int tray_id = tray_pose.id;
        geometry_msgs::msg::Pose pose = multiply_kdl_frames(sensor_pose,tray_pose.pose);
        ariac_msgs::msg::KitTrayPose detected_tray;
        detected_tray.id = tray_id;
        detected_tray.pose = pose;
        detected_trays.push_back(detected_tray);
    }
    ariac_msgs::msg::AdvancedLogicalCameraImage detected_trays_msg;
    detected_trays_msg.tray_poses = detected_trays;
    part_publisher_->publish(detected_trays_msg);
    kits_tray1_read_ = true;
    
    }
}

void Sensors::KitsTray2CameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{   

    std::vector<ariac_msgs::msg::KitTrayPose> detected_trays;
    if (!kits_tray2_read_){
    geometry_msgs::msg::Pose sensor_pose = msg->sensor_pose;
    for(auto tray_pose: msg->tray_poses){
        int tray_id = tray_pose.id;
        geometry_msgs::msg::Pose pose = multiply_kdl_frames(sensor_pose,tray_pose.pose);
        ariac_msgs::msg::KitTrayPose detected_tray;
        detected_tray.id = tray_id;
        detected_tray.pose = pose;
        detected_trays.push_back(detected_tray);
    }
    ariac_msgs::msg::AdvancedLogicalCameraImage detected_trays_msg;
    detected_trays_msg.tray_poses = detected_trays;
    part_publisher_->publish(detected_trays_msg);
    kits_tray2_read_ = true;
    
    }
}



geometry_msgs::msg::Pose Sensors::multiply_kdl_frames(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2)
{
    KDL::Frame frame1;
    KDL::Frame frame2;

    tf2::fromMsg(pose1, frame1);
    tf2::fromMsg(pose2, frame2);

    KDL::Frame frame3 = frame1 * frame2;

    return tf2::toMsg(frame3);
}


//=================================================================================================
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto sensor_node = std::make_shared<Sensors>("sensor_manager_node");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(sensor_node);
  executor.spin();  // This will start the execution
  rclcpp::shutdown();
}