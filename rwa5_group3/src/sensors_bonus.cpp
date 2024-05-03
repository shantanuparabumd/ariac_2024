#include "rwa5_group3/sensors_bonus.hpp"
#include <iostream>
#include <string>
#include <vector>

Sensors::Sensors(std::string node_name)
    : Node(node_name)
{   

    // Set the parameter use_sim_time to true
    rclcpp::ParameterValue use_sim_time_param(true);
    this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time_param));

    // Set QoS profile
    rclcpp::QoS qos_profile(10);
    qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
    qos_profile.keep_last(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::SystemDefault);

    left_bin_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/rwa_group3/sensors/left_bins_camera_yolo", qos_profile, std::bind(&Sensors::LeftBinCameraCallback, this, std::placeholders::_1));

    right_bin_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/rwa_group3/sensors/right_bins_camera_yolo", qos_profile, std::bind(&Sensors::RightBinCameraCallback, this, std::placeholders::_1));

    kits_tray1_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ariac/sensors/kts1_camera/rgb_image", qos_profile, std::bind(&Sensors::KitsTray1CameraCallback, this, std::placeholders::_1));

    kits_tray2_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/ariac/sensors/kts2_camera/rgb_image", qos_profile, std::bind(&Sensors::KitsTray2CameraCallback, this, std::placeholders::_1));


    kits_tray1_logical_subscriber_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
        "/ariac/sensors/kts1_camera_logical/image", qos_profile, std::bind(&Sensors::KitsTray1LogicalCameraCallback, this, std::placeholders::_1));

    kits_tray2_logical_subscriber_ = this->create_subscription<ariac_msgs::msg::BasicLogicalCameraImage>(
        "/ariac/sensors/kts2_camera_logical/image", qos_profile, std::bind(&Sensors::KitsTray2LogicalCameraCallback, this, std::placeholders::_1));

    part_publisher_ = this->create_publisher<ariac_msgs::msg::AdvancedLogicalCameraImage>(
        "/rwa_group3/detected_parts", qos_profile);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Sensors::PublishPartsAndTrays, this));

}


void Sensors::LeftBinCameraCallback(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{   
    std::vector<ariac_msgs::msg::PartPose> detected_parts;
    if (!left_bin_read_){
    geometry_msgs::msg::Pose sensor_pose = msg->sensor_pose;
    for(auto part_pose: msg->part_poses){
        ariac_msgs::msg::Part part = part_pose.part;
        geometry_msgs::msg::Pose pose = part_pose.pose;
        // RCLCPP_INFO_STREAM(this->get_logger(), type[part.type] << " " << color[part.color] 
        // << " at " << pose.position.x << " " << pose.position.y << " " << pose.position.z );
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
        geometry_msgs::msg::Pose pose = part_pose.pose;
        // RCLCPP_INFO_STREAM(this->get_logger(), type[part.type] << " " << color[part.color] 
        // << " at " << pose.position.x << " " << pose.position.y << " " << pose.position.z );
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


void Sensors::KitsTray1LogicalCameraCallback(const ariac_msgs::msg::BasicLogicalCameraImage::SharedPtr msg)
{   
    if (!kits_tray1_logical_read_){
        // RCLCPP_INFO(this->get_logger(), "Kits tray 1 logical camera callback");
    geometry_msgs::msg::Pose sensor_pose = msg->sensor_pose;
    for(auto tray_pose: msg->tray_poses){
        geometry_msgs::msg::Pose pose = multiply_kdl_frames(sensor_pose,tray_pose);
        // RCLCPP_INFO_STREAM(this->get_logger(),pose.position.x << " " << pose.position.y << " " << pose.position.z );
        detected_poses_kts1.push_back(pose);
    }
    
    kits_tray1_logical_read_ = true;
    
    }
}

void Sensors::KitsTray2LogicalCameraCallback(const ariac_msgs::msg::BasicLogicalCameraImage::SharedPtr msg)
{   
    if (!kits_tray2_logical_read_){
        // RCLCPP_INFO(this->get_logger(), "Kits tray 2 logical camera callback");
    geometry_msgs::msg::Pose sensor_pose = msg->sensor_pose;
    for(auto tray_pose: msg->tray_poses){
        geometry_msgs::msg::Pose pose = multiply_kdl_frames(sensor_pose,tray_pose);
        // RCLCPP_INFO_STREAM(this->get_logger(),pose.position.x << " " << pose.position.y << " " << pose.position.z );
        detected_poses_kts2.push_back(pose);
    }
    
    kits_tray2_logical_read_ = true;
    
    }
}


void Sensors::PublishPartsAndTrays(){

    if(kits_tray1_read_ && kits_tray1_logical_read_ && !kts1_published){
        std::vector<ariac_msgs::msg::KitTrayPose> detected_trays;
        for(int i = 0; i < detected_ids_kts1.size(); i++){
            ariac_msgs::msg::KitTrayPose detected_tray;
            detected_tray.id = detected_ids_kts1[i];
            detected_tray.pose = detected_poses_kts1[i];
            detected_trays.push_back(detected_tray);
            // RCLCPP_INFO_STREAM(this->get_logger(), "Tray ID:" << std::to_string(detected_ids_kts1[i])
            // << " at " << detected_poses_kts1[i].position.x << " " << detected_poses_kts1[i].position.y << " " << detected_poses_kts1[i].position.z );
        }
        ariac_msgs::msg::AdvancedLogicalCameraImage detected_trays_msg;
        detected_trays_msg.tray_poses = detected_trays;
        part_publisher_->publish(detected_trays_msg);
        kts1_published = true;

    }
    if(right_bin_read_ && kits_tray2_logical_read_ && !kts2_published){
        std::vector<ariac_msgs::msg::KitTrayPose> detected_trays;
        for(int i = 0; i < detected_ids_kts2.size(); i++){
            ariac_msgs::msg::KitTrayPose detected_tray;
            detected_tray.id = detected_ids_kts2[i];
            detected_tray.pose = detected_poses_kts2[i];
            detected_trays.push_back(detected_tray);
            // RCLCPP_INFO_STREAM(this->get_logger(), "Tray ID:" << std::to_string(detected_ids_kts2[i])
            // << " at " << detected_poses_kts2[i].position.x << " " << detected_poses_kts2[i].position.y << " " << detected_poses_kts2[i].position.z );
        }
        ariac_msgs::msg::AdvancedLogicalCameraImage detected_trays_msg;
        detected_trays_msg.tray_poses = detected_trays;
        part_publisher_->publish(detected_trays_msg);
        kts2_published = true;
    }
}


void Sensors::KitsTray1CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{   

    if(!kits_tray1_read_){
        kits_tray1_read_ = true;

        //Convert the ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e){
            RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;

        cv::Mat grayImg;
        cv::cvtColor(image, grayImg, cv::COLOR_BGR2GRAY);

        cv::Mat binaryImg;
        cv::threshold(grayImg, binaryImg, 128, 255, cv::THRESH_BINARY);

        // Extract regions of interest
        cv::Mat tag1 = image(cv::Rect(170, 216, 22, 22));
        cv::Mat tag2 = image(cv::Rect(310, 216, 22, 22));
        cv::Mat tag3 = image(cv::Rect(448, 216, 22, 22));
        
        int tag1_id = ImageToID(tag1);
        if (tag1_id>=0 && tag1_id<=9){
            detected_ids_kts1.push_back(tag1_id);
        }
        int tag2_id = ImageToID(tag2);
        if (tag2_id>=0 && tag2_id<=9){
            detected_ids_kts1.push_back(tag2_id);
        }
        int tag3_id = ImageToID(tag3);
        if (tag3_id>=0 && tag3_id<=9){
            detected_ids_kts1.push_back(tag3_id);
        }
    }
    

}

void Sensors::KitsTray2CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{   
    if(!kits_tray2_read_){
        kits_tray2_read_ = true;
        //Convert the ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e){
            RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;

        cv::Mat grayImg;
        cv::cvtColor(image, grayImg, cv::COLOR_BGR2GRAY);

        cv::Mat binaryImg;
        cv::threshold(grayImg, binaryImg, 128, 255, cv::THRESH_BINARY);

        // Extract regions of interest
        
        cv::Mat tag1 = image(cv::Rect(170, 216, 22, 22));
        
        cv::Mat tag2 = image(cv::Rect(310, 216, 22, 22));
        
        cv::Mat tag3 = image(cv::Rect(448, 216, 22, 22));
        


        int tag1_id = ImageToID(tag1);
        if(tag1_id>=0 && tag1_id<=9){
            detected_ids_kts2.push_back(tag1_id);
        }
        int tag2_id = ImageToID(tag2);
        if(tag2_id>=0 && tag2_id<=9){
            detected_ids_kts2.push_back(tag2_id);
        }
        int tag3_id = ImageToID(tag3);
        if (tag3_id>=0 && tag3_id<=9){
        detected_ids_kts2.push_back(tag3_id);
        }

    }
    
}


int Sensors::ImageToID(cv::Mat &image)
{
    int width = image.cols;
    int height = image.rows;
    int part_width = width / 4;
    int part_height = height / 4;
    cv::Mat id(4, 4, CV_8UC1, cv::Scalar(0));
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            cv::Mat part = image(cv::Rect(j * part_width, i * part_height, part_width, part_height));
            double mean = cv::mean(part)[0];
            if (mean > 145) {
                id.at<uchar>(i, j) = 1;
            } else {
                id.at<uchar>(i, j) = 0;
            }
        }
    }
    for (int i = 0; i < 10; ++i) {
        if (cv::countNonZero(id == aruco_dict[i]) == 16) {
            return i;
        }
    }
    return -1;
    
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