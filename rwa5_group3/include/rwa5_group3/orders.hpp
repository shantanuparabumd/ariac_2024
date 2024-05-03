#pragma once
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/assembly_task.hpp>
#include <ariac_msgs/msg/combined_task.hpp>
#include <ariac_msgs/msg/part_pose.hpp>
#include <ariac_msgs/msg/quality_issue.hpp>

#include <iostream>

/**
 * @brief Structure to store the data from orders
 *
 */
struct Order {

    std::string id;
    int type;
    int priority;
    int agv_number;
    int destination;

    int station;
    ariac_msgs::msg::KittingTask kitting_task;
    ariac_msgs::msg::AssemblyTask assembly_task;
    ariac_msgs::msg::CombinedTask combined_task;

    std::vector<ariac_msgs::msg::PartPose> parts_and_poses;

    
    bool operator<(const Order& other) const{
        return priority<other.priority;
    }
};


struct OrderStatus {
    bool valid_id;
    bool all_passed;
    bool incorrect_tray;
    ariac_msgs::msg::QualityIssue quadrant1;
    ariac_msgs::msg::QualityIssue quadrant2;
    ariac_msgs::msg::QualityIssue quadrant3;
    ariac_msgs::msg::QualityIssue quadrant4;

};

