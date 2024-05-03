#pragma once
#include <ariac_msgs/msg/kitting_task.hpp>
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

