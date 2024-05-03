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
    
    bool operator<(const Order& other) const{
        return priority<other.priority;
    }
};

