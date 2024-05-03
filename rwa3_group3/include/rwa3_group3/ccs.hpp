#pragma once

#include <ariac_msgs/msg/agv_status.hpp>
#include <ariac_msgs/msg/competition_state.hpp>
#include <ariac_msgs/msg/kitting_task.hpp>
#include <ariac_msgs/msg/order.hpp>
#include <ariac_msgs/srv/move_agv.hpp>
#include <ariac_msgs/srv/submit_order.hpp>
#include <chrono>  // Include the chrono library for time related functions
#include <queue>
#include <rclcpp/rclcpp.hpp>  // Include the ROS2 C++ Library
#include <std_srvs/srv/trigger.hpp>

#include "../include/rwa3_group3/orders.hpp"
#include <queue>
#include <future>
#include <functional>
#include <vector>


#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define BLUE "\033[1;34m"
#define MAGENTA "\033[1;35m"
#define RESET "\033[0m"

#define LOCK "\xF0\x9F\x94\x92"
#define CLOCK "\xF0\x9F\x95\x91"
#define TIME "\xE2\x8C\x9B"
#define AGV "\xF0\x9F\x9A\x95"
#define CHECK "\xE2\x9C\x85"
#define ORDER "\xF0\x9F\x93\xA6"
#define START "\xF0\x9F\x8F\x81"
#define END "\xF0\x9F\x94\xB4"
#define MEMO "\xF0\x9F\x93\x9D"
#define PRIORITY "\xF0\x9F\x9A\xA8"

class CCS : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new CCS object
   *
   * @param node_name
   */
  CCS(std::string node_name);

  /**
   * @brief Callback function for the timer_
   * This is the main function that manages the competition
   * It makes the decision for the competition based on
   * flags and counters like a state machine
   *
   */
  void Manager();

  // Callback Functions

  /**
   * @brief Call back function for the competition state
   * Binded to the '/ariac/competition_state' topic subscriber
   * Updates the global variable 'competition_state_'
   * @param msg ariac_msgs::msg::CompetitionState
   */
  void UpdateCompetitionState(
      const ariac_msgs::msg::CompetitionState::SharedPtr msg);

  /**
   * @brief Call back function for the order announcement
   * Bind to the '/ariac/orders' topic subscriber
   * Stores the order in the 'order_queue' priority queue
   * @param msg ariac_msgs::msg::Order
   */
  void StoreOrder(const ariac_msgs::msg::Order::SharedPtr msg);

  /**
   * @brief Call back function for the agv1 status
   * Bind to the '/ariac/agv1_status' topic subscriber
   * Uses the agv status to submit the order
   * @param msg ariac_msgs::msg::AGVStatus
   */
  void UpdateAGV1Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg);

  /**
   * @brief Call back function for the agv2 status
   * Bind to the '/ariac/agv2_status' topic subscriber
   * Uses the agv status to submit the order
   * @param msg ariac_msgs::msg::AGVStatus
   */
  void UpdateAGV2Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg);

  /**
   * @brief Call back function for the agv3 status
   * Bind to the '/ariac/agv3_status' topic subscriber
   * Uses the agv status to submit the order
   * @param msg ariac_msgs::msg::AGVStatus
   */
  void UpdateAGV3Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg);

  /**
   * @brief Call back function for the agv4 status
   * Bind to the '/ariac/agv4_status' topic subscriber
   * Uses the agv status to submit the order
   * @param msg ariac_msgs::msg::AGVStatus
   */
  void UpdateAGV4Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg);

  /**
   * @brief Callback for the move agv service future
   * Handles the asynchronous response from the '/ariac/move_agv{n}' service
   *
   * @param future The future object to handle the response from the service
   */
  void AGVResponseCallback(
      rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future);

  ///////////// Service Call Functions //////////////

  /**
   * @brief Start the competition
   * Cretaes a client to call the '/ariac/start_competition' service
   * Makes a synchronous call to the service and returns the response
   * @return true If the service call was successful
   * @return false If the service call was unsuccessful
   */
  bool StartCompetition();

  /**
   * @brief End the competition
   * Cretaes a client to call the '/ariac/end_competition' service
   * Makes a synchronous call to the service and returns the response
   * @return true If the service call was successful
   * @return false If the service call was unsuccessful
   */
  bool EndCompetition();

  /**
   * @brief Locks the tray on the AGV
   * Cretaes a client to call the '/ariac/agv{n}_lock_tray' service
   * Makes a synchronous call to the service and returns the response
   * @param number The number of the AGV to lock
   * @return true If the service call was successful
   * @return false If the service call was unsuccessful
   */
  bool LockAGV(int number);

  /**
   * @brief Move the AGV to the destination
   * Cretaes a client to call the '/ariac/move_agv{n}' service
   * Makes a asynchronous call to the service with a callback function
   * Runs the callback function when the service call is complete
   *
   * @param number  The number of the AGV to move
   * @param destination The destination to move the AGV to
   */
  void MoveAGV(int number, int destination);

  /**
   * @brief Submits the order to the competition
   * Creates a client to call the '/ariac/submit_order' service
   * Makes a synchronous call to the service and returns the response
   * @param order_id ID of the order to submit
   * @return true If the service call was successful
   * @return false If the service call was unsuccessful
   */
  bool SubmitOrder(std::string order_id);

  ///////////// Helper Functions //////////////

  /**
   * @brief Process the order
   * Locks the AGV, moves the AGV to the destination and submits the order
   * @param o Details of the order to process
   */
  void ProcessOrder(Order o);

  /**
   * @brief Select the order to process
   * Uses the priority queue to select the order to process
   * Selects the order with the highest priority
   * Uses correponding AGV to process the order
   */
  void SelectOrder();

  /**
   * @brief Checks the status of the AGV
   * Checks the location and velocity of the AGV
   * If the AGV is at the destination, and the velocity is 0
   * Submits the order to the competition
   * @param agv
   * @param location
   * @param vel
   */
  void ProcessAGVStatus(int agv, int location, float vel);

 private:
  ///////////// Timers //////////////

  // Timer to manage the competition
  rclcpp::TimerBase::SharedPtr timer_;

  ///////////// Callback Groups //////////////

  // Callback group for the processing the orders
  rclcpp::CallbackGroup::SharedPtr order_process_group_;

  // Callback group for storing the orders
  rclcpp::CallbackGroup::SharedPtr order_storing_group_;

  // Callback group for the synchronous general service callback
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  // Callback group for the synchronous lock agv1 service callback
  rclcpp::CallbackGroup::SharedPtr agv1_callback_group_;

  // Callback group for the synchronous lock agv2 service callback
  rclcpp::CallbackGroup::SharedPtr agv2_callback_group_;

  // Callback group for the synchronous lock agv3 service callback
  rclcpp::CallbackGroup::SharedPtr agv3_callback_group_;

  // Callback group for the synchronous lock agv4 service callback
  rclcpp::CallbackGroup::SharedPtr agv4_callback_group_;

  // Callback group for the agv_status subscribers
  rclcpp::CallbackGroup::SharedPtr agv_status_group_;

  ///////////// Subscribers //////////////

  // Subscriber for /ariac/competition_state
  rclcpp::Subscription<ariac_msgs::msg::CompetitionState>::SharedPtr
      c_state_sub_;

  // Subcriber for /ariac/orders
  rclcpp::Subscription<ariac_msgs::msg::Order>::SharedPtr order_subcriber_;

  // Subscriber for /ariac/agv1_status
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv1_subcriber_;

  // Subscriber for /ariac/agv2_status
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv2_subcriber_;

  // Subscriber for /ariac/agv3_status
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv3_subcriber_;

  // Subscriber for /ariac/agv4_status
  rclcpp::Subscription<ariac_msgs::msg::AGVStatus>::SharedPtr agv4_subcriber_;

  ///////////// Flags //////////////

  // Flag to keep track of the competition state
  int competiton_state_ = 0;

  // Flags to keep track of the AGV status
  std::vector<bool> agv_flag_ = {false, false, false, false};

  ///////////////// Counters /////////////////

  // Count to keep track of the number of orders
  int order_count_ = 0;

  // Count to keep track of the number of orders submitted
  int submit_count_ = 0;

  // Count to keep track of the priority orders
  int priority_order_count_ = 0;

  ///////////////// Data Structures /////////////////

  // Priority Queue to store the orders
  std::priority_queue<Order> order_queue;

    // Map to store the AGV and its corresponding order
    std::unordered_map<int, Order> agv_map;

    // Vector to store the futures of the order processing thread
    std::vector<std::future<void>> futures;
    
};
