/**
 * @file ccs.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @author Akashkumar Parmar (akasparm@umd.edu)
 * @brief This file generates a node, which calls upon a service to start the
 * competition, fetches orders published on a specific topic, treats them with
 * priority and submits accordingly with appropriate wait-time.
 * @version 0.1
 * @date 2024-03-19
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "rwa3_group3/ccs.hpp"

//=================================================================================================
CCS::CCS(std::string node_name) : Node(node_name) {
  // Create a MutEx Callback group to run the main function (manager)
  order_process_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create a MutEx Callback group for handling announced orders (StoreOrders)
  order_storing_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Timer to run the main (manager) function
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds((int)(1000.0)), std::bind(&CCS::Manager, this),
      order_process_group_);  //<< Using MutEx Callback

  // Subscriber for "ariac/competition_state"
  c_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
      "ariac/competition_state", 10,
      std::bind(&CCS::UpdateCompetitionState, this, std::placeholders::_1));

  // Add callback group to Subscriber (ariac/orders)
  rclcpp::SubscriptionOptions options;
  options.callback_group = order_storing_group_;

  // Subscriber for "ariac/orders"
  order_subcriber_ = this->create_subscription<ariac_msgs::msg::Order>(
      "ariac/orders", 10,
      std::bind(&CCS::StoreOrder, this, std::placeholders::_1),
      options  //<< Mutex Callback Group
  );

  // Create a MutEx Callback group for handling agv status updates
  agv_status_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add callback group to Subscriber (/ariac/agv{n}_status)
  rclcpp::SubscriptionOptions agv_options;
  agv_options.callback_group = agv_status_group_;

  // Subscriber for "/ariac/agv1_status"
  agv1_subcriber_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv1_status", 10,
      std::bind(&CCS::UpdateAGV1Status, this, std::placeholders::_1),
      agv_options);

  // Subscriber for "/ariac/agv2_status"
  agv2_subcriber_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv2_status", 10,
      std::bind(&CCS::UpdateAGV2Status, this, std::placeholders::_1),
      agv_options);

  // Subscriber for "/ariac/agv3_status"
  agv3_subcriber_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv3_status", 10,
      std::bind(&CCS::UpdateAGV3Status, this, std::placeholders::_1),
      agv_options);

  // Subscriber for "/ariac/agv4_status"
  agv4_subcriber_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv4_status", 10,
      std::bind(&CCS::UpdateAGV4Status, this, std::placeholders::_1),
      agv_options);
}

//=================================================================================================
void CCS::ProcessAGVStatus(int agv, int location, float vel) {
  // RCLCPP_INFO_STREAM(this->get_logger(), "AGV "<<agv<<" Location:
  // "<<location<<" Velocity: "<<vel);
  if (location == ariac_msgs::msg::AGVStatus::WAREHOUSE && !agv_flag_[agv-1]) {
    if (std::abs(vel) < 0.00025) {
        SubmitOrder(agv_map[agv].id);
        agv_flag_[agv-1] = true;
        submit_count_++;
        priority_order_count_--;
        RCLCPP_INFO_STREAM(this->get_logger(), "Order: " << agv_map[agv].id
                                                         << " on AGV " << agv
                                                         << " Submitted");
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Total Orders Submitted " << submit_count_);
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), RED << "AGV " << agv
                                                 << " has not stopped yet!!!"
                                                 << RESET);
    }
  }
//   else{
//     RCLCPP_INFO_STREAM(this->get_logger(),RED<<"AGV "<<agv<<" is not in the warehouse!!!"<<RESET);
//     RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Current Location: "<<location<<RESET);
//     RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Current Flag: "<<agv_flag_[agv-1]<<RESET);
//   }
}

//=================================================================================================
void CCS::Manager(){

    // Check if the competition is ready
    if(competiton_state_==1){
        RCLCPP_INFO_STREAM(this->get_logger(),
        YELLOW<<"Start the competition"<<RESET);
        // Start the competition
        StartCompetition();
    }
    // Check if the competition has started and if no orders are present
    if(competiton_state_ ==2 && order_queue.empty()){
        // Wait for new orders
        RCLCPP_INFO_STREAM(this->get_logger(),
        "Waiting for new order!!!");
    }
    // Check if the orders are stored
    if(order_count_>0){
        // Select the order
        SelectOrder();
    }
    if(competiton_state_ == 4){
        RCLCPP_INFO_STREAM(this->get_logger(),
        "Shutting down the node.");
        rclcpp::shutdown();
    }
    
}

////////////////// Subscriber Callbacks //////////////////////

//=================================================================================================
void CCS::UpdateAGV4Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  ProcessAGVStatus(4, location, vel);
}

//=================================================================================================
void CCS::UpdateAGV3Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  ProcessAGVStatus(3, location, vel);
}

//=================================================================================================
void CCS::UpdateAGV2Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  ProcessAGVStatus(2, location, vel);
}

//=================================================================================================
void CCS::UpdateAGV1Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  ProcessAGVStatus(1, location, vel);
}

//=================================================================================================
void CCS::StoreOrder(const ariac_msgs::msg::Order::SharedPtr msg) {
  // Get the order id from the message
  std::string id = msg->id;
  // Get the order type from the message
  int type = msg->type;

  // Create variables to store the order details
  ariac_msgs::msg::KittingTask task;
  int priority;
  int agv_number;
  int dst;
  // Check the order type to get appropriate details
  // This can be modified later to handle different order types
  if (type == 0) {
    task = msg->kitting_task;
  }
  // Get the agv number and destination from the task
  agv_number = task.agv_number;
  dst = task.destination;

  // Check if the order is a priority order
  if (msg->priority) {
    // Set the priority to 1
    priority = 1;
    // Increment the priority order count
    priority_order_count_++;

    RCLCPP_INFO_STREAM(
        this->get_logger(),
        MEMO << " Order Details:\n"
             << YELLOW << "\t\t\t\t\t\t\tOrder ID " << RESET << id << BLUE
             << "\n\t\t\t\t\t\t\tType: " << type << RESET << GREEN
             << "\n\t\t\t\t\t\t\tAGV Number: " << agv_number << RESET << RED
             << "\n\t\t\t\t\t\t\tPriority Order" << PRIORITY << RESET);
  } else {
    priority = 0;
    RCLCPP_INFO_STREAM(
        this->get_logger(),
        MEMO << " Order Details:\n"
             << YELLOW << "\t\t\t\t\t\t\tOrder ID " << RESET << id << BLUE
             << "\n\t\t\t\t\t\t\tType: " << type << RESET << GREEN
             << "\n\t\t\t\t\t\t\tAGV Number: " << agv_number << RESET);
  }

  // Create an order object and push it to the queue
  Order order;
  order.id = id;
  order.type = type;
  order.priority = priority;
  order.agv_number = agv_number;
  order.destination = dst;
  order_queue.push(order);

  // Store the order in the map with the agv number as the key
  agv_map[agv_number] = order;

  // Increment the order count
  order_count_++;
}

//=================================================================================================
void CCS::UpdateCompetitionState(
    const ariac_msgs::msg::CompetitionState::SharedPtr msg) {
  // Check the competition state and update the flag
  if (msg->competition_state == ariac_msgs::msg::CompetitionState::READY &&
      competiton_state_ != 1) {
    competiton_state_ = 1;
  }
  if (msg->competition_state == ariac_msgs::msg::CompetitionState::STARTED &&
      competiton_state_ != 2) {
    competiton_state_ = 2;
  }
  if (msg->competition_state ==
          ariac_msgs::msg::CompetitionState::ORDER_ANNOUNCEMENTS_DONE &&
      competiton_state_ != 3) {
    competiton_state_ = 3;
  }
  if (msg->competition_state == ariac_msgs::msg::CompetitionState::ENDED &&
      competiton_state_ != 4) {
    competiton_state_ = 4;
  }
}

//////////////////// Service Call Functions //////////////////

//=================================================================================================
bool CCS::StartCompetition() {
  // Create the service name
  std::string srv_name = "/ariac/start_competition";

  // Create the client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      this->create_client<std_srvs::srv::Trigger>(
          srv_name,
          rmw_qos_profile_services_default,  // quality of service
          service_callback_group_);

  // Create the request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Send the synchronous request
  auto result = client->async_send_request(request);

  try {
    // Get the result
    auto response = result.get();
    if (response->success)
      RCLCPP_INFO_STREAM(this->get_logger(),
                         GREEN << "Competition Started " << START << RESET);
  } catch (const std::exception &e) {
    // Handle the exception
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
  }

  // return the result
  return result.get()->success;
}

//=================================================================================================
bool CCS::EndCompetition() {
  // Create the service name
  std::string srv_name = "/ariac/end_competition";

  // Create the client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      this->create_client<std_srvs::srv::Trigger>(
          srv_name,
          rmw_qos_profile_services_default,  // quality of service
          service_callback_group_);

  // Create the request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Send the synchronous request
  auto result = client->async_send_request(request);

  try {
    // Get the result
    auto response = result.get();
    if (response->success)
      RCLCPP_INFO_STREAM(this->get_logger(),
                         RED << "Competition Ended " << END << RESET);
  } catch (const std::exception &e) {
    // Handle the exception
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
  }
  // return the result
  return result.get()->success;
}

//=================================================================================================
bool CCS::LockAGV(int number) {
  // Create the service name
  std::string srv_name = "ariac/agv" + std::to_string(number) + "_lock_tray";

  // Create the callback group
  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Set the callback group based on the AGV number
  switch (number) {
    case 1:
      callback_group_ = agv1_callback_group_;
      break;
    case 2:
      callback_group_ = agv2_callback_group_;
      break;
    case 3:
      callback_group_ = agv3_callback_group_;
      break;
    case 4:
      callback_group_ = agv4_callback_group_;
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Invalid AGV number.");
      break;
  }

  // Create the client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client =
      this->create_client<std_srvs::srv::Trigger>(
          srv_name,
          rmw_qos_profile_services_default,  // quality of service
          callback_group_);

  // Create the request
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // Send the synchronous request
  auto result = client->async_send_request(request);

  // Wait for the result
  try {
    // Get the result
    auto response = result.get();
    if (response->success)
      RCLCPP_INFO_STREAM(this->get_logger(), MAGENTA << AGV << " AGV " << number
                                                     << " Locked " << LOCK
                                                     << RESET);
  } catch (const std::exception &e) {  // Handle the exception
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
  }

  // Return the result
  return result.get()->success;
}

//=================================================================================================
void CCS::MoveAGV(int number, int destination) {
  // Create the service name
  std::string srv_name = "ariac/move_agv" + std::to_string(number);

  // Create the client
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr async_client_ =
      this->create_client<ariac_msgs::srv::MoveAGV>(srv_name);

  // Create the request
  std::shared_ptr<ariac_msgs::srv::MoveAGV_Request> request =
      std::make_shared<ariac_msgs::srv::MoveAGV::Request>();

  // Set the request parameters
  request->location = destination;

  // Send the asynchronous request
  auto result = async_client_->async_send_request(request);

  // Wait for the result. Using the callback function to handle the response
  auto future_result = async_client_->async_send_request(
      request,
      std::bind(&CCS::AGVResponseCallback, this, std::placeholders::_1));
}

//=================================================================================================
bool CCS::SubmitOrder(std::string order_id) {

  RCLCPP_INFO_STREAM(this->get_logger(), "Submitting Order " << order_id);
  // Create the service name
  std::string srv_name = "ariac/submit_order";

  // Create the client
  rclcpp::Client<ariac_msgs::srv::SubmitOrder>::SharedPtr client =
      this->create_client<ariac_msgs::srv::SubmitOrder>(
          srv_name,
          rmw_qos_profile_services_default,  // quality of service
          service_callback_group_);

  // Create the request
  auto request = std::make_shared<ariac_msgs::srv::SubmitOrder::Request>();

  // Set the request parameters
  request->order_id = order_id;

  // Send the synchronous request
  auto result = client->async_send_request(request);

  try {
    // Get the result
    auto response = result.get();
    if (response->success)
      RCLCPP_INFO_STREAM(this->get_logger(), BLUE << ORDER << " Order "
                                                  << order_id << " Submitted "
                                                  << CHECK << RESET);
  } catch (const std::exception &e) {
    // Handle the exception
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
  }

  // return the result
  return result.get()->success;
}

//////////////// Asynchronous Service Callback ///////////////////

//=================================================================================================
void CCS::AGVResponseCallback(
    rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedFuture future) {
  // Wait for the result.
  auto status = future.wait_for(std::chrono::seconds(1));

  // If the result is ready, get the value and print it out.
  if (status == std::future_status::ready) {
    auto result = future.get()->success;
    // TODO: Submit order can be added here
    if (result) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         GREEN << "AGV Reached Goal Location" << RESET);
    }
  }
}

///////////////////// Helper Functions //////////////////////////////

//=================================================================================================
void CCS::ProcessOrder(Order o){

    // Log the start of 15 seconds wait
    RCLCPP_INFO_STREAM(this->get_logger(),
        TIME<<" 15 seconds mandatory wait for order "<<o.id);

    // Wait for 15 seconds
    std::this_thread::sleep_for(std::chrono::seconds(15));

    // Log the end of 15 seconds wait
    RCLCPP_INFO_STREAM(this->get_logger(),
        CLOCK<<" 15 seconds wait completed for order "<<o.id);

    if (o.priority == 1) {
        // Lock the AGV corresponding to the order
        LockAGV(o.agv_number);
        // Move the AGV to the destination corresponding to the order
        MoveAGV(o.agv_number,o.destination);

    } 

    while (priority_order_count_ != 0) {
        
    }
    // Lock the AGV corresponding to the order
    LockAGV(o.agv_number);
    // Move the AGV to the destination corresponding to the order
    MoveAGV(o.agv_number,o.destination);

    
}

//=================================================================================================
void CCS::SelectOrder(){
    // Check if the order queue is not empty
    if(!order_queue.empty()){
        // Get the top order from the queue (Highest priority order)
        Order o = order_queue.top();
        // Remove the order from the queue
        order_queue.pop();

        // Create a bound function to process the each new order
        auto boundFunction = std::bind(&CCS::ProcessOrder, this, o);
        
        // Process the order on a new thread
        std::future<void> result = std::async(std::launch::async, boundFunction);

        // Store the future in the vector
        futures.push_back(std::move(result));
    
    }
    else{
        // Check if all the orders are announced and submitted
        if(competiton_state_==3 && (order_count_==submit_count_)){
            // Log the details
            RCLCPP_INFO_STREAM(this->get_logger(),BLUE<<"Orders Announced: " <<order_count_<<RESET);
            RCLCPP_INFO_STREAM(this->get_logger(),GREEN<<"Orders Submitted: " << submit_count_<<RESET);
            // End the competition
            EndCompetition(); 
        }
    }
}

//=================================================================================================
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto ccs_node = std::make_shared<CCS>("ccs_manager_node");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ccs_node);
  executor.spin();  // This will start the execution
  rclcpp::shutdown();
}
