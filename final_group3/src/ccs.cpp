/**
 * @file ccs.cpp
 * @author Shantanu Parab (sparab@umd.edu)
 * @brief This file generates a node, which calls upon a service to start the
 * competition, fetches orders published on a specific topic, treats them with
 * priority and submits accordingly with appropriate wait-time.
 * @version 0.1
 * @date 2024-03-19
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "final_group3/ccs.hpp"
#include <cmath>
#include <string>

//=================================================================================================
CCS::CCS(std::string node_name) : Node(node_name) {


  // Create a MutEx Callback group to run the main function (manager)
  order_process_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Create a MutEx Callback group for handling announced orders (storeOrders)
  order_storing_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Timer to run the main (manager) function
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds((int)(1000.0)), std::bind(&CCS::manager, this),
      order_process_group_);  //<< Using MutEx Callback

  // Subscriber for "ariac/competition_state"
  c_state_sub_ = this->create_subscription<ariac_msgs::msg::CompetitionState>(
      "ariac/competition_state", 10,
      std::bind(&CCS::updateCompetitionState, this, std::placeholders::_1));

  // Add callback group to Subscriber (ariac/orders)
  rclcpp::SubscriptionOptions options;
  options.callback_group = order_storing_group_;

  // Subscriber for "ariac/orders"
  order_subcriber_ = this->create_subscription<ariac_msgs::msg::Order>(
      "ariac/orders", 10,
      std::bind(&CCS::storeOrder, this, std::placeholders::_1),
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
      std::bind(&CCS::updateAGV1Status, this, std::placeholders::_1),
      agv_options);

  // Subscriber for "/ariac/agv2_status"
  agv2_subcriber_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv2_status", 10,
      std::bind(&CCS::updateAGV2Status, this, std::placeholders::_1),
      agv_options);

  // Subscriber for "/ariac/agv3_status"
  agv3_subcriber_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv3_status", 10,
      std::bind(&CCS::updateAGV3Status, this, std::placeholders::_1),
      agv_options);

  // Subscriber for "/ariac/agv4_status"
  agv4_subcriber_ = this->create_subscription<ariac_msgs::msg::AGVStatus>(
      "/ariac/agv4_status", 10,
      std::bind(&CCS::updateAGV4Status, this, std::placeholders::_1),
      agv_options);

  // Set QoS profile
  rclcpp::QoS qos_profile(10);
  qos_profile.history(rclcpp::HistoryPolicy::KeepLast);
  qos_profile.keep_last(10);
  qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  qos_profile.durability(rclcpp::DurabilityPolicy::SystemDefault);

  sensor_subscriber_ = this->create_subscription<ariac_msgs::msg::AdvancedLogicalCameraImage>(
      "/rwa_group3/detected_parts", qos_profile, std::bind(&CCS::updateSensorReading, this, std::placeholders::_1));

  // Robot Task Publisher
  robot_task_publisher_ = this->create_publisher<final_group3::msg::RobotTask>(
      "robot_task", 10);

  robot_status_publisher_ = this->create_publisher<final_group3::msg::RobotStatus>(
      "robot_status", 10);

  robot_status_subscriber_ = this->create_subscription<final_group3::msg::RobotStatus>(
      "robot_status", 10,
      std::bind(&CCS::updateRobotStatus, this, std::placeholders::_1),
      options);


  agv1_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  agv2_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  agv3_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  agv4_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
}



//=================================================================================================
void CCS::updateRobotStatus(const final_group3::msg::RobotStatus::SharedPtr msg) {
  // RCLCPP_INFO_STREAM(this->get_logger(), "Robot Status Updated");
  floor_robot_ = msg->floor_robot;
  ceiling_robot_ = msg->ceiling_robot;
  if(floor_robot_ == final_group3::msg::RobotStatus::FREE){
    // RCLCPP_INFO_STREAM(this->get_logger(), PURPLE<< "Floor Robot Status: FREE" << RESET);
    // task_in_waiting_--;
    
  } else if(floor_robot_ == final_group3::msg::RobotStatus::RETRY){
    // RCLCPP_INFO_STREAM(this->get_logger(), PURPLE<< "Floor Robot Status: RETRY" << RESET);
    final_group3::msg::RobotTask task = msg->robot_task;
    retry_count_++;

    // Create a bound function to process the each new order
    auto boundFunction = std::bind(&CCS::retry, this, task);
    
    // Process the order on a new thread
    std::future<void> result = std::async(std::launch::async, boundFunction);

    // Store the future in the vector
    futures.push_back(std::move(result));
  
  }
  else{
    // RCLCPP_INFO_STREAM(this->get_logger(), PURPLE<<"Floor Robot Status: OCCUPIED" << RESET);
  }
}


void CCS::retry(final_group3::msg::RobotTask task){

  final_group3::msg::RobotStatus robot_status;
  robot_status.floor_robot = final_group3::msg::RobotStatus::OCCUPIED;
  robot_status_publisher_->publish(robot_status);
  floor_robot_ = final_group3::msg::RobotStatus::OCCUPIED;
  bool part_found = false;
  while(!part_found){
              auto part_to_remove = available_parts.begin();
              for (auto part = available_parts.begin(); part != available_parts.end(); ++part) {
                  if (part->part.type == task.part.type && part->part.color == task.part.color) {
                      RCLCPP_INFO_STREAM(this->get_logger(), "Found " << color[part->part.color] << " " << type[part->part.type]);
                      ariac_msgs::msg::Part part_ = part->part;
                      geometry_msgs::msg::Pose pose = part->pose;
                      task.pose = part->pose;
                      tf2::Quaternion quat_tf;
                      tf2::fromMsg(pose.orientation, quat_tf);
                      auto rpy = utils_ptr_->set_euler_from_quaternion(quat_tf);

                      RCLCPP_INFO_STREAM(this->get_logger(), text[part_.color]<<">> " << color[part_.color] <<" "<<type[part_.type] <<RESET
                          << "\n\t\t\t\t\t\t\t>> Position (xyz) " << pose.position.x << " " << pose.position.y << " " << pose.position.z
                          << "\n\t\t\t\t\t\t\t>> Orientation (rpy) " << std::to_string(rpy[0]) << " " << std::to_string(rpy[1]) << " " << std::to_string(rpy[2])
                          << "\n\t\t\t\t\t\t\t>> Quadrant " << std::to_string(task.quadrant));
                      part_to_remove = part;
                      part_found = true;
                      break;
                  }
              }

              if (part_found) {
                  available_parts.erase(part_to_remove);
              }
              else {
                  RCLCPP_INFO_STREAM(this->get_logger(), RED << "Part Not Found" << RESET);
                  RCLCPP_INFO_STREAM(this->get_logger(), RED << "Setting Robot Free" << RESET);
                  final_group3::msg::RobotStatus robot_status;
                  robot_status.floor_robot = final_group3::msg::RobotStatus::FREE;
                  robot_status_publisher_->publish(robot_status);
                  floor_robot_ = final_group3::msg::RobotStatus::FREE;
                  break;
              }
  }
  robot_task_publisher_->publish(task);

  RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Sent a Retry Task to Robto"<<RESET);
  std::this_thread::sleep_for(std::chrono::seconds(10));  
  while (floor_robot_ == final_group3::msg::RobotStatus::OCCUPIED || floor_robot_ == final_group3::msg::RobotStatus::RETRY){
        // RCLCPP_INFO_STREAM(this->get_logger(),RED<< "Waiting for the robot to be free"<<RESET);
         }
  retry_count_ --;
}


//=================================================================================================
void CCS::processAGVStatus(int agv, int location, float vel) {

  if (location == ariac_msgs::msg::AGVStatus::WAREHOUSE && !agv_flag_[agv-1]) {
    if (std::abs(vel) < 0.00025) {
        submitOrder(agv_map[agv].id);
        agv_flag_[agv-1] = true;
        submit_count_++;
        if (agv_map[agv].priority == 1){
            priority_order_count_--;
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Order: " << agv_map[agv].id
                                                         << " on AGV " << agv
                                                         << " Submitted");
        RCLCPP_INFO_STREAM(this->get_logger(),
                           "Total Orders Submitted " << submit_count_);
    } else {

    }
  }

}

//=================================================================================================
void CCS::manager(){


  

    // Check if the competition is ready
    if(competiton_state_==1){
        RCLCPP_INFO_STREAM(this->get_logger(),
        ORANGE<<"Start the competition"<<RESET);
        // Start the competition
        startCompetition();

    }
    // Check if the competition has started and if no orders are present
    if(competiton_state_ ==2 && order_queue.empty()){
        // Wait for new orders
        // RCLCPP_INFO_STREAM(this->get_logger(),
        // "Waiting for new order!!!");
    }
    // Check if the orders are stored
    if(order_count_>0){

        if(!warm_up_){
          // Log the start of 15 seconds wait
        // RCLCPP_INFO_STREAM(this->get_logger(),
        //     TIME<<" Warming up sensors!!! ");

        // Wait for 15 seconds
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Log the end of 15 seconds wait
        // RCLCPP_INFO_STREAM(this->get_logger(),
        //     CLOCK<<" Sensors Ready ");
        warm_up_ = true;
        }
        // Select the order
        selectOrder();
    }
    if(competiton_state_ == 4){
        RCLCPP_INFO_STREAM(this->get_logger(),
        "Shutting down the node.");
        rclcpp::shutdown();
    }
    
}

////////////////// Subscriber Callbacks //////////////////////


void CCS::updateSensorReading(const ariac_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    // Store the part in the map
    for(auto part: msg->part_poses){
        // RCLCPP_INFO_STREAM(this->get_logger(), "Part Detected: " << std::to_string(part.part.type) << " " << std::to_string(part.part.color));
        available_parts.push_back(part);
    }
    for(auto tray: msg->tray_poses){
        available_trays.push_back(tray);
    }
}

//=================================================================================================
void CCS::updateAGV4Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  processAGVStatus(4, location, vel);
}

//=================================================================================================
void CCS::updateAGV3Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  processAGVStatus(3, location, vel);
}

//=================================================================================================
void CCS::updateAGV2Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  processAGVStatus(2, location, vel);
}

//=================================================================================================
void CCS::updateAGV1Status(const ariac_msgs::msg::AGVStatus::SharedPtr msg) {
  float vel = msg->velocity;
  int location = msg->location;
  processAGVStatus(1, location, vel);
}

//=================================================================================================
void CCS::storeOrder(const ariac_msgs::msg::Order::SharedPtr msg) {


  // Create an order object and push it to the queue
  Order order;
  
  // Assign Common Order Details
  order.id = msg->id;
  RCLCPP_INFO_STREAM(this->get_logger(),RED<<"==========================================================="<<RESET);
  RCLCPP_INFO_STREAM(
        this->get_logger(),
        MEMO << " Order Received: " << ORANGE << "Order ID " << RESET << order.id);
  RCLCPP_INFO_STREAM(this->get_logger(),RED<<"==========================================================="<<RESET);

  order.type = msg->type;

  if (msg->type == ariac_msgs::msg::Order::KITTING) {

      order.kitting_task = msg->kitting_task;
      order.agv_number = order.kitting_task.agv_number;
      order.destination = order.kitting_task.destination;

      // RCLCPP_INFO_STREAM(this->get_logger(), BLUE << " KITTING TASK " << RESET);
  }
  else if (msg->type == ariac_msgs::msg::Order::ASSEMBLY) {

      order.assembly_task = msg->assembly_task;

      // order.parts_and_poses= GetPartPose(order.id);

      order.station = order.assembly_task.station;

      RCLCPP_INFO_STREAM(this->get_logger(), BLUE << " ASSEMBLY TASK " << RESET);
  }
  else if (msg->type == ariac_msgs::msg::Order::COMBINED) {

      order.combined_task = msg->combined_task;

      RCLCPP_INFO_STREAM(this->get_logger(), BLUE << " COMBINED TASK " << RESET);
  }

  

  // Check if the order is a priority order
  if (msg->priority) {
    // Set the priority to 1
    order.priority = 1;
    RCLCPP_INFO_STREAM(this->get_logger(), RED << "Priority Order" << PRIORITY << RESET);
    // Increment the priority order count
    priority_order_count_++;
    priority_task_count_++;
  }
  else {
    // Set the priority to 0
    order.priority = 0;
  }


  order_queue.push(order);
  


  
  // Store the order in the map with the agv number as the key
  agv_map[order.agv_number] = order;

  // Increment the order count
  order_count_++;
}

//=================================================================================================
void CCS::updateCompetitionState(
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
bool CCS::startCompetition() {
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

  result.wait();

  // return the result
  return result.get()->success;
}

//=================================================================================================
bool CCS::endCompetition() {
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

  result.wait();

  RCLCPP_INFO_STREAM(this->get_logger(),RED<< "Competition Ended"<<RESET);
  // return the result
  return result.get()->success;
}

//=================================================================================================
bool CCS::lockAGV(int number) {
  if (number == 0) {
    return true;
  }
  // Create the service name
  std::string srv_name = "ariac/agv" + std::to_string(number) + "_lock_tray";
  // RCLCPP_INFO_STREAM(this->get_logger(),ORANGE<<"Locking AGV "<<std::to_string(number)<<RESET);
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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid AGV number. " << std::to_string(number));
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

  result.wait();
  // Return the result
  return result.get()->success;
}

//=================================================================================================
void CCS::moveAGV(int number, int destination) {
  if (number == 0) {
    return;
  }
  // Create the service name
  std::string srv_name = "ariac/move_agv" + std::to_string(number);

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
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid AGV number. " << std::to_string(number));
      break;
  }



  // Create the client
  rclcpp::Client<ariac_msgs::srv::MoveAGV>::SharedPtr async_client_ =
      this->create_client<ariac_msgs::srv::MoveAGV>(srv_name, 
      rmw_qos_profile_services_default,
      callback_group_);

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
bool CCS::submitOrder(std::string order_id) {

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

  result.wait();

  // return the result
  return result.get()->success;
}


//=================================================================================================
OrderStatus CCS::performQualityCheck(std::string order_id) {

  RCLCPP_INFO_STREAM(this->get_logger(), PURPLE << "Quality Check Order " << order_id <<RESET);
  // Create the service name
  std::string srv_name = "/ariac/perform_quality_check";


  // Create the client
  rclcpp::Client<ariac_msgs::srv::PerformQualityCheck>::SharedPtr client =
      this->create_client<ariac_msgs::srv::PerformQualityCheck>(
          srv_name,
          rmw_qos_profile_services_default,  // quality of service
          service_callback_group_);

  // Create the request
  auto request = std::make_shared<ariac_msgs::srv::PerformQualityCheck::Request>();

  // Set the request parameters
  request->order_id = order_id;

  // Send the synchronous request
  auto result = client->async_send_request(request);

  result.wait();

  OrderStatus status;

  status.valid_id = result.get()->valid_id;
  status.all_passed = result.get()->all_passed;
  status.incorrect_tray = result.get()->incorrect_tray;
  status.quadrant1 = result.get()->quadrant1;
  status.quadrant2 = result.get()->quadrant2;
  status.quadrant3 = result.get()->quadrant3;
  status.quadrant4 = result.get()->quadrant4;

  return status;
  
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
std::vector<final_group3::msg::RobotTask> CCS::createRobotTask(Order o){
  // Create a vector to store the tasks
  std::vector<final_group3::msg::RobotTask> tasks;

  if(o.type == ariac_msgs::msg::Order::KITTING){

      // Create a task object
      final_group3::msg::RobotTask tray_task;
      // Get AGV Number
      tray_task.agv_number = o.agv_number;
      // Get Tray ID
      tray_task.tray_id = o.kitting_task.tray_id;
      // Set the task type
      tray_task.task_type = final_group3::msg::RobotTask::TRAY;
      // Push the task to the vector
      tasks.push_back(tray_task);

    for (auto part : o.kitting_task.parts) {

        // Create a task object
        final_group3::msg::RobotTask part_task;

        // Set the task type
        part_task.task_type = final_group3::msg::RobotTask::PART;

        // Get order id
        part_task.order_id = o.id;

        // Get AGV Number
        part_task.agv_number = o.agv_number;
        
        // Set the part quadruant
        part_task.quadrant = part.quadrant;

        // Set the part
        part_task.part = part.part;
      
        // Push the task to the vector
        tasks.push_back(part_task);
      }
  }

   return tasks;

}


void CCS::executeTasks(std::vector<final_group3::msg::RobotTask> tasks, Order o){
    
    RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Order Execution "<< o.id <<RESET);
    final_group3::msg::RobotTask prev_task;

    for (auto task : tasks) {
          start:
          RCLCPP_INFO_STREAM(this->get_logger(), CYAN<<"======================== " << o.id << " ========================"<<RESET);
          
          if(o.priority==0){
            if(priority_task_count_ > 0){
              RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Priority Task in Queue "<<priority_task_count_<<RESET);
              while(priority_task_count_ > 0){
                
              }
            }
          }
          
          if (floor_robot_ == final_group3::msg::RobotStatus::RETRY){
              RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Retry Count "<<retry_count_<<RESET);
              RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Robot is in Retry Mode"<<RESET);
              while (floor_robot_ == final_group3::msg::RobotStatus::RETRY){

              }
          }
          
          RCLCPP_INFO_STREAM(this->get_logger(),ORANGE<<"Robot Status While waiting Task Execution "<<floor_robot_<<RESET);
          
          if(floor_robot_ == final_group3::msg::RobotStatus::OCCUPIED){
            RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Robot is Occupied"<<RESET);
            while (floor_robot_ == final_group3::msg::RobotStatus::OCCUPIED){

            }
            if(o.priority==0){
            if(priority_task_count_ > 0){
              RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Priority Task was added to Queue while waiting "<<priority_task_count_<<RESET);
              RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Restarting Task"<<RESET);
              goto start;
            }
            if (floor_robot_ == final_group3::msg::RobotStatus::RETRY){
              RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"A retry task was added to the queue while waiting  "<<retry_count_<<RESET);
              RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Restarting Task"<<RESET);
              goto start;
          }
          }
            task_in_waiting_++;
            RCLCPP_INFO_STREAM(this->get_logger(), CYAN<<"======================== " << o.id << " ========================"<<RESET);
            RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Prority Tasks in Queue "<<priority_task_count_<<RESET);
            RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Retry Count "<<retry_count_<<RESET);
            RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"Task in waiting "<< task_in_waiting_ <<RESET);
          }
          
         
          final_group3::msg::RobotStatus robot_status;
          robot_status.floor_robot = final_group3::msg::RobotStatus::OCCUPIED;
          robot_status_publisher_->publish(robot_status);
          floor_robot_ = final_group3::msg::RobotStatus::OCCUPIED;
          // RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Robot Status: OCCUPIED"<<RESET);
          
          
         bool insufficent_parts = false;
         if(task.task_type == final_group3::msg::RobotTask::PART){
            // continue;
            bool part_found = false;

          while(!part_found){
              auto part_to_remove = available_parts.begin();
              for (auto part = available_parts.begin(); part != available_parts.end(); ++part) {
                  if (part->part.type == task.part.type && part->part.color == task.part.color) {
                      // RCLCPP_INFO_STREAM(this->get_logger(), "Found " << color[part->part.color] << " " << type[part->part.type]);
                      ariac_msgs::msg::Part part_ = part->part;
                      geometry_msgs::msg::Pose pose = part->pose;
                      task.pose = part->pose;
                      tf2::Quaternion quat_tf;
                      tf2::fromMsg(pose.orientation, quat_tf);
                      auto rpy = utils_ptr_->set_euler_from_quaternion(quat_tf);

                      RCLCPP_INFO_STREAM(this->get_logger(), text[part_.color]<<">> " << color[part_.color] <<" "<<type[part_.type] <<RESET
                          << "\n\t\t\t\t\t\t\t>> Position (xyz) " << pose.position.x << " " << pose.position.y << " " << pose.position.z
                          << "\n\t\t\t\t\t\t\t>> Orientation (rpy) " << std::to_string(rpy[0]) << " " << std::to_string(rpy[1]) << " " << std::to_string(rpy[2])
                          << "\n\t\t\t\t\t\t\t>> Quadrant " << std::to_string(task.quadrant));
                      part_to_remove = part;
                      part_found = true;
                      break;
                  }
              }

              if (part_found) {
                  available_parts.erase(part_to_remove);
              }
              else {
                  RCLCPP_INFO_STREAM(this->get_logger(), RED << "Part Not Found" << RESET);
                  RCLCPP_INFO_STREAM(this->get_logger(), RED << "Setting Robot Free" << RESET);
                  final_group3::msg::RobotStatus robot_status;
                  robot_status.floor_robot = final_group3::msg::RobotStatus::FREE;
                  robot_status_publisher_->publish(robot_status);
                  floor_robot_ = final_group3::msg::RobotStatus::FREE;
                  insufficent_parts = true;
                  break;
              }
              
          }
          
         }
         else if(task.task_type == final_group3::msg::RobotTask::TRAY){
            
            bool tray_found = false;
            while(!tray_found){
              auto tray_to_remove = available_trays.begin();
              for (auto tray = available_trays.begin(); tray != available_trays.end(); ++tray) {
                  if (tray->id == task.tray_id) {
                      // RCLCPP_INFO_STREAM(this->get_logger(), "Found Tray " << std::to_string(tray->id));
                      int tray_id_ = tray->id;
                      geometry_msgs::msg::Pose pose = tray->pose;
                      task.pose = pose;
                      tf2::Quaternion quat_tf;
                      tf2::fromMsg(pose.orientation, quat_tf);
                      auto rpy = utils_ptr_->set_euler_from_quaternion(quat_tf);

                      RCLCPP_INFO_STREAM(this->get_logger(), ">> " << "Kitting Tray"
                          << "\n\t\t\t\t\t\t\t>> Tray ID " << std::to_string(tray_id_)
                          << "\n\t\t\t\t\t\t\t>> Position (xyz) " << pose.position.x << " " << pose.position.y << " " << pose.position.z
                          << "\n\t\t\t\t\t\t\t>> Orientation (rpy) " << std::to_string(rpy[0]) << " " << std::to_string(rpy[1]) << " " << std::to_string(rpy[2]));
                      tray_to_remove = tray;
                      tray_found = true;
                      break;
                  }
              }
              if (tray_found) {
                  available_trays.erase(tray_to_remove);
              }
              else {
                  RCLCPP_INFO_STREAM(this->get_logger(), RED << "Part Not Found" << RESET);
                  RCLCPP_INFO_STREAM(this->get_logger(), RED << "Setting Robot Free" << RESET);
                  final_group3::msg::RobotStatus robot_status;
                  robot_status.floor_robot = final_group3::msg::RobotStatus::FREE;
                  robot_status_publisher_->publish(robot_status);
                  floor_robot_ = final_group3::msg::RobotStatus::FREE;
              }
         }
    }
    if(!insufficent_parts){
    robot_task_publisher_->publish(task);
    }
    RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"==========================================================="<<RESET);
    }
    std::this_thread::sleep_for(std::chrono::seconds(10));
    while (floor_robot_ == final_group3::msg::RobotStatus::OCCUPIED || floor_robot_ == final_group3::msg::RobotStatus::RETRY){
        // RCLCPP_INFO_STREAM(this->get_logger(),RED<< "Waiting for the robot to be free"<<RESET);
         }
 
}

//=================================================================================================
void CCS::processOrder(Order o){

    
    // Wait for 15 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));

    if(o.priority == 0){
        while(order_in_waiting_ > 0){
        // RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Waiting for the robot to be free"<<RESET);
        }
    }
    order_in_waiting_++;
   
    // RCLCPP_INFO_STREAM(this->get_logger(), "Processing Priority Order " << o.id);
    // Create a vector to store the tasks
    std::vector<final_group3::msg::RobotTask> tasks;
    tasks = createRobotTask(o);
    // Execute the tasks
    executeTasks(tasks,o);

    while(retry_count_ > 0){
        // RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Waiting for the robot to be free"<<RESET);
    }
    order_in_waiting_--;
    OrderStatus status = performQualityCheck(o.id);

    if(status.all_passed){
      RCLCPP_INFO_STREAM(this->get_logger(),GREEN<<"Order "<<o.id<<" Ready to submit"<<RESET);
    } else{
      RCLCPP_INFO_STREAM(this->get_logger(),RED<<"Order "<<o.id<<" Failed Quality Check Cannot be Sumbitted"<<RESET);
    }

    RCLCPP_INFO_STREAM(this->get_logger(),CYAN<<"==========================================================="<<RESET);

    if(o.priority == 1){
        priority_task_count_--;
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    // Lock the AGV corresponding to the order
    lockAGV(o.agv_number);
    // Move the AGV to the destination corresponding to the order
    moveAGV(o.agv_number,o.destination);


    
}

//=================================================================================================
void CCS::selectOrder(){
    // Check if the order queue is not empty
    if(!order_queue.empty()){
        // Get the top order from the queue (Highest priority order)
        Order o = order_queue.top();
        // Remove the order from the queue
        order_queue.pop();

        // Create a bound function to process the each new order
        auto boundFunction = std::bind(&CCS::processOrder, this, o);
        
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
            endCompetition(); 
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
