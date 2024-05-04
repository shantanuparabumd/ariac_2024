#include "rwa5_group3/robot_manager.hpp"
#include <string>
#include <unistd.h>

RobotManager::RobotManager() : Node("robot_manager"),
     floor_node_(std::make_shared<rclcpp::Node>("floor_group_node")),
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
     planning_scene_()
{    

    // Create a MutEx Callback group for handling announced orders (StoreOrders)
    gripper_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Set the parameter use_sim_time to true
    rclcpp::ParameterValue use_sim_time_param(true);
    this->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time_param));

    floor_node_->set_parameter(rclcpp::Parameter("use_sim_time", use_sim_time_param));

    auto mgi_options_floor = moveit::planning_interface::MoveGroupInterface::Options(
      "floor_robot", "robot_description");
    // Create a move_group_interface::MoveGroupInterface object for the floor robot
    floor_robot_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(floor_node_,mgi_options_floor);
    
    if(floor_robot_ ->startStateMonitor()){
        RCLCPP_INFO(floor_node_->get_logger(), "Floor robot state monitor started");
    } else {
        RCLCPP_ERROR(floor_node_->get_logger(), "Floor robot state monitor failed to start");
    }

    // use upper joint velocity and acceleration limits
    floor_robot_->setMaxAccelerationScalingFactor(1.0);
    floor_robot_->setMaxVelocityScalingFactor(1.0);
    floor_robot_->setNumPlanningAttempts(10);
    floor_robot_->setPlanningTime(5.0);


    // Add the planning node to the executor
    executor_->add_node(floor_node_);
    executor_thread_ = std::thread([this]() { this->executor_->spin(); });


    // Publihser to publish robot status
    robot_status_publisher_ = this->create_publisher<rwa5_group3::msg::RobotStatus>(
      "robot_status", 10);

    // Subscriber to get robot task
    robot_task_subscription_ = this->create_subscription<rwa5_group3::msg::RobotTask>(
        "robot_task", 10, std::bind(&RobotManager::robotTaskCallback, this, std::placeholders::_1));

    rclcpp::SubscriptionOptions options;
      options.callback_group = gripper_group_;
      
    // Subscriber to get floor gripper state
    floor_gripper_state_sub_ = this->create_subscription<ariac_msgs::msg::VacuumGripperState>(
        "/ariac/floor_robot_gripper_state", 10, std::bind(&RobotManager::floorGripperStateCB, this, std::placeholders::_1), options);

    // Callback group for service calls
    service_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RobotManager::addModelsToPlanningScene();
}

//=====================================================================================================================
RobotManager::~RobotManager()
{
    floor_robot_->~MoveGroupInterface();
}



//=====================================================================================================================
void RobotManager::robotTaskCallback(const rwa5_group3::msg::RobotTask::SharedPtr msg)
{   
  current_task_ = *msg;

  RCLCPP_INFO_STREAM(this->get_logger(), PURPLE<<"Received Task "<< current_task_.task_type<<RESET);

  retry:
  // Check if the task is to pick up a tray
  if(msg->task_type == rwa5_group3::msg::RobotTask::TRAY){
    

    RCLCPP_INFO(this->get_logger(), "Picking Up Tray");
    
    // Call the function to pick up the tray
    floorRobotPickTray(msg->pose,msg->agv_number,msg->tray_id);

    OrderStatus status = performQualityCheck(msg->order_id);
    if(status.incorrect_tray){
      RCLCPP_INFO_STREAM(this->get_logger(), RED<<"Retry Picking Tray"<<RESET);
      goto retry;
    }
    else{
      RCLCPP_INFO(this->get_logger(), "Tray Placed Successfully");
      rwa5_group3::msg::RobotStatus robot_status_msg;
      robot_status_msg.floor_robot = rwa5_group3::msg::RobotStatus::FREE;
      robot_status_publisher_->publish(robot_status_msg);
    }
  }

  // Check if the task is to pick up a part
  else if(msg->task_type == rwa5_group3::msg::RobotTask::PART){
    RCLCPP_INFO(this->get_logger(), "Picking Up Part");
    bool success = floorRobotPickPart(msg->pose);
    if (!success){
      RCLCPP_ERROR_STREAM(get_logger(), RED<<"Get a New Part"<<RESET);
      rwa5_group3::msg::RobotStatus robot_status_msg;
      robot_status_msg.floor_robot = rwa5_group3::msg::RobotStatus::RETRY;
      robot_status_msg.robot_task = current_task_;
      robot_status_publisher_->publish(robot_status_msg);
    }
    else{
      // Set the robot status to free    
      rwa5_group3::msg::RobotStatus robot_status_msg;
      robot_status_msg.floor_robot = rwa5_group3::msg::RobotStatus::FREE;
      robot_status_publisher_->publish(robot_status_msg);
    }
  }

}



bool RobotManager::checkFaultyPart(int quadrant)
{
    OrderStatus status = performQualityCheck(current_task_.order_id);
    switch (quadrant)
    { case 1:
        return status.quadrant1.faulty_part;
        break;
    case 2:
        return status.quadrant2.faulty_part;
        break;
    case 3:
        return status.quadrant3.faulty_part;
        break;
    case 4:    
        return status.quadrant4.faulty_part;
        break;
    default:
        return false;
        break;
    }

  return false;
}


bool RobotManager::checkMissingPart(int quadrant)
{
    OrderStatus status = performQualityCheck(current_task_.order_id);
    switch (quadrant)
    { case 1:
        return status.quadrant1.missing_part;
        break;
    case 2:
        return status.quadrant2.missing_part;
        break;
    case 3:
        return status.quadrant3.missing_part;
        break;
    case 4:    
        return status.quadrant4.missing_part;
        break;
    default:
        return false;
        break;
    }

  return false;
}

//=====================================================================================================================
void RobotManager::floorRobotPickTray(geometry_msgs::msg::Pose pose,int agv_number, int tray_id)
{    


    std::vector<double> current_joint_values = floor_robot_->getCurrentJointValues();
    double rail = current_joint_values[0];
    RCLCPP_INFO_STREAM(get_logger(), PURPLE<<"Current Rail Position: " << rail<<RESET << " Part Position: "<< pose.position.y);
    if((rail < 0.0 && pose.position.y < 0.0)  || (rail > 0.0 && pose.position.y > 0.0) ){
      floor_robot_->setJointValueTarget("linear_actuator_joint",0.0);
      moveFloorToTarget();
    }

    // Move to the closest tray station
    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget(floor_kts2_js_);
      moveFloorToTarget();
    } else{
      floor_robot_->setJointValueTarget(floor_kts1_js_);
      moveFloorToTarget();
    }
    // return;
    // Check if the gripper is already in the correct state
    if(current_grip == 0){
      // Move to the closest tray station
      if(pose.position.y > 0.0){
        changeGripper("kts2","trays");
      }  else{
        changeGripper("kts1","trays");
      }
    }

    
    



    // Enable the gripper to pick up the tray
    setGripperState(true);

    // Pick Tray
    RCLCPP_INFO_STREAM(get_logger(), "Picking up tray at "<< pose.position.x << " " << pose.position.y << " " << pose.position.z);
    pick(pose,"tray");

    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray_id) + "_" + std::to_string(std::rand());
    addSingleModelToPlanningScene(tray_name, "kit_tray.stl", pose);
    floor_robot_->attachObject(tray_name);


    // Lift the tray up slightly
    double tray_rotation =utils.get_yaw_from_pose_(pose);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    // Move up slightly
    waypoints.clear();
    waypoints.push_back(utils.build_pose(
        pose.position.x, pose.position.y, pose.position.z + 0.2,
        setRobotOrientation(tray_rotation)));

    double offset = 0.001;
    while (!moveThroughWaypoints(waypoints, 0.2, 0.1)) {
      RCLCPP_ERROR(get_logger(), "Unable to move up");
      waypoints.clear();
      waypoints.push_back(utils.build_pose(
        pose.position.x, pose.position.y, pose.position.z + 0.2 + offset,
        setRobotOrientation(tray_rotation)));
    }


    // Move the tray to the AGV based on order
    moveTrayToAGV(agv_number);

    // Place Tray on AGV and detach
    setGripperState(false);

    floor_robot_->detachObject(tray_name);

    RCLCPP_INFO(get_logger(), "Tray placed on AGV");
    

}



//=====================================================================================================================
bool RobotManager::floorRobotPickPart(geometry_msgs::msg::Pose pose)
{    

    std::vector<double> current_joint_values = floor_robot_->getCurrentJointValues();
    double rail = current_joint_values[0];
    RCLCPP_INFO_STREAM(get_logger(), PURPLE<<"Current Rail Position: " << rail<<RESET << " Part Position: "<< pose.position.y);
    if((rail < 0.0 && pose.position.y < 0.0)  || (rail > 0.0 && pose.position.y > 0.0) ){
      floor_robot_->setJointValueTarget("linear_actuator_joint",0.0);
      moveFloorToTarget();
    }

    int flag = 0;
    // Check if the gripper is already in the correct state
    if(current_grip == 1){
        
    // Move to the closest tray station
    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget(floor_kts2_js_);
      moveFloorToTarget();
      // Change Gripper
      changeGripper("kts2","parts");
    }
    else{
      floor_robot_->setJointValueTarget(floor_kts1_js_);
      moveFloorToTarget();
      // Change Gripper
      changeGripper("kts1","parts");
    }
    }

    
    


    
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint",0.0);
    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["right_bins"]);
      moveFloorToTarget();
    }
    else{
      floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["left_bins"]);
      moveFloorToTarget();
    }

    // Enable the gripper to pick up the tray
    setGripperState(true);

    // // Pick Part
    RCLCPP_INFO_STREAM(get_logger(), "Picking up part at "<< pose.position.x << " " << pose.position.y << " " << pose.position.z);
    pick(pose,"part");
    
    // // Add kit tray to planning scene
    std::string part_name = part_names_[current_task_.part.type] + "_" + std::to_string(std::rand());
    std::string mesh = part_names_[current_task_.part.type]+".stl";
    addSingleModelToPlanningScene(part_name, mesh, pose);
    floor_robot_->attachObject(part_name);

    pick_and_place = true;



    // Lift the tray up slightly
    double tray_rotation =utils.get_yaw_from_pose_(pose);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    // Move up slightly
    waypoints.clear();
    waypoints.push_back(utils.build_pose(
        pose.position.x, pose.position.y, pose.position.z + 0.2,
        setRobotOrientation(tray_rotation)));
    double offset = 0.001;
    while (!moveThroughWaypoints(waypoints, 0.2, 0.2)) {
      if(retry){
        retry = false;
        return false;
      }
      RCLCPP_ERROR(get_logger(), "Unable to move up");
      waypoints.clear();
      waypoints.push_back(utils.build_pose(
          pose.position.x, pose.position.y, pose.position.z + 0.2 +offset,
          setRobotOrientation(tray_rotation)));
    }

    if(retry){
      retry = false;
      return false;
    }

    

    // // Move the tray to the AGV based on order
    movePartToQuadrant();

    if(retry){
      retry = false;  
      return false;
    }

    if(checkFaultyPart(current_task_.quadrant)){

      RCLCPP_INFO_STREAM(this->get_logger(), RED<<"Faulty Part in Quadrant: " << current_task_.quadrant<<RESET);
        // // Move the tray to the AGV based on order
      movePartToDiscard();

      flag =1;
    }

    

    
    setGripperState(false);
    
    floor_robot_->detachObject(part_name);

    pick_and_place = false;

    if(flag ==1){
      return false;
    }
    RCLCPP_INFO_STREAM(get_logger(), RED<<"Part placed on AGV"<<RESET);
    rwa5_group3::msg::RobotStatus robot_status_msg;
    robot_status_msg.floor_robot = rwa5_group3::msg::RobotStatus::FREE;
    robot_status_publisher_->publish(robot_status_msg);
    return true;
    

}



//=================================================================================================
OrderStatus RobotManager::performQualityCheck(std::string order_id) {

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


bool RobotManager::pick(geometry_msgs::msg::Pose pose,std::string type)
{   
    double yaw = 0.0;
    if (pose.position.y > 0.0)
        yaw = 0.0;
    else
        yaw = 3.142;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(utils.build_pose(pose.position.x, pose.position.y,
                                     pose.position.z + 0.4, setRobotOrientation(yaw)));

    double offset = 0.0;
    if(type=="part"){
        offset = part_heights_[current_task_.part.type];
        RCLCPP_INFO(get_logger(), "Part height: %f", offset);
    }
    else if(type=="discard"){
        offset = part_heights_[current_task_.part.type] + 0.006;
        RCLCPP_INFO(get_logger(), "Part height: %f", offset);
    }
    else if(type=="tray"){
        offset = 0.0;
    }
    waypoints.push_back(utils.build_pose(pose.position.x, pose.position.y,
                                          pose.position.z + offset , setRobotOrientation(yaw)));

    if (!moveThroughWaypoints(waypoints, 0.2, 0.1))
        return false;
    
    // wait_for_attach_completion_(10.0);

    return true;

}


bool RobotManager::moveTrayToAGV(int agv_number)
{   


    RCLCPP_INFO_STREAM(get_logger(), ORANGE<<"Moving tray to AGV: " << std::to_string(agv_number)<<RESET);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_number)]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

    if (!moveFloorToTarget())
    {
        RCLCPP_ERROR_STREAM(get_logger(), RED<<"Unable to move tray to AGV"<<RESET);
        return false;
    }
    auto agv_tray_pose = getPoseInWorldFrame("agv" + std::to_string(agv_number) + "_tray");
    
    auto agv_rotation = 1.570807;

    RCLCPP_INFO_STREAM(get_logger()," Moving to top of AGV: " << std::to_string(agv_number));

    waypoints.clear();
    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + 0.3, setRobotOrientation(agv_rotation)));

    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, setRobotOrientation(agv_rotation)));

    double offset=0.001;

    while (!moveThroughWaypoints(waypoints, 0.2, 0.1))
    {
        RCLCPP_ERROR(get_logger(), "Unable to move Tray");

        waypoints.clear();
        waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                              agv_tray_pose.position.z + 0.3 + offset,  setRobotOrientation(agv_rotation)));

        waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                              agv_tray_pose.position.z + part_heights_[current_task_.part.type],  setRobotOrientation(agv_rotation)));
        
        offset += 0.001;
        // return false;
    }
    return true;
}



bool RobotManager::movePartToDiscard()
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    floor_robot_->setJointValueTarget("linear_actuator_joint", 0.0);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

    while (!moveFloorToTarget())
    {
        RCLCPP_ERROR(get_logger(), "Unable to move tray to AGV");
        // return false;
    }
    geometry_msgs::msg::Pose pose;
    pose.position.x = -2.2;
    pose.position.y = 0;
    pose.position.z = 1.0;
    auto agv_rotation = 0.0;

    waypoints.clear();
    waypoints.push_back(utils.build_pose(pose.position.x, pose.position.y,
                                          pose.position.z ,  setRobotOrientation(agv_rotation)));

    while (!moveThroughWaypoints(waypoints, 0.2, 0.1))
    {
        RCLCPP_ERROR(get_logger(), "Unable to move part to quadrant");
        // return false;
    }
    return true;
}




bool RobotManager::movePartToQuadrant()
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(current_task_.agv_number)]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);
    if(retry){
      return false;
    }
    while (!moveFloorToTarget())
    {   
        if(retry){
          return false;
        }
        RCLCPP_ERROR(get_logger(), "Unable to move tray to AGV");
        // return false;
    }
    auto agv_tray_pose = getPoseInWorldFrame("agv" + std::to_string(current_task_.agv_number) + "_tray");
    
    agv_tray_pose.position.x += quad_offsets_[current_task_.quadrant].first;
    agv_tray_pose.position.y += quad_offsets_[current_task_.quadrant].second;

    RCLCPP_INFO_STREAM(get_logger(), BLUE<<"Quadrant: "<< std::to_string(current_task_.quadrant) <<RESET);
    RCLCPP_INFO_STREAM(get_logger(), BLUE<<"Pose: " << agv_tray_pose.position.x << " " << agv_tray_pose.position.y << " " << agv_tray_pose.position.z<<RESET);

    auto agv_rotation = 1.570807;

    

    waypoints.clear();
    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + 0.3,  setRobotOrientation(agv_rotation)));

    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + part_heights_[current_task_.part.type],  setRobotOrientation(agv_rotation)));

    double offset=0.001;
    if(retry){
          return false;
        }
    while (!moveThroughWaypoints(waypoints, 0.2, 0.1))
    {   
        if(retry){
          return false;
        }
        RCLCPP_ERROR(get_logger(), "Unable to move part to quadrant");

        waypoints.clear();
        waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                              agv_tray_pose.position.z + 0.3 + offset,  setRobotOrientation(agv_rotation)));

        waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                              agv_tray_pose.position.z + part_heights_[current_task_.part.type],  setRobotOrientation(agv_rotation)));
        
        offset += 0.001;
        // return false;
    }
    return true;
}


//=============================================//
bool RobotManager::changeGripper(std::string changing_station,std::string gripper_type) {

  auto tc_pose = getPoseInWorldFrame(changing_station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(utils.build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        setRobotOrientation(0.0)));

  waypoints.push_back(utils.build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z,
                                        setRobotOrientation(0.0)));

  if (!moveThroughWaypoints(waypoints, 0.2, 0.1)) return false;

  std::string srv_name = "ariac/floor_robot_change_gripper";

  // Create the client
  rclcpp::Client<ariac_msgs::srv::ChangeGripper>::SharedPtr client =
      this->create_client<ariac_msgs::srv::ChangeGripper>(srv_name,
      rmw_qos_profile_services_default,
      service_callback_group_);

  // Create the request
  auto request = std::make_shared<ariac_msgs::srv::ChangeGripper::Request>();

  if (gripper_type == "trays") {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::TRAY_GRIPPER;
        current_grip = 1;
  } else if (gripper_type == "parts") {
    request->gripper_type =
        ariac_msgs::srv::ChangeGripper::Request::PART_GRIPPER;
        current_grip = 0;
  }

  // Send the synchronous request
  auto result = client->async_send_request(request);

  result.wait();

  if (!result.get()->success)
  {
      RCLCPP_ERROR(get_logger(), "Error calling gripper change service");
      return false;
  }

  waypoints.clear();
  waypoints.push_back(utils.build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        setRobotOrientation(0.0)));

  if (!moveThroughWaypoints(waypoints, 0.2, 0.1)) return false;

  return true;
}


bool RobotManager::moveThroughWaypoints(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
{
    moveit_msgs::msg::RobotTrajectory trajectory;

    double path_fraction = floor_robot_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (path_fraction < 0.9)
    {
        RCLCPP_ERROR(get_logger(), "Unable to generate trajectory through waypoints");
        return false;
    }
    // Retime trajectory
    robot_trajectory::RobotTrajectory rt(floor_robot_->getCurrentState()->getRobotModel(), "floor_robot");
    rt.setRobotTrajectoryMsg(*floor_robot_->getCurrentState(), trajectory);
    totg_.computeTimeStamps(rt, vsf, asf);
    rt.getRobotTrajectoryMsg(trajectory);

    return static_cast<bool>(floor_robot_->execute(trajectory));
}



geometry_msgs::msg::Pose RobotManager::getPoseInWorldFrame(std::string frame_id)
{
    geometry_msgs::msg::TransformStamped t;
    geometry_msgs::msg::Pose pose;
    try {
        t = tf_buffer->lookupTransform("world", frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(get_logger(), "Could not get transform");
    }
    pose.position.x = t.transform.translation.x;
    pose.position.y = t.transform.translation.y;
    pose.position.z = t.transform.translation.z;
    pose.orientation = t.transform.rotation;

    return pose;
}

//=============================================//
geometry_msgs::msg::Quaternion RobotManager::setRobotOrientation(double rotation)
{
    tf2::Quaternion tf_q;
    tf_q.setRPY(0, 3.14159, rotation);

    geometry_msgs::msg::Quaternion q;

    q.x = tf_q.x();
    q.y = tf_q.y();
    q.z = tf_q.z();
    q.w = tf_q.w();

    return q;
}

bool RobotManager::moveFloorToTarget() {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_->plan(plan));

  if (success) {
    return static_cast<bool>(floor_robot_->execute(plan));
  } else {
    RCLCPP_ERROR(floor_node_->get_logger(), "Unable to generate plan");
    return false;
  }
}


void RobotManager::addSingleModelToPlanningScene(
    std::string name, std::string mesh_file,
    geometry_msgs::msg::Pose model_pose) {
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("rwa5_group3");
  std::stringstream path;
  path << "file://" << package_share_directory << "/meshes/" << mesh_file;
  std::string model_path = path.str();

  shapes::Mesh *m = shapes::createMeshFromResource(model_path);
  shapes::constructMsgFromShape(m, mesh_msg);

  mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision.meshes.push_back(mesh);
  collision.mesh_poses.push_back(model_pose);

  collision.operation = collision.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision);

  planning_scene_.addCollisionObjects(collision_objects);
}

void RobotManager::addModelsToPlanningScene()
{
  // Add bins
  std::map<std::string, std::pair<double, double>> bin_positions = {{"bin1", std::pair<double, double>(-1.9, 3.375)},
                                                                    {"bin2", std::pair<double, double>(-1.9, 2.625)},
                                                                    {"bin3", std::pair<double, double>(-2.65, 2.625)},
                                                                    {"bin4", std::pair<double, double>(-2.65, 3.375)},
                                                                    {"bin5", std::pair<double, double>(-1.9, -3.375)},
                                                                    {"bin6", std::pair<double, double>(-1.9, -2.625)},
                                                                    {"bin7", std::pair<double, double>(-2.65, -2.625)},
                                                                    {"bin8", std::pair<double, double>(-2.65, -3.375)}};

  geometry_msgs::msg::Pose bin_pose;
  for (auto const &bin : bin_positions)
  {
    bin_pose.position.x = bin.second.first;
    bin_pose.position.y = bin.second.second;
    bin_pose.position.z = 0;
    bin_pose.orientation = utils.get_quaternion_from_euler(0, 0, 3.14159);

    addSingleModelToPlanningScene(bin.first, "bin.stl", bin_pose);
  }

  // Add assembly stations
  std::map<std::string, std::pair<double, double>> assembly_station_positions = {
      {"as1", std::pair<double, double>(-7.3, 3)},
      {"as2", std::pair<double, double>(-12.3, 3)},
      {"as3", std::pair<double, double>(-7.3, -3)},
      {"as4", std::pair<double, double>(-12.3, -3)},
  };

  geometry_msgs::msg::Pose assembly_station_pose;
  for (auto const &station : assembly_station_positions)
  {
    assembly_station_pose.position.x = station.second.first;
    assembly_station_pose.position.y = station.second.second;
    assembly_station_pose.position.z = 0;
    assembly_station_pose.orientation = utils.get_quaternion_from_euler(0, 0, 0);

    addSingleModelToPlanningScene(
        station.first, "assembly_station.stl", assembly_station_pose);
  }

  // Add assembly briefcases
  std::map<std::string, std::pair<double, double>> assembly_insert_positions = {
      {"as1_insert", std::pair<double, double>(-7.7, 3)},
      {"as2_insert", std::pair<double, double>(-12.7, 3)},
      {"as3_insert", std::pair<double, double>(-7.7, -3)},
      {"as4_insert", std::pair<double, double>(-12.7, -3)},
  };

  geometry_msgs::msg::Pose assembly_insert_pose;
  for (auto const &insert : assembly_insert_positions)
  {
    assembly_insert_pose.position.x = insert.second.first;
    assembly_insert_pose.position.y = insert.second.second;
    assembly_insert_pose.position.z = 1.011;
    assembly_insert_pose.orientation = utils.get_quaternion_from_euler(0, 0, 0);

    addSingleModelToPlanningScene(insert.first, "assembly_insert.stl",
                                       assembly_insert_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose = geometry_msgs::msg::Pose();
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = utils.get_quaternion_from_euler(0, 0, 0);

  addSingleModelToPlanningScene("conveyor", "conveyor.stl",
                                     conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = utils.get_quaternion_from_euler(0, 0, 3.14159);

  addSingleModelToPlanningScene("kts1_table", "kit_tray_table.stl",
                                     kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = utils.get_quaternion_from_euler(0, 0, 0);

  addSingleModelToPlanningScene("kts2_table", "kit_tray_table.stl",
                                     kts2_table_pose);
}

//=====================================================================================================================
void RobotManager::floorGripperStateCB(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{   
  if(floor_gripper_state_.attached!=msg->attached){
      RCLCPP_INFO_STREAM(get_logger(), PURPLE<<"================================================================================"<<RESET);
      RCLCPP_INFO_STREAM(get_logger(), PURPLE<<"Gripper State Changed: "<< msg->attached<<RESET);
      RCLCPP_INFO_STREAM(get_logger(), PURPLE<<"================================================================================"<<RESET);

  }

  // Store the gripper state
  floor_gripper_state_ = *msg;
    
  if(pick_and_place && !floor_gripper_state_.attached){
    RCLCPP_INFO_STREAM(get_logger(), PURPLE<<"================================================================================"<<RESET);
    RCLCPP_INFO_STREAM(get_logger(), RED<<"Part Dropped Stoping Robot"<<RESET);
    RCLCPP_INFO_STREAM(get_logger(), PURPLE<<"================================================================================"<<RESET);

    floor_robot_->stop();
    pick_and_place = false;
    retry = true;

  }

    // floor_gripper_state_.attached = true; if the part is attached
}

//=====================================================================================================================
bool RobotManager::setGripperState(bool enable) {
  // true: enable gripper to pick part, false: disable gripper drop part


  // Service name
  std::string srv_name = "ariac/floor_robot_enable_gripper";

  // Create the client
  rclcpp::Client<ariac_msgs::srv::VacuumGripperControl>::SharedPtr client =
      this->create_client<ariac_msgs::srv::VacuumGripperControl>(srv_name,
      rmw_qos_profile_services_default,
      service_callback_group_);

  // Create the request
  auto request = std::make_shared<ariac_msgs::srv::VacuumGripperControl::Request>();

  request->enable = enable;

  // Send the synchronous request
  auto result = client->async_send_request(request);

  result.wait();

  if(result.get()->success){
    RCLCPP_INFO(get_logger(), "Gripper enabled successfully");
  } else {
    RCLCPP_ERROR(get_logger(), "Failed to enable gripper");
    return false;
  }

  return true;
}



// ================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto floor_robot_node = std::make_shared<RobotManager>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(floor_robot_node);

    try {
        executor.spin();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        executor.cancel();
        rclcpp::shutdown();
    }


    
}