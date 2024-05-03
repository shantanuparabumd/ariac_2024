#include "rwa4_group3/robot_manager.hpp"
#include <string>
#include <unistd.h>

RobotManager::RobotManager() : Node("robot_manager"),
     floor_node_(std::make_shared<rclcpp::Node>("floor_group_node")),
     executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>()),
     planning_scene_()
{   

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
    robot_status_publisher_ = this->create_publisher<rwa4_group3::msg::RobotStatus>(
      "robot_status", 10);

    // Subscriber to get robot task
    robot_task_subscription_ = this->create_subscription<rwa4_group3::msg::RobotTask>(
        "robot_task", 10, std::bind(&RobotManager::robot_task_callback, this, std::placeholders::_1));

    // Callback group for service calls
    service_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    RobotManager::add_models_to_planning_scene();
}

//=====================================================================================================================
RobotManager::~RobotManager()
{
    floor_robot_->~MoveGroupInterface();
}



//=====================================================================================================================
void RobotManager::robot_task_callback(const rwa4_group3::msg::RobotTask::SharedPtr msg)
{   
  current_task_ = *msg;


  // Check if the task is to pick up a tray
  if(msg->task_type == rwa4_group3::msg::RobotTask::TRAY){
    RCLCPP_INFO(this->get_logger(), "Picking Up Tray");
    
    // Call the function to pick up the tray
    FloorRobotPickTray(msg->pose,msg->agv_number,msg->tray_id);
  }

  // Check if the task is to pick up a part
  else if(msg->task_type == rwa4_group3::msg::RobotTask::PART){
    RCLCPP_INFO(this->get_logger(), "Picking Up Part");

    FloorRobotPickPart(msg->pose);
  }
  
}


//=====================================================================================================================
void RobotManager::FloorRobotPickTray(geometry_msgs::msg::Pose pose,int agv_number, int tray_id)
{    

    // Check if the gripper is already in the correct state
    if(current_grip == 0){
        
    // Move to the closest tray station
    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget(floor_kts2_js_);
      move_floor_to_target_();
      // Change Gripper
      change_gripper_("kts2","trays");
    }
    else{
      floor_robot_->setJointValueTarget(floor_kts1_js_);
      move_floor_to_target_();
      // Change Gripper
      change_gripper_("kts1","trays");
    }
    
    }

    // Move to the closest tray station
    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget(floor_kts2_js_);
      move_floor_to_target_();
    }
    else{
      floor_robot_->setJointValueTarget(floor_kts1_js_);
      move_floor_to_target_();
    }

    // Enable the gripper to pick up the tray
    set_gripper_state_(true);

    // Pick Tray
    RCLCPP_INFO_STREAM(get_logger(), "Picking up tray at "<< pose.position.x << " " << pose.position.y << " " << pose.position.z);
    pick(pose,"tray");

    // Add kit tray to planning scene
    std::string tray_name = "kit_tray_" + std::to_string(tray_id);
    add_single_model_to_planning_scene_(tray_name, "kit_tray.stl", pose);
    floor_robot_->attachObject(tray_name);


    // Lift the tray up slightly
    double tray_rotation =utils.get_yaw_from_pose_(pose);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    // Move up slightly
    waypoints.clear();
    waypoints.push_back(utils.build_pose(
        pose.position.x, pose.position.y, pose.position.z + 0.2,
        set_robot_orientation_(tray_rotation)));

    if (!move_through_waypoints_(waypoints, 0.2, 0.1)) {
      RCLCPP_ERROR(get_logger(), "Unable to move up");
    }


    // Move the tray to the AGV based on order
    move_tray_to_agv(agv_number);

    // Place Tray on AGV and detach
    set_gripper_state_(false);

    floor_robot_->detachObject(tray_name);

    RCLCPP_INFO(get_logger(), "Tray placed on AGV");
    // Set the robot status to free    
    rwa4_group3::msg::RobotStatus robot_status_msg;
    robot_status_msg.floor_robot = rwa4_group3::msg::RobotStatus::FREE;
    robot_status_publisher_->publish(robot_status_msg);

}



//=====================================================================================================================
void RobotManager::FloorRobotPickPart(geometry_msgs::msg::Pose pose)
{    

    // Check if the gripper is already in the correct state
    if(current_grip == 1){
        
    // Move to the closest tray station
    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget(floor_kts2_js_);
      move_floor_to_target_();
      // Change Gripper
      change_gripper_("kts2","parts");
    }
    else{
      floor_robot_->setJointValueTarget(floor_kts1_js_);
      move_floor_to_target_();
      // Change Gripper
      change_gripper_("kts1","parts");
    }
    
    }

    // Move to the closest tray station
    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget(floor_kts2_js_);
      move_floor_to_target_();
    }
    else{
      floor_robot_->setJointValueTarget(floor_kts1_js_);
      move_floor_to_target_();
    }

    

    if(pose.position.y > 0.0){
      floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["right_bins"]);
      move_floor_to_target_();
    }
    else{
      floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["left_bins"]);
      move_floor_to_target_();
    }

    // Enable the gripper to pick up the tray
    set_gripper_state_(true);

    // // Pick Tray
    RCLCPP_INFO_STREAM(get_logger(), "Picking up part at "<< pose.position.x << " " << pose.position.y << " " << pose.position.z);
    pick(pose,"part");

    // // Add kit tray to planning scene
    std::string part_name = part_names_[current_task_.part.type];
    std::string mesh = part_names_[current_task_.part.type]+".stl";
    add_single_model_to_planning_scene_(part_name, mesh, pose);
    floor_robot_->attachObject(part_name);


    // Lift the tray up slightly
    double tray_rotation =utils.get_yaw_from_pose_(pose);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    // Move up slightly
    waypoints.clear();
    waypoints.push_back(utils.build_pose(
        pose.position.x, pose.position.y, pose.position.z + 0.2,
        set_robot_orientation_(tray_rotation)));

    if (!move_through_waypoints_(waypoints, 0.2, 0.2)) {
      RCLCPP_ERROR(get_logger(), "Unable to move up");
    }


    // // Move the tray to the AGV based on order
    move_part_to_quadrant();

    // // Place Tray on AGV and detach
    set_gripper_state_(false);

    floor_robot_->detachObject(part_name);

    RCLCPP_INFO(get_logger(), "Part placed on AGV");
    // Set the robot status to free    
    rwa4_group3::msg::RobotStatus robot_status_msg;
    robot_status_msg.floor_robot = rwa4_group3::msg::RobotStatus::FREE;
    robot_status_publisher_->publish(robot_status_msg);

}




// Note: This is not being used for now
//=============================================//
void RobotManager::wait_for_attach_completion_(double timeout) {
  // Wait for part to be attached
  rclcpp::Time start = now();
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose starting_pose = floor_robot_->getCurrentPose().pose;

  while (!floor_gripper_state_.attached) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Waiting for gripper attach");

    waypoints.clear();
    starting_pose.position.z -= 0.001;
    waypoints.push_back(starting_pose);

    move_through_waypoints_(waypoints, 0.1, 0.1);

    usleep(200);

    if (now() - start > rclcpp::Duration::from_seconds(timeout)) {
      RCLCPP_ERROR(get_logger(), "Unable to pick up object");
      return;
    }
  }
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
                                     pose.position.z + 0.4, set_robot_orientation_(yaw)));

    double offset = 0.0;
    if(type=="part"){
        offset = part_heights_[current_task_.part.type];
        RCLCPP_INFO(get_logger(), "Part height: %f", offset);
    }
    else if(type=="tray"){
        offset = 0.0;
    }
    waypoints.push_back(utils.build_pose(pose.position.x, pose.position.y,
                                          pose.position.z + offset , set_robot_orientation_(yaw)));

    if (!move_through_waypoints_(waypoints, 0.2, 0.1))
        return false;
    
    // wait_for_attach_completion_(10.0);

    return true;

}


bool RobotManager::move_tray_to_agv(int agv_number)
{   


    RCLCPP_INFO_STREAM(get_logger(), ORANGE<<"Moving tray to AGV: " << std::to_string(agv_number)<<RESET);
    std::vector<geometry_msgs::msg::Pose> waypoints;
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(agv_number)]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

    if (!move_floor_to_target_())
    {
        RCLCPP_ERROR_STREAM(get_logger(), RED<<"Unable to move tray to AGV"<<RESET);
        return false;
    }
    auto agv_tray_pose = get_pose_in_world_frame_("agv" + std::to_string(agv_number) + "_tray");
    
    auto agv_rotation = 1.570807;

    RCLCPP_INFO_STREAM(get_logger()," Moving to top of AGV: " << std::to_string(agv_number));

    waypoints.clear();
    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + 0.3, set_robot_orientation_(agv_rotation)));

    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + kit_tray_thickness_ + drop_height_, set_robot_orientation_(agv_rotation)));

    if (!move_through_waypoints_(waypoints, 0.2, 0.1))
    {
        RCLCPP_ERROR(get_logger(), "Unable to move tray to AGV");
        return false;
    }
    return true;
}




bool RobotManager::move_part_to_quadrant()
{
    std::vector<geometry_msgs::msg::Pose> waypoints;
    floor_robot_->setJointValueTarget("linear_actuator_joint", rail_positions_["agv" + std::to_string(current_task_.agv_number)]);
    floor_robot_->setJointValueTarget("floor_shoulder_pan_joint", 0);

    while (!move_floor_to_target_())
    {
        RCLCPP_ERROR(get_logger(), "Unable to move tray to AGV");
        // return false;
    }
    auto agv_tray_pose = get_pose_in_world_frame_("agv" + std::to_string(current_task_.agv_number) + "_tray");
    
    agv_tray_pose.position.x += quad_offsets_[current_task_.quadrant].first;
    agv_tray_pose.position.y += quad_offsets_[current_task_.quadrant].second;

    RCLCPP_INFO_STREAM(get_logger(), BLUE<<"Quadrant: "<< std::to_string(current_task_.quadrant) <<RESET);
    RCLCPP_INFO_STREAM(get_logger(), BLUE<<"Pose: " << agv_tray_pose.position.x << " " << agv_tray_pose.position.y << " " << agv_tray_pose.position.z<<RESET);

    auto agv_rotation = 1.570807;

    waypoints.clear();
    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + 0.3,  set_robot_orientation_(agv_rotation)));

    waypoints.push_back(utils.build_pose(agv_tray_pose.position.x, agv_tray_pose.position.y,
                                          agv_tray_pose.position.z + part_heights_[current_task_.part.type] + drop_height_,  set_robot_orientation_(agv_rotation)));

    while (!move_through_waypoints_(waypoints, 0.2, 0.1))
    {
        RCLCPP_ERROR(get_logger(), "Unable to move part to quadrant");
        // return false;
    }
    return true;
}


//=============================================//
bool RobotManager::change_gripper_(std::string changing_station,std::string gripper_type) {

  auto tc_pose = get_pose_in_world_frame_(changing_station + "_tool_changer_" + gripper_type + "_frame");

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(utils.build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z + 0.4,
                                        set_robot_orientation_(0.0)));

  waypoints.push_back(utils.build_pose(tc_pose.position.x, tc_pose.position.y,
                                        tc_pose.position.z,
                                        set_robot_orientation_(0.0)));

  if (!move_through_waypoints_(waypoints, 0.2, 0.1)) return false;

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
                                        set_robot_orientation_(0.0)));

  if (!move_through_waypoints_(waypoints, 0.2, 0.1)) return false;

  return true;
}


bool RobotManager::move_through_waypoints_(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf)
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



geometry_msgs::msg::Pose RobotManager::get_pose_in_world_frame_(std::string frame_id)
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
geometry_msgs::msg::Quaternion RobotManager::set_robot_orientation_(double rotation)
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

bool RobotManager::move_floor_to_target_() {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(floor_robot_->plan(plan));

  if (success) {
    return static_cast<bool>(floor_robot_->execute(plan));
  } else {
    RCLCPP_ERROR(floor_node_->get_logger(), "Unable to generate plan");
    return false;
  }
}


void RobotManager::add_single_model_to_planning_scene_(
    std::string name, std::string mesh_file,
    geometry_msgs::msg::Pose model_pose) {
  moveit_msgs::msg::CollisionObject collision;

  collision.id = name;
  collision.header.frame_id = "world";

  shape_msgs::msg::Mesh mesh;
  shapes::ShapeMsg mesh_msg;

  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("rwa4_group3");
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

void RobotManager::add_models_to_planning_scene()
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

    add_single_model_to_planning_scene_(bin.first, "bin.stl", bin_pose);
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

    add_single_model_to_planning_scene_(
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

    add_single_model_to_planning_scene_(insert.first, "assembly_insert.stl",
                                       assembly_insert_pose);
  }

  geometry_msgs::msg::Pose conveyor_pose = geometry_msgs::msg::Pose();
  conveyor_pose.position.x = -0.6;
  conveyor_pose.position.y = 0;
  conveyor_pose.position.z = 0;
  conveyor_pose.orientation = utils.get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene_("conveyor", "conveyor.stl",
                                     conveyor_pose);

  geometry_msgs::msg::Pose kts1_table_pose;
  kts1_table_pose.position.x = -1.3;
  kts1_table_pose.position.y = -5.84;
  kts1_table_pose.position.z = 0;
  kts1_table_pose.orientation = utils.get_quaternion_from_euler(0, 0, 3.14159);

  add_single_model_to_planning_scene_("kts1_table", "kit_tray_table.stl",
                                     kts1_table_pose);

  geometry_msgs::msg::Pose kts2_table_pose;
  kts2_table_pose.position.x = -1.3;
  kts2_table_pose.position.y = 5.84;
  kts2_table_pose.position.z = 0;
  kts2_table_pose.orientation = utils.get_quaternion_from_euler(0, 0, 0);

  add_single_model_to_planning_scene_("kts2_table", "kit_tray_table.stl",
                                     kts2_table_pose);
}

//=====================================================================================================================
void RobotManager::floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg)
{   
    // Store the gripper state
    floor_gripper_state_ = *msg;

    // floor_gripper_state_.attached = true; if the part is attached
}

//=====================================================================================================================
bool RobotManager::set_gripper_state_(bool enable) {
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