#pragma once

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rwa5_group3/msg/robot_task.hpp>
#include <rwa5_group3/msg/robot_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "rwa5_group3/utils.hpp"

#include <ariac_msgs/srv/perform_quality_check.hpp>
#include <ariac_msgs/msg/quality_issue.hpp>

#include <ariac_msgs/srv/change_gripper.hpp>
#include <ariac_msgs/srv/vacuum_gripper_control.hpp>
#include <ariac_msgs/msg/vacuum_gripper_state.hpp>
#include <ariac_msgs/msg/kitting_part.hpp>

// TF2
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "utils.hpp"

// KDL
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>


// Adding Collision Objects
#include <moveit_msgs/msg/collision_object.hpp>
// Messages
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/msg/mesh.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "../include/rwa5_group3/orders.hpp"

#include <unistd.h>

#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define BLUE "\033[1;34m"
#define PURPLE "\033[1;35m"
#define ORANGE "\033[1;33m"
#define CYAN "\033[1;36m"

#define RESET "\033[0m"

class RobotManager : public rclcpp::Node
{

    public:

        /**
         * @brief Construct a new Robot Manager object
         * 
         */
        RobotManager();

        /**
         * @brief Destroy the Robot Manager object
         * 
         */
        ~RobotManager();

        // Main Start Function

        /**
         * @brief Pick and Place the given tray to the given agv
         * 
         * @param pose 
         * @param agv_number 
         * @param tray_id 
         */
        void FloorRobotPickTray(geometry_msgs::msg::Pose pose, int agv_number,int tray_id);


        /**
         * @brief Pick and Place the given part to the given agv tray
         * 
         * @param pose 
         */

        bool FloorRobotPickPart(geometry_msgs::msg::Pose pose);
        // Subscriber Callbacks

        /**
         * @brief Callback function for the robot task subscription
         * Get the new task tobe executed by the robots
         * @param msg 
         */
        void robot_task_callback(const rwa5_group3::msg::RobotTask::SharedPtr msg);


        /**
         * @brief Get the status of the robot gripper
         *  The state is neccessary to identify wether the gripper is attached to the part or not
         * @param msg 
         */
        void floor_gripper_state_cb(const ariac_msgs::msg::VacuumGripperState::ConstSharedPtr msg);

        // Helper Functions Motion

        /**
         * @brief Move the robot to the given pose
         * 
         * @param pose 
         * @return true 
         * @return false 
         */
        bool move_floor_to_target_();
        

        /**
         * @brief Move the robot to the given pose using waypoints
         * 
         * @param pose 
         * @return true 
         * @return false 
         */
        bool move_through_waypoints_(std::vector<geometry_msgs::msg::Pose> waypoints, double vsf, double asf);


        /**
         * @brief Pick the tray/part at give pose
         * 
         * @param pose 
         * @return true 
         * @return false 
         */
        bool pick(geometry_msgs::msg::Pose pose, std::string type);


        /**
         * @brief Move towards the part/tray until the gripper is attached
         * 
         * @param timeout Time to wait for the gripper to attach
         */
        void wait_for_attach_completion_(double timeout);


        /**
         * @brief Move the robot to the given AGV
         * 
         * @param agv_number 
         * @return true 
         * @return false 
         */
        bool move_tray_to_agv(int agv_number);

        bool move_part_to_quadrant();


        /**
         * @brief Change the gripper of the robot
         * 
         * @param changing_station 
         * @param gripper_type 
         * @return true 
         * @return false 
         */
        bool change_gripper_(std::string changing_station, std::string gripper_type);


        // Utility Functions

        /**
         * @brief Give a frame get the pose of the frame wrt to world.
         * This function utilizes TF tree to get the pose of the frame wrt to world
         * 
         * @return geometry_msgs::msg::Pose 
         */
        geometry_msgs::msg::Pose get_pose_in_world_frame_(std::string frame_id);
        

        /**
         * @brief Set the robot orientation object
         * 
         * @param rotation 
         * @return geometry_msgs::msg::Quaternion 
         */
        geometry_msgs::msg::Quaternion set_robot_orientation_(double rotation);

        
        // Other Helper Functions

        /**
         * @brief Set the gripper state object
         * 
         * @param enable Ready for picking or not
         * @return true 
         * @return false 
         */
        bool set_gripper_state_(bool enable);


        /**
         * @brief Add a collision object to the planning scene
         * Once a object is attached to robot add it to planning scene to avoid collision
         * @param name 
         * @param mesh_file 
         * @param pose 
         */
        void add_single_model_to_planning_scene_(std::string name, std::string mesh_file, geometry_msgs::msg::Pose model_pose);
        
        
        void add_models_to_planning_scene ();

        OrderStatus PerformQualityCheck(std::string order_id);

        bool CheckFaultyPart(int quadrant);

        bool CheckMissingPart(int quadrant);

        

        bool move_part_to_discard();



    private:

        Utils utils;  

        // Callback group for storing the orders
        rclcpp::CallbackGroup::SharedPtr gripper_group_; 

        rclcpp::Publisher<rwa5_group3::msg::RobotStatus>::SharedPtr robot_status_publisher_;

        rclcpp::Subscription<rwa5_group3::msg::RobotTask>::SharedPtr robot_task_subscription_;
        
        rclcpp::Subscription<ariac_msgs::msg::VacuumGripperState>::SharedPtr floor_gripper_state_sub_;

        moveit::planning_interface::MoveGroupInterfacePtr floor_robot_;

        moveit::planning_interface::PlanningSceneInterface planning_scene_;

        
        rclcpp::Node::SharedPtr floor_node_;

        rclcpp::Executor::SharedPtr executor_;

        std::thread executor_thread_;

        rwa5_group3::msg::RobotTask current_task_;

        int current_grip = 0;

        ariac_msgs::msg::VacuumGripperState floor_gripper_state_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());

        std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg_;


        rclcpp::CallbackGroup::SharedPtr service_callback_group_;

        std::map<std::string, double> floor_kts1_js_ = {
        {"linear_actuator_joint", 4.0},
        {"floor_shoulder_pan_joint", 1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

        std::map<std::string, double> floor_kts2_js_ = {
        {"linear_actuator_joint", -4.0},
        {"floor_shoulder_pan_joint", -1.57},
        {"floor_shoulder_lift_joint", -1.57},
        {"floor_elbow_joint", 1.57},
        {"floor_wrist_1_joint", -1.57},
        {"floor_wrist_2_joint", -1.57},
        {"floor_wrist_3_joint", 0.0}};

        std::map<std::string, double> rail_positions_ = {
        {"agv1", -4.5},
        {"agv2", -1.2},
        {"agv3", 1.2},
        {"agv4", 4.5},
        {"left_bins", 3},
        {"right_bins", -3}};


        
        //! Mapping between part type constants and part heights
        std::map<int, double> part_heights_ = {
            {ariac_msgs::msg::Part::BATTERY, 0.04},
            {ariac_msgs::msg::Part::PUMP, 0.12},
            {ariac_msgs::msg::Part::REGULATOR, 0.07},
            {ariac_msgs::msg::Part::SENSOR, 0.07}};
        //! Mapping between quadrant type constants and offsets from the center of the
        //! tray
        std::map<int, std::pair<double, double>> quad_offsets_ = {
            {ariac_msgs::msg::KittingPart::QUADRANT1,
            std::pair<double, double>(-0.08, -0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT2,
            std::pair<double, double>(-0.08, 0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT3,
            std::pair<double, double>(0.08, -0.12)},
            {ariac_msgs::msg::KittingPart::QUADRANT4,
            std::pair<double, double>(0.08, 0.12)},
        };


        //! Thickness of the tray in meters
        /*! This is used to ensure the gripper does not collide with the tray when placing a part in the tray */
        double kit_tray_thickness_ = 0.02;
        //! Distance between the tray and the gripper in meters.
        /*! This is used to place the gripper at a safe distance from the tray when dropping a part in the tray */
        double drop_height_ = 0.015;
        //! Distance between the tray and the part in meters.
        /*! This is used to pick up a part */
        double pick_offset_ = 0.003;

        int part_number_= 0;

        bool pick_and_place = false;

        bool retry = false;


        std::map<int, std::string> part_names_ = {
            {ariac_msgs::msg::Part::BATTERY, "battery"},
            {ariac_msgs::msg::Part::PUMP, "pump"},
            {ariac_msgs::msg::Part::REGULATOR, "regulator"},
            {ariac_msgs::msg::Part::SENSOR, "sensor"}};

    




     
        

};