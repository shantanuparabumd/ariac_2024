cmake_minimum_required(VERSION 3.8)
project(final_group3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)


include_directories(include)

#--------------------------------
# Python
#--------------------------------

find_package(rclpy REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)

set(msg_files
  "msg/RobotTask.msg"
  "msg/RobotStatus.msg"
)



rosidl_generate_interfaces(${PROJECT_NAME}
${msg_files}

DEPENDENCIES
builtin_interfaces
geometry_msgs
std_msgs
ariac_msgs
)

set(GROUP3_INCLUDE_DEPENDS
  rclcpp 
  geometry_msgs
  tf2
  orocos_kdl
  tf2_ros
  tf2_geometry_msgs
  ariac_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  moveit_msgs
  moveit_ros_planning_interface
)

# add_library(utils src/utils.cpp)
# ament_target_dependencies(utils ${GROUP3_INCLUDE_DEPENDS})


add_executable(ccs_manager src/ccs.cpp src/utils.cpp)

add_executable(sensor_manager src/sensors.cpp src/utils.cpp)

add_executable(sensor_manager_bonus src/sensors_bonus.cpp src/utils.cpp)

add_executable(robot_manager src/robot_manager.cpp src/utils.cpp)




ament_target_dependencies(ccs_manager  ${GROUP3_INCLUDE_DEPENDS})
ament_target_dependencies(sensor_manager  ${GROUP3_INCLUDE_DEPENDS}) 
ament_target_dependencies(sensor_manager_bonus  ${GROUP3_INCLUDE_DEPENDS}) 
ament_target_dependencies(robot_manager  ${GROUP3_INCLUDE_DEPENDS})




install(TARGETS 
  ccs_manager
  sensor_manager
  sensor_manager_bonus
  robot_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY 
  config
  launch
  scripts
  meshes
  DESTINATION share/${PROJECT_NAME}/
)




rosidl_target_interfaces(ccs_manager
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(robot_manager
      ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

#--------------------------------
# Python
#--------------------------------

ament_python_install_package(${PROJECT_NAME})

install(
  PROGRAMS
  scripts/advance_sensor_interface.py
  launch/final.launch.py
  launch/final_bonus.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
