cmake_minimum_required(VERSION 3.8)

project(local_planner)
set(TARGET ${PROJECT_NAME}_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf_listener REQUIRED)
find_package(kinematics REQUIRED)

include_directories(
  include
)

add_executable(${TARGET}
  src/config.cpp
  src/planner.cpp
  src/plannerdata.cpp
  src/node.cpp
  src/planningalgorithm.cpp
  src/timer.cpp
  src/trajectory.cpp
  src/main.cpp
)

target_compile_features(${TARGET} PRIVATE cxx_std_17)

ament_target_dependencies(${TARGET}
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf_listener
    kinematics
)


install(TARGETS
  ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
