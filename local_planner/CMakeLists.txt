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
find_package(image_transport REQUIRED)
find_package(lois_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  /opt/ros/humble/include
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
    image_transport
    lois_interfaces
    OpenCV
    pcl_conversions
)
#libpcl-conversions-dev

include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS} ${PCL_DEFINITIONS})
target_link_libraries(${TARGET} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS
  ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
