#ifndef PLANNERDATA_HPP
#define PLANNERDATA_HPP
#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <mutex>
#include <limits>
#include <thread>
#include <unistd.h>

#include <kinematics/model.hpp>
#include <kinematics/models/model_differential.hpp>
#include <kinematics/pose.hpp>
#include <msgs/filteroutput.h>
#include <planner/trajectory.hpp>
#include <planner/decisionrules/decision_distance.hpp>
#include <planner/decisionrules/decision_yaw.hpp>
#include <planner/config.hpp>
#include <planner/timer.hpp>

/**
 @brief Class for holding planner metadata passed to every algorithm
 **/
class PlannerData
{
public:
  PlannerData(){}
  ~PlannerData(){}

  PlannerConfig* config;
  Model* model;
  bool odomInit, gridInit, lidarInit, lidarInit2;
  bool goalSet;

  double avg, noway, nearleft, nearright;
  double currentV, currentW;
  sensor_msgs::LaserScan lidar;
  sensor_msgs::LaserScan lidar2;

  cv::Mat grid, inGrid, debugGrid;
  double metersPerCell;

  Pose origin_m;
  Pose p0_m;
  Pose odom_m;
  Pose goal_m;
  Pose origin0_m;
  Pose boxLeft_m, boxRight_m;

  double vX, w;

  std::mutex mu;
  std::vector<std::pair<int, int>> posesVis; 
 
  void begin();

  /**
   @brief Map meter pose to grid pixels
   @param p Position
   @param rotate Rotate with robot angle in coordinate frame
   @return <x, y> Position
   **/
  std::pair<int, int> metersToGrid(Pose p, bool rotate = true, bool fromOdom = true);
 
  /**
   @brief Check if a position collides with obstacle, bounding box possible
   @param p Position in meters
   @param width Bounding box width in meters
   @param height Bounding box height in meters
   @return collision
   **/
  bool collides(Pose p, double width, double height);

  /**
   @brief Overlays current lidar message to map.
   **/
  void overlayLidar();
  void overlayLidar2();

  /**
   @brief Updates grid
   @param msg Occupancy grid message
   **/
  void setGrid(const nav_msgs::OccupancyGrid& msg);
  
  /**
   @brief Updates robot odometry
   @param msg Odometry message
   **/
  void setOdometry(const nav_msgs::Odometry& msg);
  
  /**
   @brief Updates goal
   @param msg Goal message
   **/  
  void setGoal(const move_base_msgs::MoveBaseActionGoal& msg);
  
  /**
   @brief Updates goal
   @param dist distance in m
   @param heading in degrees 
   **/  
  void setGlobalGoal(double dist, double heading);

  /**
   @brief Updates fast sensor
   @brief msg Laser Scan sensor data
   **/
  void setLidar(const sensor_msgs::LaserScan& msg);
  void setLidar2(const sensor_msgs::LaserScan& msg);

  /**
   @brief Updates road middle
   @param msg Middle filter data
   **/  
  void setAvg(const std_msgs::Float64& msg);
  
  /**
   @brief Draws rect in visualizatiion
   @param p Position in meters
   @param width Width in meters
   @param height Height in meters
   @param col color
   **/
  void drawRect(Pose p, double width, double height, cv::Scalar col);
  
  /**
   @brief General planner rendering function for debugging.
  **/
  void visualize();
  
  /**
   @brief Set new linear velocity
   @param v Velocity in m/s
   **/
  void setV(double v);
  
  /**
   @brief Set new angular velocity
   @param w Angular velocity in rad/s
   **/
  void setW(double w);
  
  /**
   @brief Updates road middle and critical flag
   @param msg Middle filter data
   **/  
  void setRoadmiddle(const msgs::filteroutput& msg);

  /**
   @brief Function for getting motion commands
   @retval twist message
   **/
  geometry_msgs::Twist getCmdVel();
};
#endif
