#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <sensor_msgs/msg/Image.hpp>
#include <sensor_msgs/msg/CameraInfo.hpp>
#include <std_msgs/msg/Float64.hpp>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf2/transform_broadcaster.hpp>
#include <msgs/filteroutput.hpp>
#include <nav_msgs/msg/Odometry.hpp>
#include <nav_msgs/msg/OccupancyGrid.hpp>
#include <move_base_msgs/msg/MoveBaseActionGoal.hpp>
#include <sensor_msgs/msg/LaserScan.hpp>
#include <mutex>
#include <limits>
#include <thread>
#include <unistd.h>

#include <kinematics/model.hpp>
#include <kinematics/models/model_differential.hpp>
#include <kinematics/pose.hpp>
#include <local_planner/trajectory.hpp>
#include <local_planner/decisionrules/decision_distance.hpp>
#include <local_planner/decisionrules/decision_yaw.hpp>
#include <local_planner/config.hpp>
#include <local_planner/timer.hpp>
#include <local_planner/plannerdata.hpp>
#include <local_planner/planningalgorithm.hpp>
#include <local_planner/algorithms/trajectoryrollout.hpp>
#include <local_planner/algorithms/roadmiddle.hpp>
#include <local_planner/algorithms/ripper.hpp>
#include <local_planner/algorithms/recoveryback.hpp>
#include <local_planner/algorithms/dwa.hpp>

/**
 @brief Planner main class
 **/
class Planner
{
public:
 /**
  @brief Initializes planner
  @param config Planner configuration
  @param model Robot motion model
  @param decision Trajectory planner decision rule
  **/
 Planner(PlannerConfig* config, Model* model, DecisionRule* decision);
  ~Planner();

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
   @brief Updates road middle and critical flag
   @param msg Middle filter data
   **/
  void setRoadmiddle(const msgs::filteroutput& msg);

  /**
   @brief Informs if a goal is set
   @retval goal set
   **/
  bool hasGoal() { return plannerData.goalSet; }

  /**
   @brief General planner rendering function for debugging.
   **/
  cv::Mat visualize();

  /**
   @brief Function for getting motion commands
   @retval twist message
   **/
  geometry_msgs::Twist getCmdVel();

private:

  DecisionRule* decision;
  PlannerData plannerData;

  Algo_Trajectoryrollout* trajectoryPlanner;
  Algo_DWA* dwaPlanner;
  Algo_Roadmiddle* middlePlanner;
  Algo_Ripper* ripperPlanner;
  Algo_Recoveryback* recoveryBack;

  std::thread thread;
  bool threadStop;
  void worker();

  std::vector<PlanningAlgorithm*> algos;
  std::vector<PlanningAlgorithm*> recoveries;
  std::vector<PlanningAlgorithm::Outcomes> results;

  int defaultId;

  /**
   @brief Executes core planning step
   **/
  void plan();
  
  /**
   @brief Adds algorithm to planner hierachy.
   @param algo Algorithm
   @param useDefault use as default is all algorithms are uncritical
   **/
  void addAlgo(PlanningAlgorithm* algo, bool useDefault);

  void addAlgoDecode(PlanningAlgorithm* algo, int enableMask, int defaultMask);

  /**
   @brief Adds recovery algorithm to planner. Recovery algorithms are mapped by Id to the planning algorithms.
   @param recovery Recovery algorithm
   **/
  void addRecovery(PlanningAlgorithm* recovery);

};
#endif
