#ifndef PLANNER_HPP
#define PLANNER_HPP

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
#include <msgs/filteroutput.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <sensor_msgs/LaserScan.h>
#include <mutex>
#include <limits>
#include <thread>
#include <unistd.h>

#include <kinematics/model.hpp>
#include <kinematics/models/model_differential.hpp>
#include <kinematics/pose.hpp>
#include <planner/trajectory.hpp>
#include <planner/decisionrules/decision_distance.hpp>
#include <planner/decisionrules/decision_yaw.hpp>
#include <planner/config.hpp>
#include <planner/timer.hpp>
#include <planner/plannerdata.hpp>
#include <planner/planningalgorithm.hpp>
#include <planner/algorithms/trajectoryrollout.hpp>
#include <planner/algorithms/roadmiddle.hpp>
#include <planner/algorithms/ripper.hpp>
#include <planner/algorithms/recoveryback.hpp>
#include <planner/algorithms/dwa.hpp>

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
