#ifndef PLANNER_CONFIG_HPP
#define PLANNER_CONFIG_HPP

#include <string>
#include <kinematics/pose.hpp>

#define CONFIG_ENABLE_BUMPER      0
#define CONFIG_ENABLE_DWA         1
#define CONFIG_ENABLE_ROADMIDDLE  2 
#define CONFIG_DEFAULT_BUMPER     3
#define CONFIG_DEFAULT_DWA        4
#define CONFIG_DEFAULT_ROADMIDDLE 5
#define CONFIG_SWITCH_DWA_MIDDLE  6
/**
 @brief Class for holding planner configuration parameters
 **/
class PlannerConfig
{
public:
  int trajectories;
  int trajectoryLayers;
  double trajectoryLength_m;
  double trajectoryCrit_deg;
  double planningHorizon_secs, planningStep_secs;
  double botWidth_m, botHeight_m;
  double gridWidth_m, gridHeight_m;
  double vxMin, vxMax, wMax;
  double aXMax, aYMax, aYawMax;
  double collision_th;
  double check_startfactor;
  double boxX_m, boxY_m;
  double vxRecovery, wRecovery;
  int dilation_size;
  int opening_size;
  int configMask;
  bool debug;
  bool overlay_grid;
  double avg_switch, avg_hyst, avg_p; 
  double rate_ms;
  Pose lidarOffset, lidarOffset2;
  double angleOffset;
  std::string str();
};
#endif
