#include <local_planner/config.hpp>

std::string PlannerConfig::str()
{
  std::string s = 
  "Set up trajectory planner\n" +
  std::to_string(trajectories) + " trajectories with length of " + 
  std::to_string(trajectoryLength_m) + "m\n" +
  std::to_string(planningStep_secs) + "s planning step and " + 
  std::to_string(planningHorizon_secs) + "s planning horizon\n" + 
  + "Trajecotry critical with " + std::to_string(trajectoryCrit_deg) + " deg\n" + 
  "Bot dimensions: " + std::to_string(botWidth_m) + "mx" +
  std::to_string(botHeight_m) + "m \nGrid dimensions: " +
  std::to_string(gridWidth_m) + "mx" +
  std::to_string(gridHeight_m) + "m\n" +
  "Bot vMax: " +
  std::to_string(vxMax) + "m/s, wMax: " + 
  std::to_string(wMax) + "rad/s\n" + 
  "Collision threshold: " + std::to_string(collision_th) + 
  "Avg switch: " + std::to_string(avg_switch) + 
  ", Avg p: " + std::to_string(avg_p) + "\n" + 
  "Rate: " + std::to_string(rate_ms) + "m/s\n" + 
  "Lidaroffset: (" + std::to_string(lidarOffset.x) + "," + std::to_string(lidarOffset.y) + ")";

  return s;
}
