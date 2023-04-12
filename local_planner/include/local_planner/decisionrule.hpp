#ifndef DECISIONRULE_HPP
#define DECISIONRULE_HPP

#include <kinematics/pose.hpp>
#include "trajectory.hpp"

/**
 @brief base class for defining decision rules
 **/
class DecisionRule
{
public:
  DecisionRule(){};
  virtual ~DecisionRule(){};

  /**
   @brief Decision function for trajectories. 
   @param trajectory Pointer to trajectory
   @param goal Pointer to goal
   @param odom Pointer to robot position
   @return Integer weight for comparism
   **/
  virtual int decide(Trajectory* trajectory, Pose* goal, Pose* odom) = 0;
};
#endif
