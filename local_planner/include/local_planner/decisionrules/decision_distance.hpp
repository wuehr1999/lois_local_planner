#ifndef DECISION_DISTANCE_HPP
#define DECISION_DISTANCE_HPP

#include <planner/decisionrule.hpp>

/**
 @brief Rate trajectory based on euclidian distance to goal point
 **/
class Decision_Distance : public DecisionRule
{
public:
  Decision_Distance():DecisionRule() {}

  int decide(Trajectory* trajectory, Pose* goal, Pose* odom)
  {
    Pose* p = &trajectory->poses[trajectory->poses.size() - 1];
  
    Pose current;

    double s = sin(odom->yaw - 0.5 * M_PI);
    double c = cos(odom->yaw - 0.5 * M_PI);

    current.x = p->x * c - p->y * s;
    current.y = p->x * s + p->y * c;
    current.yaw = odom->yaw - 0.5 * M_PI + p->yaw; 
    current.x += odom->x;
    current.y += odom->y;
    return (int)(100.0 * sqrt(pow((goal->x - current.x), 2) + pow((goal->y - current.y), 2)));
  }
};
#endif
