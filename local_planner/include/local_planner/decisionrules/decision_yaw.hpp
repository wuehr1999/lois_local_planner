#ifndef DECISION_YAW_HPP
#define DECISION_YAW_HPP

#include <planner/decisionrule.hpp>

/**
 @brief Rate trajectory based on tangential angle to goal line
 **/
class Decision_Yaw : public DecisionRule
{
public:
  Decision_Yaw():DecisionRule() {}

  int decide(Trajectory* trajectory, Pose* goal, Pose* odom)
  {
    Pose* p = &trajectory->poses[trajectory->poses.size() -1];

    Pose current;

    double s = sin(odom->yaw - 0.5 * M_PI);
    double c = cos(odom->yaw - 0.5 * M_PI);

    current.x = p->x * c - p->y * s;
    current.y = p->x * s + p->y * c;
    current.yaw = odom->yaw - 0.5 * M_PI + p->yaw; 
    current.x += odom->x;
    current.y += odom->y;

    Pose b;
    b.x = goal->x - odom->x;
    b.y = goal->y - odom->y;

    Pose p_a;
    p_a.x = current.x - odom->x;
    p_a.y = current.y - odom->y;
    double p_axb = p_a.x * b.y - p_a.y * b.x;
    double dist = 100.0 * fabs(p_axb) / fabs(sqrt(b.x * b.x + b.y * b.y));
    return (int)dist;
  }
};
#endif
