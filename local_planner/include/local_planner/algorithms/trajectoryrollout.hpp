#ifndef ALGO_TRAJECTORYROLLOUT_HPP
#define ALGO_TRAJECTORYROLLOUT_HPP

#include <local_planner/decisionrule.hpp>
#include <local_planner/planningalgorithm.hpp>

/**
 @brief Implementation of trajectory planner with constrainted yaw acceleration and y-Axis acceleration.
 **/
class Algo_Trajectoryrollout : public PlanningAlgorithm
{
public:
  
  Algo_Trajectoryrollout(std::string name, int id, int idRecovery, PlannerData* dat, DecisionRule* decision, double v0 = 0.3):PlanningAlgorithm(name, id, idRecovery, dat) {
    chosenTrajectory = 0;
    this->v = v0;
    this->vPlanner = v0;
    this->decision = decision;
    initTrajectories();
  }

  PlanningAlgorithm::Outcomes plan()
  {
    int weight = std::numeric_limits<int>::max();
    int weightMin = std::numeric_limits<int>::max();
    static double minCollided = std::numeric_limits<double>::max();
    static double lastV = 0.0;
    int collided = 0;
  
    Pose currentOdom;
    for(int t = 0; t < trajectoriesVector.size(); t++)
    {
      Trajectory* current = &trajectoriesVector.at(t);
      current->reset();
      double dw = fabs(current->getW() - dat->w) / dat->config->planningStep_secs;
      double dy = fabs(currentOdom.dY(current->poses[1]) / dat->config->planningStep_secs); 
      if(dw > dat->config->aYawMax || dy > dat->config->aYMax)
      {
        current->setImpossible();
      }
      if(!current->isImpossible())
      {
        for(int p = 0; p < current->poses.size(); p++)
        {
          if(dat->collides(current->poses[p], dat->config->botWidth_m, dat->config->botHeight_m) && (double)p > (double)current->poses.size() * dat->config->check_startfactor)
          {
            current->setCollided(p * dat->config->planningStep_secs);
            if(current->getCollisionTime() < minCollided)
            {
              minCollided = current->getCollisionTime();
            }
            break;
          }
        }
        int currentWeight = decision->decide(current, &dat->goal_m, &dat->odom_m);
        if(currentWeight < weightMin)
        {
          weightMin = currentWeight;
        }
        if(!current->isCollided())
        {
//          int currentWeight = decision->decide(current, &dat->goal_m, &dat->odom_m);
          if(currentWeight < weight)
          {
            weight = currentWeight;
            chosenTrajectory = t;
          }
        }
        else
        {
          collided++;
        }
      }
    }
    v = trajectoriesVector.at(chosenTrajectory).getv0();
    w = -trajectoriesVector.at(chosenTrajectory).getW();
    double angleDest = -90.0 + dat->odom_m.yaw / M_PI * 180.0 + dat->odom_m.angle(dat->goal_m) / M_PI * 180.0;
//    printf("%f, %f\n", dat->goal_m.x, dat->goal_m.y);
//    printf("%i, %i, %f, %f, %f\n", weight, weightMin, dat->odom_m.yaw / M_PI * 180.0, angleDest, dat->config->trajectoryCrit_deg);
    if(/*0 == collided*/ weight == weightMin && fabs(angleDest) < dat->config->trajectoryCrit_deg)
    {
      return PlanningAlgorithm::Outcomes::UNCTRITICAL;
    }
    else if(trajectoriesVector.size() == collided)
    {
      return PlanningAlgorithm::Outcomes::RECOVERY;
    }
    else
    {
      return PlanningAlgorithm::Outcomes::CRITICAL;
    }
  }

  void visualize()
  {
    for(int t = 0; t < trajectoriesVector.size(); t++)
    {
      Trajectory* current = &trajectoriesVector.at(t);
      cv::Scalar col;
      if(current->isCollided())
      {
        col = cv::Scalar(0, 0, 255);
      }
      else if(current->isImpossible())
      {
        col = cv::Scalar(255, 102, 255);
      }
      else if(chosenTrajectory == t)
      {
        col = cv::Scalar(66, 239, 245);
      }
      else
      {
        col = cv::Scalar(0, 255, 0);
      }
      std::pair<int, int> lastPose = dat->metersToGrid(dat->origin0_m);
      for(int p = 0; p < current->poses.size(); p++)
      {
        std::pair<int, int> pose = dat->metersToGrid(current->poses[p]);  
        cv::line(dat->debugGrid, cv::Point(lastPose.first, lastPose.second), cv::Point(pose.first, pose.second), col);
        lastPose = pose;
      }
    }
  }

private:

  DecisionRule* decision;
  std::vector<Trajectory> trajectoriesVector;
  int chosenTrajectory;
  double vPlanner;

  void initTrajectories()
  {
    double w_increment = 2 * dat->config->wMax / (dat->config->trajectories - 1);
    for(int t = 0; t < dat->config->trajectories; t++)
    {
      trajectoriesVector.push_back(Trajectory(dat->model, vPlanner, -dat->config->wMax + t * w_increment, dat->config->planningStep_secs, dat->config->planningHorizon_secs));
    }
  }
};
#endif
