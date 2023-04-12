#ifndef ALGO_RIPPER
#define ALGO_RIPPER

#include <planner/planningalgorithm.hpp>

/**
 @brief Implementation of Algorithm using two bounding boxes on the left and right side of the robot to push it away from a very near obstacle.
 **/
class Algo_Ripper : public PlanningAlgorithm
{
public:
  Algo_Ripper(std::string name, int id, int idRecovery, PlannerData* dat):PlanningAlgorithm(name, id, idRecovery, dat)
  {
    left = false;
    right = false;
  }

  PlanningAlgorithm::Outcomes plan()
  {
    left = dat->collides(dat->boxLeft_m, dat->config->boxX_m, dat->config->boxY_m);
    right = dat->collides(dat->boxRight_m, dat->config->boxX_m, dat->config->boxY_m);

    if(left && right)
    {
      return PlanningAlgorithm::Outcomes::RECOVERY;
    }
    else if(left)
    {
      v = dat->config->vxRecovery;
      w = dat->config->wRecovery;
      return PlanningAlgorithm::Outcomes::CRITICAL;
    }
    else if(right)
    {
      v = dat->config->vxRecovery;
      w = -dat->config->wRecovery;
      return PlanningAlgorithm::Outcomes::CRITICAL;
    }
    else
    {
      w = 0.0;
      v = dat->config->vxMax;
      return PlanningAlgorithm::Outcomes::UNCTRITICAL;
    }
  }

  void visualize()
  {
    if(left || right)
    {
      if(left)
      {
        dat->drawRect(dat->boxLeft_m, dat->config->boxX_m, dat->config->boxY_m, cv::Scalar(0, 0, 255));
      }
      else 
      {
        dat->drawRect(dat->boxLeft_m, dat->config->boxX_m, dat->config->boxY_m, cv::Scalar(0, 255, 0));
      }
      if(right)
      {
        dat->drawRect(dat->boxRight_m, dat->config->boxX_m, dat->config->boxY_m, cv::Scalar(0, 0, 255));
      }
      else 
      {
        dat->drawRect(dat->boxRight_m, dat->config->boxX_m, dat->config->boxY_m, cv::Scalar(0, 255, 0));
      }
    }
  }

private:
  bool left, right;
};
#endif
