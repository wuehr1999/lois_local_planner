#ifndef ALGO_RECOVERYBACK
#define ALGO_RECOVERYBACK

#include <unistd.h>
#include <planner/planningalgorithm.hpp>

/**
 @brief Implementation of blocking recovery algorithm driving the robot backwards slowly.
 **/
class Algo_Recoveryback : public PlanningAlgorithm
{
public:
  Algo_Recoveryback(std::string name, int id, int idRecovery, PlannerData* dat):PlanningAlgorithm(name, id, idRecovery, dat){}

  PlanningAlgorithm::Outcomes plan()
  {
    dat->setW(0.0);
    dat->setV(dat->config->vxMax * 0.25);
    usleep(100000);
    dat->setV(0.0);
    usleep(100000);
    dat->setV(-dat->config->vxRecovery);
    usleep(3000000);
    dat->setV(0.0);

    return PlanningAlgorithm::Outcomes::UNCTRITICAL;
  }

  void visualize(){}
};
#endif
