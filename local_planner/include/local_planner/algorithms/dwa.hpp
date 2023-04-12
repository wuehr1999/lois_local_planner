#ifndef ALGO_DWA_HPP
#define ALGO_DWA_HPP

#include <planner/decisionrule.hpp>
#include <planner/planningalgorithm.hpp>
#include <planner/algorithms/trajectoryrollout.hpp>

/**
 @brief Implementation of dynamic window approach.
 **/
class Algo_DWA : public PlanningAlgorithm
{
public:


  /**
   @brief Initializes DWA algorithm
   @param name Algorithm name
   @param id Algorithm identification index
   @param idRecovery Identification index of corresponding recovery algorithm
   @param dat Planner metadata
   @param decision Decision rule for selecting trajectories
   **/
  Algo_DWA(std::string name, int id, int idRecovery, PlannerData* dat, DecisionRule* decision):PlanningAlgorithm(name, id, idRecovery, dat) {
    vStep = (dat->config->vxMax - dat->config->vxMin) / dat->config->trajectoryLayers;
    for(int i = 0; i < dat->config->trajectoryLayers; i++)
    {
      rollouts.push_back(Algo_Trajectoryrollout(std::to_string(i), i, idRecovery, dat, decision, dat->config->vxMin + i * vStep));
    }
  }

  PlanningAlgorithm::Outcomes plan()
  {
    PlanningAlgorithm::Outcomes outcome = PlanningAlgorithm::Outcomes::RECOVERY;
    double dv = dat->config->aXMax * dat->config->rate_ms * 0.001;
    chosenRollout = -1;
    for(int i = 0; i < rollouts.size(); i++)
    {
      if(fabs(rollouts.at(i).getV() - dat->vX) <= dv)
      {
        outcome = rollouts.at(i).plan();
        chosenRollout = i;
        if(PlanningAlgorithm::Outcomes::UNCTRITICAL != outcome)
//        if(PlanningAlgorithm::Outcomes::RECOVERY == outcome)
        {
          break;
        }
//        chosenRollout = i;
      }
    }
    v = rollouts.at(chosenRollout).getV();
    w = rollouts.at(chosenRollout).getW();
    return outcome;
  }
  
  void visualize()
  {
    if(chosenRollout >= 0 && chosenRollout < rollouts.size())
    {
      rollouts.at(chosenRollout).visualize();
    }
  }

private:
  std::vector<Algo_Trajectoryrollout> rollouts;
  double vStep;
  int chosenRollout;
};
#endif
