#include <planner/planningalgorithm.hpp>

PlanningAlgorithm::PlanningAlgorithm(std::string name, int id, int idRecovery, PlannerData* dat)
{
  this->name = name;
  this->id = id;
  this->idRecovery = idRecovery;
  this->dat = dat;
}
