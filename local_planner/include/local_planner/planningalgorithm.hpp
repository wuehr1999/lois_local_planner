#ifndef PLANNINGALGORITHM_HPP
#define PLANNINGALGORITHM_HPP

#include <string>

#include <planner/plannerdata.hpp>

/**
 @brief Planning Algorithm base class
 **/
class PlanningAlgorithm
{
public:

  /**
   @brief Planning results
   **/
  enum Outcomes
  {
    UNCTRITICAL = 0,
    CRITICAL = 1,
    RECOVERY = 2
  };

  /**
   @brief Initializes algorithm
   @param name Algorithm name
   @param id Algorithm identification index
   @param idRecovery Identification index of corresponding recovery algorithm
   @param dat Planner metadata
   **/
  PlanningAlgorithm(std::string name, int id, int idRecovery, PlannerData* dat);
  virtual ~PlanningAlgorithm(){};
  
  /**
   @brief Returns name
   @return name
   **/
  std::string getName(){ return name; }

  /**
   @brief Returns algorithm id
   @return id
   **/
  int getId(){ return id; }

  /**
   @brief Returns recovery algorithm id
   @return id
   **/
  int getIdRecovery(){ return idRecovery; }

  /**
   @brief Get linear velocity
   @return velocity in m/s
   **/
  double getV(){ return v; }

  /**
   @brief Get angular velocity
   @return velocity in rad/s
   **/
  double getW(){ return w; } 

  /**
   @brief Core planning step
   @return Planning result
   **/
  virtual PlanningAlgorithm::Outcomes plan() = 0;

  /**
   @brief Renders algorithm to global planner image
   **/
  virtual void visualize() = 0;

protected:
  std::string name;
  int id, idRecovery;
  PlannerData* dat;
  double v, w;
};
#endif
