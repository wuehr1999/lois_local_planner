#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <kinematics/model.hpp>
#include <kinematics/pose.hpp>
#include <vector>

/**
 @brief Representation of single trajectory
 **/
class Trajectory
{
public:

  std::vector<Pose> poses;

  /**
   @brief Initializes Trajectory and does roll out.
   @param model Robot motion model
   @param v0 Trajecory linear velocity in m/s
   @param w Trajectory angular velocity in rad/s
   @param tStep Planning step in seconds
   @param tMax Planning horizon in seconds
   **/  
  Trajectory(Model* model, double v0, double w, double tStep, double tMax);
  ~Trajectory() {};

  /**
   @brief Check if trajectory collides with obstacle in grid
   @return collision
   **/
  bool isCollided(){ return collided; }

  /**
   @brief Mark trajectory as collided
   **/
  bool setCollided(){ collided = true; }

  /**
   @brief Mark trajectory as collided
   @param colTime planning step of collision in seconds
   **/
  bool setCollided(double colTime){ collided = true; this->colTime = colTime; }

  /**
   @brief Get planning step of collision
   @param time in seconds
   **/
  double getCollisionTime(){ return colTime; }

  /**
   @brief Check if trajectory is impossible to drive because of robots velocity and acceleration constarints.
   @return imposible
   **/
  bool isImpossible(){ return impossible; }

  /**
   @brief Set trajectory as impossible to drive because of robots velocity and acceleration constarints.
   **/
  void setImpossible(){ impossible = true; }
  
  /**
   @brief get linear velocity
   @return velocity in m/s
   **/
  double getv0(){ return v0; }

  /**
   @brief get angular velocity
   @return velocity in rad/s
   **/
  double getW(){ return w; }

  /**
   @brief Reset to initial state
   **/
  void reset();

  /**
   @brief roll out/ plan
   **/
  void rollout();

  /**
   @brief roll out/ plan
   @param v0 Trajecory linear velocity in m/s
   @param w Trajectory angular velocity in rad/s
   @param tStep Planning step in seconds
   @param tMax Planning horizon in seconds  
   **/
  void rollout(double v0, double w, double tStep, double tMax);

private:
  Model* model;
  double v0, w;
  double tStep, tMax;
  double colTime;
  bool collided;
  bool impossible;
};
#endif
