#include <local_planner/trajectory.hpp>

#include <limits>
  
Trajectory::Trajectory(Model* model, double v0, double w, double tStep, double tMax)
{
  this->model = model;
  rollout(v0, w, tStep, tMax);
  reset();
}

void Trajectory::reset()
{
  collided = false;
  impossible = false;
  colTime = std::numeric_limits<double>::max();
}

void Trajectory::rollout()
{
  poses.clear();
//  printf("%f, %f\n", v0, w);
  for(double t = 0.0; t < tMax; t += tStep)
  {
    poses.push_back(model->calculate(v0, w, t));
//    printf("%s\n", poses[poses.size() - 1].str().c_str());
  }
}

void Trajectory::rollout(double v0, double w, double tStep, double tMax)
{
  this->v0 = v0;
  this->w = w;
  this->tStep = tStep;
  this->tMax = tMax;
  rollout();
}
