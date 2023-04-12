#include <planner/planner.hpp>
  
Planner::Planner(PlannerConfig* config, Model* model, DecisionRule* decision)
{
  plannerData.config = config;
  plannerData.model = model;
  this->decision = decision;

  plannerData.odomInit = false;
  plannerData.gridInit = false;
  plannerData.goalSet = false;
  plannerData.lidarInit = false;

  plannerData.origin0_m.x = 0.0;
  plannerData.origin0_m.y = 0.0;
  plannerData.origin0_m.yaw = 0.0;

  plannerData.boxLeft_m.x = 0.5 * plannerData.config->boxX_m;
  plannerData.boxLeft_m.y = 0.5 * plannerData.config->boxY_m;
  plannerData.boxLeft_m.yaw = 0.0;
  plannerData.boxRight_m.x = 0.5 * plannerData.config->boxX_m;
  plannerData.boxRight_m.y = -0.5 * plannerData.config->boxY_m;
  plannerData.boxRight_m.yaw = 0.0;

  plannerData.avg = 0.0;

  recoveryBack = new Algo_Recoveryback("backrecovery", 0, -1, &plannerData);
  addRecovery(recoveryBack);
  
  ripperPlanner = new Algo_Ripper("ripper", 0, 0, &plannerData);
//  trajectoryPlanner = new Algo_Trajectoryrollout("trajectory", 1, 0, &plannerData, decision, plannerData.config->vxMax);
  dwaPlanner = new Algo_DWA("trajectory", 1, 0, &plannerData, decision);
  middlePlanner = new Algo_Roadmiddle("middle", 2, 0, &plannerData);
  
  addAlgoDecode(ripperPlanner, CONFIG_ENABLE_BUMPER, CONFIG_DEFAULT_BUMPER);
//  addAlgo(middlePlanner, true);
//  addAlgo(trajectoryPlanner, false);
  if(!(plannerData.config->configMask >> CONFIG_SWITCH_DWA_MIDDLE) & 1)
  {
    addAlgoDecode(dwaPlanner, CONFIG_ENABLE_DWA, CONFIG_DEFAULT_DWA);
    addAlgoDecode(middlePlanner, CONFIG_ENABLE_ROADMIDDLE, CONFIG_DEFAULT_ROADMIDDLE);
  }
  else
  {
    addAlgoDecode(middlePlanner, CONFIG_ENABLE_ROADMIDDLE, CONFIG_DEFAULT_ROADMIDDLE);
    addAlgoDecode(dwaPlanner, CONFIG_ENABLE_DWA, CONFIG_DEFAULT_DWA);
  }
  plannerData.begin();

  threadStop = false;
  thread = std::thread(&Planner::worker, this);

  if(config->debug)
  {
    ROS_INFO("%s", plannerData.config->str().c_str());
  }
}

Planner::~Planner()
{
  threadStop = true;
  thread.join();
}

void Planner::addAlgo(PlanningAlgorithm* algo, bool useDefault)
{
  algos.push_back(algo);
  results.push_back(PlanningAlgorithm::Outcomes::UNCTRITICAL);
  ROS_INFO("Set up %s algorithm as default: %d", algo->getName().c_str(), useDefault);
  if(useDefault)
  {
    defaultId = algo->getId();
  }
}

void Planner::addAlgoDecode(PlanningAlgorithm* algo, int enableMask, int defaultMask)
{
  if((plannerData.config->configMask >> enableMask) & 1)
  {
    addAlgo(algo, (plannerData.config->configMask >> defaultMask) & 1);
  }
}

void Planner::addRecovery(PlanningAlgorithm* recovery)
{
  recoveries.push_back(recovery);
}

void Planner::worker()
{
  thread.detach();
  while(!threadStop)
  {
    plan();
    usleep(plannerData.config->rate_ms * 1000.0);
  }
}

void Planner::setRoadmiddle(const msgs::filteroutput& msg)
{
  plannerData.setRoadmiddle(msg);
}
void Planner::setGrid(const nav_msgs::OccupancyGrid& msg)
{
  plannerData.setGrid(msg);
}

void Planner::setOdometry(const nav_msgs::Odometry& msg)
{
  plannerData.setOdometry(msg);
}

cv::Mat Planner::visualize()
{
  plannerData.visualize();
  plannerData.mu.lock();
  for(int i = 0; i < algos.size(); i++)
  {
    algos.at(i)->visualize();
  }
  for(int i = 0; i < recoveries.size(); i++)
  {
    recoveries.at(i)->visualize();
  }
  cv::Mat m = plannerData.debugGrid.clone();
  plannerData.mu.unlock();
  return m;
}

void Planner::setGoal(const move_base_msgs::MoveBaseActionGoal& msg)
{
  plannerData.setGoal(msg);
}

void Planner::setGlobalGoal(double dist, double heading)
{
  plannerData.setGlobalGoal(dist, heading);
}

void Planner::setLidar(const sensor_msgs::LaserScan& msg)
{
  plannerData.setLidar(msg);
}

void Planner::setLidar2(const sensor_msgs::LaserScan& msg)
{
  plannerData.setLidar2(msg);
}

void Planner::plan()
{
  static Timer t; 
  plannerData.overlayLidar();
  plannerData.overlayLidar2();
  if(!plannerData.goalSet)
  {
    plannerData.currentV = 0.0; 
    plannerData.currentW = 0.0;
    return;
  }
  plannerData.mu.lock();
  plannerData.posesVis.clear();

  for(int i = 0; i < algos.size(); i++)
  {
    results.at(i) = algos.at(i)->plan();
    if(plannerData.config->debug)
    {
      ROS_INFO("%s, %i", algos.at(i)->getName().c_str(), (int)results.at(i));
    }
  }
  plannerData.mu.unlock();
 
  bool normal = true;
  bool critical = false;

  for(int i = 0; i < results.size(); i++)
  {
    if(PlanningAlgorithm::Outcomes::CRITICAL == results.at(i) && !critical)
    {
      normal = false;
      critical = true;
      plannerData.mu.lock();
      ROS_WARN("%s\n", algos.at(i)->getName().c_str());
      plannerData.currentV = algos.at(i)->getV();
      plannerData.currentW = algos.at(i)->getW();
      plannerData.mu.unlock();
    }
    else if(PlanningAlgorithm::Outcomes::RECOVERY == results.at(i))
    {
      normal = false;
      recoveries.at(algos.at(i)->getIdRecovery())->plan();
    }
  }
  if(normal)
  {
    plannerData.mu.lock();
    ROS_ERROR("%s\n", algos.at(defaultId)->getName().c_str());
    plannerData.currentV = algos.at(defaultId)->getV();
    plannerData.currentW = algos.at(defaultId)->getW();
    plannerData.mu.unlock();
  }
}

geometry_msgs::Twist Planner::getCmdVel()
{
  return plannerData.getCmdVel();
}
