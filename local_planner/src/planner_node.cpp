#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <msgs/filteroutput.h>
#include <msgs/loisgoal.h>

#include <planner/planner.hpp>
#include <planner/trajectory.hpp>
#include <planner/config.hpp>
#include <kinematics/models/model_differential.hpp>
#include <planner/decisionrules/decision_distance.hpp>
#include <tflistener/poseFromTf.hpp>

Planner* planner;
PlannerConfig config;

ros::Subscriber subGrid;
ros::Subscriber subOdom;
ros::Subscriber subGoal;
ros::Subscriber subGlobalGoal;
ros::Subscriber subLidar, subLidar2;
ros::Publisher pubVel;
ros::Subscriber subFilter;

void callbackGridIn(const nav_msgs::OccupancyGrid& msg)
{
//  ROS_INFO("New Grid");
  planner->setGrid(msg);
}

void callbackOdomIn(const nav_msgs::Odometry& msg)
{
//  ROS_INFO("New Odometry");
  planner->setOdometry(msg);
}

void callbackGoalIn(const move_base_msgs::MoveBaseActionGoal& msg)
{
//  ROS_INFO("New Goal");
  planner->setGoal(msg);
}

void callbackGlobalGoalIn(const msgs::loisgoal& msg)
{
  //ROS_INFO("New Goal");
  planner->setGlobalGoal(msg.distance_m.data, msg.heading_deg.data);
}

void callbackLidarIn(const sensor_msgs::LaserScan& msg)
{
//  ROS_INFO("New Lidar");
  planner->setLidar(msg);
}

void callbackLidarIn2(const sensor_msgs::LaserScan& msg)
{
//  ROS_INFO("New Lidar");
  planner->setLidar2(msg);
}

void callbackAvg(const msgs::filteroutput& msg)
{
//  ROS_INFO("New Avg");
  planner->setRoadmiddle(msg);
}

void poseCallback(double dx, double dy, double dz, double r, double p, double y)
{
  config.lidarOffset.x = dx;
  config.lidarOffset.y = dy;
  config.lidarOffset.yaw = 0.0;
}

void poseCallback2(double dx, double dy, double dz, double r, double p, double y)
{
  config.lidarOffset2.x = dx;
  config.lidarOffset2.y = dy;
  config.lidarOffset2.yaw = 0.0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grid_planner");
    ros::NodeHandle nh("~");

    config.trajectories = nh.param("trajectories", 7);
    config.overlay_grid = nh.param("overlay_grid", true);
    config.trajectoryLayers = nh.param("trajectoryLayers", 3);
    config.trajectoryLength_m = nh.param("trajectoryLength_m", 1.5);
    config.trajectoryCrit_deg = nh.param("trajectoryCrit_deg", 45.0);
    config.planningHorizon_secs = nh.param("planningHorizon_secs", 20.0);
    config.planningStep_secs = nh.param("planningStep_secs", 2.0);
    config.botWidth_m = nh.param("botWidth_m", 60.0); 
    config.botHeight_m = nh.param("botHeight_m", 70.0);
    config.gridWidth_m = nh.param("gridWidth_m", 5.0);
    config.gridHeight_m = nh.param("gridHeight_m", 5.0);
    config.vxMax = nh.param("vxMax", 3.0); 
    config.vxMin = nh.param("vxMin", 0.1);
    config.wMax = nh.param("wMax", 1.0);
    config.collision_th = nh.param("collision_th", 0.1);
    config.check_startfactor = nh.param("check_startfactor", 0.2);
    config.boxX_m = nh.param("boxX_m", 0.6);
    config.boxY_m = nh.param("boxY_m", 0.6);
    config.vxRecovery = nh.param("vRecover", 0.1);
    config.wRecovery = nh.param("wRecover", 0.3);
    config.dilation_size = nh.param("dilation_size", 2);
    config.opening_size = nh.param("opening_size", 2);
    config.debug = nh.param("debug", true);
    config.avg_switch = nh.param("avg_switch", 0.05);
    config.avg_hyst = nh.param("avg_hyst", 0.05);
    config.avg_p = nh.param("avg_p", 1.0);
    config.rate_ms = nh.param("rate_ms", 100.0);
    config.configMask = nh.param("config_mask", 39);
    config.aXMax = nh.param("aXMax", 3.0);
    config.aYMax = nh.param("aYMax", 3.0);
    config.aYawMax = nh.param("aYawMax", 3.0);
    config.angleOffset = nh.param("angleOffset", 0.0);
  
    PoseLookup tfStreeker(poseCallback, 
      nh.param("base", std::string("base_link")), nh.param("lidar_frame", std::string("lidar_link")));
    PoseLookup tfStreeker2(poseCallback2, 
      nh.param("base", std::string("base_link")), nh.param("lidar_frame2", std::string("lidar_link")));
    if(config.debug)
    {
      poseCallback(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      poseCallback2(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }
    else
    {
      tfStreeker.waitForIt();
      tfStreeker2.waitForIt();
    }

    Decision_Distance decision;
    Model_Differential model(nh.param("wheelwidth_m", 0.48));

    planner = new Planner(&config, &model, &decision); 

    subGrid = nh.subscribe(nh.param("grid_topic", std::string("/rtabmap/grid_map")), 1, callbackGridIn);
    subOdom = nh.subscribe(nh.param("odom_topic", std::string("/odom_slam")), 1, callbackOdomIn);
    subGoal = nh.subscribe(nh.param("goal_topic", std::string("move_base/goal")), 1, callbackGoalIn);
    subGlobalGoal = nh.subscribe(nh.param("globalgoal_topic", std::string("globalgoal")), 1, callbackGlobalGoalIn);
    subLidar = nh.subscribe(nh.param("lidar_topic", std::string("/sick_publisher/sick")), 1, callbackLidarIn);
    subLidar2 = nh.subscribe(nh.param("lidar_topic2", std::string("/sick_publisher/sick")), 1, callbackLidarIn2);
    subFilter = nh.subscribe(nh.param("avg_topic", std::string("/roat_detetor/road_filter")), 1, callbackAvg);

    pubVel = nh.advertise<geometry_msgs::Twist>(nh.param("/cmdvel_topic", std::string("/cmd_vel")), 1);
  
    image_transport::ImageTransport it(nh);
    image_transport::Publisher visPublisher = it.advertise("planner/out", 1);

    ros::Rate r(5);
    while(ros::ok())
    {
      if(planner->hasGoal())
      {
        pubVel.publish(planner->getCmdVel());
      }
    
      cv::Mat m = planner->visualize();
      std::vector<char> data(3 * m.rows * m.cols);
      sensor_msgs::Image out;
      out.height = m.rows;
      out.width = m.cols;
      out.encoding = "8UC3";
      out.is_bigendian = false;
      out.step = m.cols * 3;
      out.data.assign(m.data, m.data +  3 * m.rows * m.cols);
      visPublisher.publish(out);

      ros::spinOnce();
      r.sleep();
    }

    return 0;
}

