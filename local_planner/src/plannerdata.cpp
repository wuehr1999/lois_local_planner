#include <planner/plannerdata.hpp>

void PlannerData::begin()
{
  if(!config->overlay_grid)
  {
    mu.lock();
    metersPerCell = 0.1;
    int width = config->gridWidth_m / metersPerCell;
    int height = config->gridHeight_m / metersPerCell;
  
    p0_m.x = -config->gridWidth_m * 0.5;
    p0_m.y = config->gridWidth_m * 0.5;
    p0_m.yaw = 0.0;

    int odomX = (int)(odom_m.x / metersPerCell  - p0_m.x / metersPerCell);
    int odomY = height - (int)(odom_m.y / metersPerCell  - p0_m.y / metersPerCell);
  
    origin_m.x = odom_m.x - config->gridWidth_m * 0.5;
    int offsetX = odomX - width / 2;
    origin_m.y = odom_m.y - config->gridHeight_m * 0.5;
    int offsetY = odomY -height / 2;
    printf("%i, %i\n", width, height); 
    inGrid = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
    gridInit = true;
    odom_m.x = 0.0;
    odom_m.y = 0.0;
    odom_m.yaw = 0.0;
    odomInit = true;
    mu.unlock();
  }
}

std::pair<int, int> PlannerData::metersToGrid(Pose p, bool rotate, bool fromOdom)
{
  std::pair<int, int> coords;
  int xx = 0;
  int yy = 0;
  if(gridInit)
  {
    Pose pivot;
    pivot.x = -origin_m.x;
    pivot.y = -origin_m.y;
    if(fromOdom)
    {
      pivot.x += odom_m.x;
      pivot.y += odom_m.y;
    }
    pivot.yaw = odom_m.yaw;
    
    double x_m = p.x;
    double y_m = p.y;
    if(rotate)
    {
      double s = sin(pivot.yaw - 0.5 * M_PI);
      double c = cos(pivot.yaw - 0.5 * M_PI);

      x_m = p.x * c - p.y * s;
      y_m = p.x * s + p.y * c;
    }

    xx = (int)((x_m + pivot.x) / metersPerCell);
    yy = grid.rows - (int)((y_m + pivot.x) / metersPerCell);
  }
  return std::make_pair(xx, yy);
}

bool PlannerData::collides(Pose p, double width, double height)
{
  std::pair<int, int> pose = metersToGrid(p);  
  int widthPx = (int)(width / metersPerCell);
  int heightPx = (int)(height / metersPerCell);
  double yaw = -odom_m.yaw + p.yaw - 0.5 * M_PI;

  cv::RotatedRect roi(cv::Point(pose.first, pose.second), cv::Size(widthPx, heightPx), yaw / M_PI * 180.0);
  posesVis.push_back(pose);
  bool collided = false; 
  float angle = roi.angle;
  cv::Size rectSize = roi.size;
  if(roi.angle < -45.)
  {
    roi.angle += 90.0;
    cv::swap(rectSize.width, rectSize.height);
  }
  cv::Mat m = getRotationMatrix2D(roi.center, angle, 1.0);
  cv::Mat rotated, collisions;
  cv::warpAffine(grid, rotated, m, grid.size(), cv::INTER_CUBIC);
  cv::getRectSubPix(rotated, rectSize, roi.center, collisions);
  int colCounter = 0;
  for(int rx = 0; rx < collisions.cols; rx++)
  {
    for(int ry = 0; ry < collisions.rows; ry++)
    {
      if(collisions.at<uchar>(ry, rx))
      {
        colCounter++;
      }
      if((float)colCounter > config->collision_th * (float)collisions.cols * (float)collisions.rows)
      {
        collided = true;
        break;
      }
    }
  }
  return collided;
}

void PlannerData::overlayLidar()
{
  mu.lock();
  grid = inGrid.clone();
  if(lidarInit && gridInit && odomInit)
  {
    double angle = lidar.angle_min;
    Pose current;
    std::pair<int, int> odom = metersToGrid(origin0_m);
    current.yaw = 0.0;
    current.x += odom_m.x;
    current.y += odom_m.y;

    std::pair<int, int> pos = metersToGrid(current, false, false);
    int size = lidar.ranges.size();
    for(int s = 0; s < size; s++)
    {
      int i = s;
      if(lidar.ranges[i] < lidar.range_max && lidar.ranges[i] > lidar.range_min)
      {
        double x = ((cos((size - i - 1) * lidar.angle_increment) * lidar.ranges[i]));
        double y = ((sin((size - i - 1) * lidar.angle_increment) * lidar.ranges[i]));
        double s = sin(-odom_m.yaw);
        double c = cos(-odom_m.yaw);
 
        double xx = 0;
        double yy = 0;
        if(config->overlay_grid)
        {
          xx = x * c - y * s;
          yy = x * s + y * c;
        }
        else
        {
          xx = x;
          yy = y;
        }
        xx = xx / metersPerCell;
        yy = yy / metersPerCell;
        angle += lidar.angle_increment;
        cv::circle(grid, cv::Point((int)(xx + 0.5) + odom.first, (int)(yy + 0.5) + odom.second), 1, cv::Scalar(255), cv::FILLED);
      }
    }
  }
  mu.unlock();
}

void PlannerData::overlayLidar2()
{
  mu.lock();
  if(lidarInit && lidarInit2 && gridInit && odomInit)
  {
    double angle = lidar.angle_min;
    Pose current;
    std::pair<int, int> odom = metersToGrid(origin0_m);
    current.yaw = 0.0;
    current.x += odom_m.x;
    current.y += odom_m.y;

    std::pair<int, int> pos = metersToGrid(current, false, false);
    int size = lidar2.ranges.size();
    for(int s = 0; s < size; s++)
    {
      int i = s;
      if(lidar2.ranges[i] < lidar2.range_max && lidar2.ranges[i] > lidar2.range_min)
      {
        double x = ((cos((size - i - 1) * lidar2.angle_increment) * lidar2.ranges[i]));
        double y = ((sin((size - i - 1) * lidar2.angle_increment) * lidar2.ranges[i]));
        double s = sin(-odom_m.yaw);
        double c = cos(-odom_m.yaw);

        double xx = 0;
        double yy = 0;
        if(config->overlay_grid)
        {
          xx = x * c - y * s;
          yy = x * s + y * c;
        }
        else
        {
          xx = x;
          yy = y;
        }
        xx = xx / metersPerCell;
        yy = yy / metersPerCell;
        angle += lidar.angle_increment;
        cv::circle(grid, cv::Point((int)(xx + 0.5) + odom.first, (int)(yy + 0.5) + odom.second), 1, cv::Scalar(255), cv::FILLED);
      }
    }
  }
  mu.unlock();
}

void PlannerData::setGrid(const nav_msgs::OccupancyGrid& msg)
{
  if(!odomInit || !config->overlay_grid)
  {
    return;
  }
  mu.lock();
  metersPerCell = msg.info.resolution;
  int width = config->gridWidth_m / metersPerCell;
  int height = config->gridHeight_m / metersPerCell;
  
  p0_m.x = (double)msg.info.origin.position.x;
  p0_m.y = (double)msg.info.origin.position.y;
  p0_m.yaw = 0.0;

  int odomX = (int)(odom_m.x / metersPerCell  - p0_m.x / metersPerCell);
  int odomY = msg.info.height - (int)(odom_m.y / metersPerCell  - p0_m.y / metersPerCell);
  
  origin_m.x = odom_m.x - config->gridWidth_m * 0.5;
  int offsetX = odomX - width / 2;
  origin_m.y = odom_m.y - config->gridHeight_m * 0.5;
  int offsetY = odomY -height / 2;
  
  inGrid = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
  for(int y = offsetY; y < offsetY + height; y++)
  {
    for(int x = offsetX; x < offsetX + width; x++)
    {
      int data = -1;
      if(y < msg.info.height && x < msg.info.width && y > 0 && x > 0)
      {
        data = msg.data[x + (msg.info.height - y) * msg.info.width];
      }
      if(-1 == data)
      {
        data = 0;
      }
      else if(data)
      {
        data = 255;
      }
      inGrid.at<uchar>(y - offsetY, x - offsetX) = data;
    }
  }
  cv::morphologyEx(inGrid, inGrid, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(config->opening_size, config->opening_size)));
  cv::morphologyEx(inGrid, inGrid, cv::MORPH_DILATE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(config->dilation_size, config->dilation_size)));
  gridInit = true;
  mu.unlock();
}


void PlannerData::setOdometry(const nav_msgs::Odometry& msg)
{
  mu.lock();
  if(!config->overlay_grid)
  {
    odom_m.x = 0.0;
    odom_m.y = 0.0;
    odom_m.yaw = 0.0;
  }
  else
  {
    odom_m.x = (double)msg.pose.pose.position.x;
    odom_m.y = (double)msg.pose.pose.position.y;
    double x = msg.pose.pose.orientation.x;
    double y = msg.pose.pose.orientation.y;
    double z = msg.pose.pose.orientation.z;
    double w = msg.pose.pose.orientation.w;
    odom_m.yaw = atan2(2.0*(w * z + w * y), w * w + x * x - y * y - z * z) + 0.5 * M_PI;
  }
  vX = msg.twist.twist.linear.x;
  w = msg.twist.twist.angular.z;
  odomInit = true;
  mu.unlock();
}

void PlannerData::setGoal(const move_base_msgs::MoveBaseActionGoal& msg)
{
  mu.lock();
  goal_m.x = msg.goal.target_pose.pose.position.x;
  goal_m.y = msg.goal.target_pose.pose.position.y;
  goalSet = true;
  mu.unlock();
}

void PlannerData::setGlobalGoal(double dist, double heading)
{
  mu.lock();
  dist = 100.0;
  goal_m.x = odom_m.x + dist * sin(- M_PI - odom_m.yaw + (heading / 180.0 * M_PI));
  goal_m.y = odom_m.y + dist * cos(- M_PI - odom_m.yaw + (heading / 180.0 * M_PI));
  goalSet = true;
  mu.unlock();
}

void PlannerData::setLidar(const sensor_msgs::LaserScan& msg)
{
  mu.lock();
  lidar = msg;
  lidarInit = true;
  mu.unlock();
}

void PlannerData::setLidar2(const sensor_msgs::LaserScan& msg)
{
  mu.lock();
  lidar2 = msg;
  lidarInit2 = true;
  mu.unlock();
}

void PlannerData::drawRect(Pose p, double width, double height, cv::Scalar col)
{
  std::pair<int, int> pose = metersToGrid(p);
  double yaw = -odom_m.yaw + p.yaw - 0.5 * M_PI;
  cv::RotatedRect rotatedRectangle(cv::Point(pose.first, pose.second), cv::Size(width / metersPerCell, height / metersPerCell), yaw / M_PI * 180.0);
  cv::Point2f vertices2f[4];
  rotatedRectangle.points(vertices2f); 
  cv::Point vertices[4];
  for(int i = 0; i < 4; i++)
  {
    vertices[i] = vertices2f[i];
  }

  cv::fillConvexPoly(debugGrid, vertices, 4, col);
}

void PlannerData::visualize()
{
 if(gridInit && odomInit)
 {
    mu.lock();
    debugGrid = cv::Mat(grid.rows, grid.cols, CV_8UC1, cv::Scalar(0));
    cv::cvtColor(grid, debugGrid, cv::COLOR_GRAY2BGR);
    std::pair<int, int> odom = metersToGrid(origin0_m);
    if(goalSet)
    {
      std::pair<int, int> g = metersToGrid(goal_m, false, false);
      cv::arrowedLine(debugGrid, cv::Point(odom.first, odom.second), cv::Point(g.first, g.second), cv::Scalar(255, 0, 0));  
    }

    int x2 = odom.first + 20 * sin(odom_m.yaw);
    int y2 = odom.second + 20 * cos(odom_m.yaw);
    cv::arrowedLine(debugGrid, cv::Point(odom.first, odom.second), cv::Point(x2, y2), cv::Scalar(0, 0, 255));
    mu.unlock();
    
  }
}
  
void PlannerData::setV(double v)
{
  mu.lock();
  currentV = v;
  mu.unlock();
}

void PlannerData::setW(double w)
{
  mu.lock();
  currentW = w;
  mu.unlock();
}

void PlannerData::setRoadmiddle(const msgs::filteroutput& msg)
{
  mu.lock();
  avg = msg.offset.data;
  noway = msg.noway.data;
  nearleft = msg.nearleft.data;
  nearright = msg.nearright.data;
  mu.unlock();
}

geometry_msgs::Twist PlannerData::getCmdVel()
{
  mu.lock();
  geometry_msgs::Twist cmdVel;
  cmdVel.linear.x = currentV;
  cmdVel.linear.y = 0;
  cmdVel.linear.z = 0;
  cmdVel.angular.x = 0;
  cmdVel.angular.y = 0;
  cmdVel.angular.z = currentW;
  mu.unlock();
  return cmdVel;
}
