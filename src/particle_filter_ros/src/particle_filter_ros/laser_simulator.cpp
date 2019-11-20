/*
 * File name: lasersimulator.cpp
 * Date:      Sun Nov 22 15:43:00 +0200 2015
 * Author:    Miroslav Kulich
 */

/* #include <unistd.h> */
/* #include <iostream> */
/* #include <sstream> */
#include <cmath>  // For M_PI.

#include <tf2/LinearMath/Quaternion.h>  // For tf2::Quaternion.
#include <tf2/LinearMath/Matrix3x3.h>  // For tf2::Matrix3x3.
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // For tf2::fromMsg.


#include <particle_filter_ros/typedefs.h>

#include <particle_filter_ros/laser_simulator.h>

namespace particle_filter_ros
{

inline
double yawFromPose(const geometry_msgs::Pose & pose)
{
  tf2::Quaternion q;
  tf2::fromMsg(pose.orientation, q);
  tf2::Matrix3x3 t{q};
  double roll;
  double pitch;
  double yaw;
  t.getRPY(roll, pitch, yaw);
  return yaw;
}

LaserSimulator::LaserSimulator(const nav_msgs::OccupancyGrid & map)
{
  if (std::abs(yawFromPose(map.info.origin)) > 1e-10)
  {
    ROS_ERROR("Map with non-zero orientation not supported yet");
    return;
  }

  laserConf.count = 37;
  laserConf.maxRange = 30.0;
  laserConf.resolution = M_PI / static_cast<double>(laserConf.count - 1);
  laserConf.minAngle = -M_PI / 2;
  laserConf.maxAngle = M_PI / 2;

  W = map.info.width;
  H = map.info.height;

  grid_ptr_.reset(new ByteMatrix(H, W));
  for (size_t r = 0; r < H; r++)
  {
    for (size_t c = 0; c < W; c++)
    {
      uint8_t val = map.data[r * W + c];
      (*grid_ptr_)(r, c) = (val==0 ? OCCUPIED_CELL : (val==255 ? FREESPACE_CELL : UNKNOWN_CELL));
    }
  }

  x0 = map.info.origin.position.x;
  y0 = map.info.origin.position.y;
  cellSize = map.info.resolution;
  K = 1.0 / cellSize;
}

/// - public --------------------------------------------------------------
bool LaserSimulator::onMap(int x, int y)
{
  return ((x >= 0) and (y >= 0) and (x < static_cast<int>(W)) and (y < static_cast<int>(H)));
}


/// - public --------------------------------------------------------------
LaserScan LaserSimulator::getScan(const RobotPosition &pose)
{
  LaserScan result;
  numSamples = laserConf.count;

  points.resize(numSamples);
  result.resize(numSamples);
  scan.resize(numSamples);
  //   DEBUG("laser: count " << lconf.count);
  //   DEBUG("laser: maxRange " << lconf.maxRange);
  //   DEBUG("laser: resolution " << lconf.resolution);
  //   DEBUG("laser: minAngle " << lconf.minAngle);
  //   DEBUG("laser: maxAngle " << lconf.maxAngle);
  //   DEBUG("laser: numSamples " << numSamples);

  robot_x = pose.x;
  robot_y = pose.y;
  int rx = real2gridX(robot_x);
  int ry = real2gridY(robot_y);
  double fx, fy;
  double xx, yy;
  double angle = pose.phi + laserConf.minAngle;
  for(int i=0;i<numSamples;i++) {
    fx = pose.x + laserConf.maxRange*cos(angle);
    fy = pose.y + laserConf.maxRange*sin(angle);
    SIntPoint pt = bresenham(rx,ry,real2gridX(fx),real2gridY(fy));
    xx = rx - pt.x;
    yy = ry - pt.y;
    result[i] = cellSize * sqrt(xx*xx+yy*yy);
    points[i].x = grid2realX(pt.x);
    points[i].y = grid2realY(pt.y);

    angle += laserConf.resolution;
  }

  return result;
}



PointList LaserSimulator::getRawPoints()
{
  return points;
}

bool LaserSimulator::isFeasible(const RobotPosition& pos)
{
  const int x = real2gridX(pos.x);
  const int y = real2gridY(pos.y);
  if (!onMap(x, y))
  {
    return false;
  }

  return (*grid_ptr_)(real2gridY(pos.y), real2gridX(pos.x)) == FREESPACE_CELL;
}


void SWAP(int &x, int &y)
{
  int p;
  p = x;
  x = y;
  y = p;
}


SIntPoint LaserSimulator::bresenham(int x0, int y0, int x1, int y1)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steep = (abs(dy) >= abs(dx));
  if (steep)
  {
    SWAP(x0, y0);
    SWAP(x1, y1);
    // recompute Dx, Dy after swap
    dx = x1 - x0;
    dy = y1 - y0;
  }
  int xstep = 1;
  if (dx < 0)
  {
    xstep = -1;
    dx = -dx;
  }
  int ystep = 1;
  if (dy < 0)
  {
    ystep = -1;
    dy = -dy;
  }
  int twoDy = 2 * dy;
  int twoDyTwoDx = twoDy - 2 * dx; // 2*Dy - 2*Dx
  int e = twoDy - dx; //2*Dy - Dx
  int y = y0;
  int xDraw, yDraw;
  for (int x = x0; x != x1; x += xstep)
  {
    if (steep)
    {
      xDraw = y;
      yDraw = x;
    }
    else
    {
      xDraw = x;
      yDraw = y;
    }
    if (xDraw <= 0 || xDraw >= grid_ptr_->NCOLS
        || yDraw <= 0 || yDraw >= grid_ptr_->NROWS
        || (*grid_ptr_)(yDraw, xDraw) == OCCUPIED_CELL)
    {
      return SIntPoint(xDraw,yDraw);
    }

    // next
    if (e > 0)
    {
      e += twoDyTwoDx; //E += 2*Dy - 2*Dx;
      y = y + ystep;
    }
    else
    {
      e += twoDy; //E += 2*Dy;
    }
  }
  return SIntPoint(x1,y1);
}

} /* namespace particle_filter_ros */
