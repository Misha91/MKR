/*
 * Date:      2018-12-12
 * Author:    Miroslav Kulich, Gaël Écorchard
 */

#pragma once

#include <cmath>  //For std::{abs,floor}.
#include <cstdint>  // For uint8_t.
#include <memory>  // For std::unique_ptr.
#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <particle_filter_ros/matrix_utils.h>
#include <particle_filter_ros/typedefs.h>

namespace particle_filter_ros
{

struct SLaserConfig
{
  int count;
  double maxRange;
  double resolution;
  double minAngle;
  double maxAngle;
};

struct SIntPoint
{
  int x;
  int y;
  SIntPoint(int x, int y) : x(x), y(y) {};
};

class LaserSimulator
{
  public:

    LaserSimulator(const nav_msgs::OccupancyGrid & map);
    bool onMap(int x, int y);
    LaserScan getScan(const RobotPosition& pose);
    PointList getRawPoints();
    int real2gridX(double x) {return std::floor(K*(x-x0));};
    int real2gridY(double y) {return std::floor(K*(y-y0));};
    double grid2realX(int x) {return x0 + x*cellSize;};
    double grid2realY(int y) {return y0 + y*cellSize;};
    bool isFeasible(const RobotPosition& pos);

  private:

    enum CellState {FREESPACE_CELL, OCCUPIED_CELL, UNKNOWN_CELL};

    SLaserConfig laserConf;
    std::unique_ptr<ByteMatrix> grid_ptr_;
    size_t W;
    size_t H;
    double maxRange;
    double cellSize;
    double robot_x;
    double robot_y;
    double K, x0, y0;
    std::vector<double> scan;
    PointList points;
    int numSamples;
    SIntPoint bresenham(int x0, int y0, int x1, int y1);
};

} /* namespace particle_filter_ros */
