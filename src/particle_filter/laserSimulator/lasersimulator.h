/*
 * File name: lasersimulator.h
 * Date:      10/11/2014
 * Author:    Miroslav Kulich
 */

#ifndef __LASER_SIMULATOR_H__
#define __LASER_SIMULATOR_H__

#include <vtkPNGReader.h>
#include <vtkSmartPointer.h>
#include <vector>

#include <typedefs.h>
#include "matrix_utils.h"

namespace imr {

/// ----------------------------------------------------------------------------
/// @brief
/// ----------------------------------------------------------------------------
struct SLaserConfig {
  int count;
  double maxRange;
  double resolution;
  double minAngle;
  double maxAngle;
};

struct SIntPoint {
  int x;
  int y;
  SIntPoint(int x, int y) : x(x), y(y) {};
};

class LaserSimulator {
public:
  LaserSimulator(vtkSmartPointer<vtkPNGReader> reader);
  ~LaserSimulator();
  bool onMap(unsigned int x, unsigned int y);
  LaserScan getScan(const RobotPosition &pose);
  PointList getRawPoints();
  int real2gridX(double x) { return floor(K*(x-x0)); };
  int real2gridY(double y) { return floor(K*(y-y0)); };
  double grid2realX(int x) { return x0 + x*cellSize; };
  double grid2realY(int y) { return y0 + y*cellSize; };
  bool isFeasible(RobotPosition pos);

private:
  enum CellState {FREESPACE_CELL, OCCUPIED_CELL, UNKNOWN_CELL};
  SLaserConfig laserConf;
  ByteMatrix *grid;
  int W;
  int H;
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

}

#endif

/* end of sndsimulator.h */
