/*
 * File name: lasersimulator.cpp
 * Date:      Sun Nov 22 15:43:00 +0200 2015
 * Author:    Miroslav Kulich
 */

#include <unistd.h>
#include <iostream>
#include <sstream>
#include <vtkImageData.h>
#include <math.h>

#include <typedefs.h>
#include "lasersimulator.h"

using namespace imr;

/// Callbacks ------------------------------------------------------




/// - constructor --------------------------------------------------------------
LaserSimulator::LaserSimulator(vtkSmartPointer<vtkPNGReader> reader)
{
  laserConf.count = 36;
  laserConf.maxRange = 30.0;
  laserConf.resolution = M_PI/double(laserConf.count-1);
  laserConf.minAngle = -M_PI/2;
  laserConf.maxAngle = M_PI/2;

  vtkSmartPointer<vtkImageData> imageData;
  imageData = reader->GetOutput();
  reader->Update();
  int dims[3];
  imageData->GetDimensions (dims);
//   std::cout << "DIMS: " << dims[0] << " " << dims[1] << " " << dims[2] << std::endl;
  grid = new ByteMatrix(dims[1], dims[0]);
  for(size_t r=0;r<dims[1];r++) {
    for(size_t c=0;c<dims[0];c++) {
      unsigned char val = *static_cast<unsigned char*>(imageData->GetScalarPointer(c, r, 0));
      (*grid)(r,c) = (val==0 ? OCCUPIED_CELL : (val==255 ? FREESPACE_CELL : UNKNOWN_CELL));
      //std::cout << (int) val << " ";
    }
  }
  W = dims[0];
  H = dims[1];


  K = 51.03;
  x0 = -16.96;
  y0 = -43.25;
  cellSize = 1.0/K;

}

/// - destructor --------------------------------------------------------------
LaserSimulator::~LaserSimulator() {
}



/// - public --------------------------------------------------------------
bool LaserSimulator::onMap(unsigned int x, unsigned int y) {
  return (x<W && y<H);
}


/// - public --------------------------------------------------------------
LaserScan LaserSimulator::getScan(const RobotPosition &pose) {
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



PointList LaserSimulator::getRawPoints() {
  return points;
}

bool LaserSimulator::isFeasible(RobotPosition pos) {
  return (*grid)(real2gridY(pos.y), real2gridX(pos.x)) == FREESPACE_CELL;
}


void SWAP(int &x, int &y) {
  int p;
  p = x;
  x = y;
  y = p;
}


SIntPoint LaserSimulator::bresenham(int x0, int y0, int x1, int y1) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steep = (abs(dy) >= abs(dx));
  if (steep) {
    SWAP(x0, y0);
    SWAP(x1, y1);
    // recompute Dx, Dy after swap
    dx = x1 - x0;
    dy = y1 - y0;
  }
  int xstep = 1;
  if (dx < 0) {
    xstep = -1;
    dx = -dx;
  }
  int ystep = 1;
  if (dy < 0) {
    ystep = -1;
    dy = -dy;
  }
  int twoDy = 2 * dy;
  int twoDyTwoDx = twoDy - 2 * dx; // 2*Dy - 2*Dx
  int e = twoDy - dx; //2*Dy - Dx
  int y = y0;
  int xDraw, yDraw;
  for (int x = x0; x != x1; x += xstep) {
    if (steep) {
      xDraw = y;
      yDraw = x;
    } else {
      xDraw = x;
      yDraw = y;
    }
    if (xDraw <= 0 || yDraw <= 0 || xDraw >= grid->NCOLS || yDraw >= grid->NROWS ||  (*grid)(yDraw, xDraw) == OCCUPIED_CELL ) {     
      return SIntPoint(xDraw,yDraw);
//      setPixel(surface, xDraw,yDraw);
    }

    // next
    if (e > 0) {
      e += twoDyTwoDx; //E += 2*Dy - 2*Dx;
      y = y + ystep;
    } else {
      e += twoDy; //E += 2*Dy;
    }
  }
  return SIntPoint(x1,y1);
}

/* end of sndsimulator.cc */
