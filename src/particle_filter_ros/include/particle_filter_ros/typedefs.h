#ifndef IMR_TYPEDEFS_H
#define IMR_TYPEDEFS_H

#include <functional>
#include <vector>

namespace particle_filter_ros
{

struct Point
{
    double x;
    double y;

    Point() : x(0.0), y(0.0) {}
    Point(const Point& other) : x(other.x), y(other.y) {}
    Point(double _x, double _y) : x(_x), y(_y) {}
};

typedef std::vector<Point> PointList;

struct RobotPosition
{
    double x;
    double y;
    double phi;
    RobotPosition() : x(0.0), y(0.0), phi(0.0) {}

    RobotPosition(double _x, double _y, double _phi) : x(_x), y(_y), phi(_phi) {}
};

typedef std::vector<double> LaserScan;

struct Measurement
{
    RobotPosition position;
    LaserScan scan;
};

typedef std::vector<Measurement> MeasurementList;
    
struct Particle
{
    RobotPosition pos;
    double weight;

    Particle() {}
    Particle(const RobotPosition& pos, double weight) : pos(pos), weight(weight) {}
};

typedef std::vector<Particle> ParticleVector;

} /* namespace particle_filter_ros */

#endif /* ifndef IMR_TYPEDEFS_H */
