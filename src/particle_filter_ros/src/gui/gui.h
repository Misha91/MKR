#ifndef __GUI__
#define __GUI__

#include <vector>

#include <vtkActor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookupTable.h>
#include <vtkPNGReader.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>
//#include <typedefs.h>
//#include "dataLoader/laserDataLoader.h"

namespace imr
{
namespace gui
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

//typedef std::function<RobotPosition(double /* drot1 */, double /* dtrans */, double /* drot2 */, double /* alpha1 */, double /* alpha2 */, double /* alpha3 */, double /* alpha4 */)> MotionModelFunction;

struct WeightedPoint
{
    Point pos;
    double weight;

    WeightedPoint() : pos(0.0, 0.0), weight(0.0) {}
    WeightedPoint(Point& p, double weight) : pos(p), weight(weight) {}
    WeightedPoint(double x, double y, double weight) : pos(x, y), weight(weight) {}
};

typedef std::vector<WeightedPoint> WeightedPointList;

class Gui
{
    private:

        vtkSmartPointer<vtkPolyData> map;
        vtkSmartPointer<vtkVertexGlyphFilter> mapFilter;
        vtkSmartPointer<vtkPolyDataMapper> mapMapper;
        vtkSmartPointer<vtkActor> mapActor;

        vtkSmartPointer<vtkPolyData> particles;
        vtkSmartPointer<vtkVertexGlyphFilter> particlesFilter;
        vtkSmartPointer<vtkPolyDataMapper> particlesMapper;
        vtkSmartPointer<vtkActor> particlesActor;

        vtkSmartPointer<vtkPolyData> scanMeasurement;
        vtkSmartPointer<vtkVertexGlyphFilter> scanFilter;
        vtkSmartPointer<vtkPolyDataMapper> scanMapper;
        vtkSmartPointer<vtkActor> scanActor;

        vtkSmartPointer<vtkPolyData> position;
        vtkSmartPointer<vtkVertexGlyphFilter> positionFilter;
        vtkSmartPointer<vtkPolyDataMapper> positionMapper;
        vtkSmartPointer<vtkActor> positionActor;

        vtkSmartPointer<vtkPolyData> probabilityData;
        vtkSmartPointer<vtkVertexGlyphFilter> probabilityFilter;
        vtkSmartPointer<vtkPolyDataMapper> probabilityMapper;
        vtkSmartPointer<vtkActor> probabilityActor;

        vtkSmartPointer<vtkRenderer> renderer;
        vtkSmartPointer<vtkRenderWindow> renderWindow;
        vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
        vtkSmartPointer<vtkInteractorStyleImage> interactorStyle;

        void vtkPointsFromRawPoints(const PointList& pts, vtkSmartPointer<vtkPoints> vtkPts);
        void vtkPointsFromRawPoints(const Point& pts, vtkSmartPointer<vtkPoints> vtkPts);
        void vtkPointsFromParticles(const ParticleVector& pts, bool with_colormap, vtkSmartPointer<vtkPoints> vtkPts, vtkSmartPointer<vtkUnsignedCharArray> vtkColors);
        void vtkPointsFromWeightedPoints(const WeightedPointList& pts, bool normalize, vtkSmartPointer<vtkPoints> vtkPts, vtkSmartPointer<vtkUnsignedCharArray> vtkColors);

    public:

        Gui(vtkSmartPointer<vtkPNGReader> reader);
        void startInteractor();

        void setPointsToMap(const PointList& pts, const Point& pos);
        void clearPositionPoints();
        void setPosition(const Point& pos);
        void clearMapPoints();
        void setScanPoints(const PointList& pts);

        /** Draw particles
         *
         * Draw each particle given in metric coordinate system with a color
         * depending on the particle's weight. min --> red, max --> blue.
         * If normalize is true, min and max correspond to the weight range
         * without considering the values outside [0, 1], else min=0 and max=1.
         *
         * Values out of [0, 1] are shown in grey.
         */
        void setParticlePoints(const ParticleVector& pts, bool with_colormap = true, double size = 3);

        void clearScanPoints();
        void screenshot(const std::string& filename);

        /** Remove all the points from the probability map
         */
        void clearProbabilityMap();

        /** Add a probability map
         *
         * Draw each point given in metric coordinate system with a color
         * depending on the point's weight. min --> red, max --> blue.
         * If normalize is true, min and max correspond to the weight range
         * without considering the values outside [0, 1], else min=0 and max=1.
         *
         * Values out of [0, 1] are shown in grey.
         */
        void setProbabilityMap(const WeightedPointList& probs, bool normalize = true, double size = 3);
};

} // namespace gui
} // namespace imr

#endif

