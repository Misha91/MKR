#ifndef __GUI__
#define __GUI__

#include <iostream>
#include <vector>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleImage.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkMath.h>
#include <vtkMatrix4x4.h>
#include <vtkPNGReader.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkVersion.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkGlyph3D.h>


namespace imr {

struct RobotPosition {
  double x;
  double y;
  double phi;
  RobotPosition() : x(0.0), y(0.0), phi(0.0) {}

  RobotPosition(const double &_x, const double &_y, const double &_phi)
      : x(_x), y(_y), phi(_phi) {}
};

struct Particle {
  RobotPosition pos;
  double weight;
};

typedef std::vector<Particle> ParticleVector;

struct Point {
  double x;
  double y;

  Point() : x(0.0), y(0.0) {}
  Point(const double &_x, const double &_y) : x(_x), y(_y) {}
};

typedef std::vector<Point> RawPoints;

Point robotPosition2point(const RobotPosition &rp);

namespace gui {
class Gui {
private:
  vtkSmartPointer<vtkPolyData> particles;
//   vtkSmartPointer<vtkVertexGlyphFilter> particlesFilter;
//   vtkSmartPointer<vtkPolyDataMapper> particlesMapper;
//   vtkSmartPointer<vtkActor> particlesActor;

  vtkSmartPointer<vtkPolyData> position;
//   vtkSmartPointer<vtkVertexGlyphFilter> positionFilter;
//   vtkSmartPointer<vtkPolyDataMapper> positionMapper;
//   vtkSmartPointer<vtkActor> positionActor;

  vtkSmartPointer<vtkRenderer> renderer;
  vtkSmartPointer<vtkRenderWindow> renderWindow;
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
  vtkSmartPointer<vtkInteractorStyleImage> interactorStyle;

  void RawPoints2vtkPoints(const RawPoints &pts,
                           vtkSmartPointer<vtkPoints> vtkPts);
  void RawPoints2vtkPoints(const Point &pts, vtkSmartPointer<vtkPoints> vtkPts);
  void Particles2vtkPoints(const ParticleVector &pts,
                           vtkSmartPointer<vtkPoints> vtkPts);
  void MakeGlyphs(vtkPolyData *src, double size, vtkGlyph3D *glyph, bool drawArrow=true);

public : 
  Gui(bool arrows=true);
  void startInteractor();

  void setPointsToMap(RawPoints pts, Point pos);
  void setPointsToMap2(RawPoints pts, Point pos);
  void clearPositionPoints();
  void setPosition(RobotPosition& pos);
  void clearMapPoints();
  void setParticlePoints(ParticleVector pts);
  void screenshot(std::string filename);
};
} // namespace gui
} // namespace imr

#endif
