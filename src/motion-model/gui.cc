#include "gui.h"
#include <vtkArrowSource.h>
#include <vtkCamera.h>
#include <vtkDoubleArray.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageData.h>
#include <vtkImageExport.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkTextureMapToPlane.h>
#include <vtkWindowToImageFilter.h>
#include <vtkDiskSource.h>
using namespace imr;
using namespace imr::gui;

Point imr::robotPosition2point(const RobotPosition &rp) {
  return Point(rp.x, rp.y);
}

Gui::Gui(bool arrows) {
  //   RawPoints initial;
  Point pos;

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  // RawPoints2vtkPoints(initial,points);

  particles = vtkSmartPointer<vtkPolyData>::New();
  particles->SetPoints(points);

  points = vtkSmartPointer<vtkPoints>::New();
  RawPoints2vtkPoints(pos, points);
  position = vtkSmartPointer<vtkPolyData>::New();
  position->SetPoints(points);

  //   particlesFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  // #if VTK_MAJOR_VERSION <= 5
  //   particlesFilter->AddInput(particles);
  // #else
  //   particlesFilter->AddInputData(particles);
  // #endif
  //   particlesFilter->Update();

  //   particlesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //   particlesMapper->SetInputConnection(particlesFilter->GetOutputPort());

  //   particlesActor = vtkSmartPointer<vtkActor>::New();
  //   particlesActor->SetMapper(particlesMapper);
  //   particlesActor->GetProperty()->SetColor(1, 0, 0);
  //   particlesActor->GetProperty()->SetPointSize(3);

  vtkSmartPointer<vtkGlyph3D> glyph3D = vtkSmartPointer<vtkGlyph3D>::New();
  MakeGlyphs(particles, 0.02, glyph3D.GetPointer(),arrows);

  vtkSmartPointer<vtkPolyDataMapper> glyph3DMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  glyph3DMapper->SetInputConnection(glyph3D->GetOutputPort());
  vtkSmartPointer<vtkActor> glyph3DActor = vtkSmartPointer<vtkActor>::New();
  glyph3DActor->SetMapper(glyph3DMapper);
  glyph3DActor->GetProperty()->SetColor(0.8900, 0.8100, 0.3400);

  //   positionFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  // #if VTK_MAJOR_VERSION <= 5
  //   positionFilter->AddInput(position);
  // #else
  //   positionFilter->AddInputData(position);
  // #endif
  //   positionFilter->Update();

  //   positionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //   positionMapper->SetInputConnection(positionFilter->GetOutputPort());

  //   positionActor = vtkSmartPointer<vtkActor>::New();
  //   positionActor->SetMapper(positionMapper);
  //   positionActor->GetProperty()->SetColor(0, 1, 0);
  //   positionActor->GetProperty()->SetPointSize(5);

  vtkSmartPointer<vtkGlyph3D> positionGlyph =
      vtkSmartPointer<vtkGlyph3D>::New();
  MakeGlyphs(position, 0.1, positionGlyph.GetPointer());

  vtkSmartPointer<vtkPolyDataMapper> positionGlyphMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  positionGlyphMapper->SetInputConnection(positionGlyph->GetOutputPort());
  vtkSmartPointer<vtkActor> positionGlyphActor =
      vtkSmartPointer<vtkActor>::New();
  positionGlyphActor->SetMapper(positionGlyphMapper);
  positionGlyphActor->GetProperty()->SetColor(0.3400, 0.8900, 0.8100);

  renderer = vtkSmartPointer<vtkRenderer>::New();
  renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->SetSize(1400, 1050);
  renderWindow->AddRenderer(renderer);

  renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  interactorStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
  renderWindowInteractor->SetInteractorStyle(interactorStyle);

  //   renderer->AddActor(particlesActor);
  //   renderer->AddActor(positionActor);
  renderer->AddActor(glyph3DActor);
  renderer->AddActor(positionGlyphActor);
  renderer->SetBackground(0.15, 0.15, 0.15);
  renderWindow->Render();
}

void Gui::MakeGlyphs(vtkPolyData *src, double size, vtkGlyph3D *glyph, bool drawArrow) {
  // Source for the glyph filter
  if (drawArrow) {
    vtkSmartPointer<vtkArrowSource> arrow = vtkSmartPointer<vtkArrowSource>::New();
    arrow->SetTipResolution(1);
    arrow->SetShaftResolution(1);
    arrow->SetTipLength(.4);
    arrow->SetTipRadius(.1);
    arrow->SetShaftRadius(.03);
    glyph->SetSourceConnection(arrow->GetOutputPort());
  } else {
    vtkSmartPointer<vtkDiskSource> ss = vtkSmartPointer<vtkDiskSource>::New();
    ss->SetInnerRadius(0);
    ss->SetOuterRadius(0.1);
    glyph->SetSourceConnection(ss->GetOutputPort());   
  }


  glyph->SetInputData(src);
  glyph->SetVectorModeToUseNormal();
  glyph->SetScaleModeToScaleByVector();
  glyph->SetScaleFactor(size);
  glyph->OrientOn();
  glyph->Update();
}

void Gui::RawPoints2vtkPoints(const RawPoints &pts,
                              vtkSmartPointer<vtkPoints> vtkPts) {
  for (int i = 0; i < pts.size(); i++) {
    vtkPts->InsertNextPoint(pts[i].x, pts[i].y, .0);
  }
}

void Gui::RawPoints2vtkPoints(const Point &pts,
                              vtkSmartPointer<vtkPoints> vtkPts) {
  vtkPts->InsertNextPoint(pts.x, pts.y, .0);
}

void Gui::Particles2vtkPoints(const ParticleVector &pts,
                              vtkSmartPointer<vtkPoints> vtkPts) {
  for (int i = 0; i < pts.size(); i++) {
    vtkPts->InsertNextPoint(pts[i].pos.x, pts[i].pos.y, .0);
  }
}

void Gui::startInteractor() { renderWindowInteractor->Start(); }

void Gui::setParticlePoints(ParticleVector pts) {
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  Particles2vtkPoints(pts, points);

  // Set point normals
  vtkSmartPointer<vtkDoubleArray> pointNormalsArray =
      vtkSmartPointer<vtkDoubleArray>::New();
  pointNormalsArray->SetNumberOfComponents(3); // 3d normals (ie x,y,z)
  pointNormalsArray->SetNumberOfTuples(pts.size());

  for (int i = 0; i < pts.size(); i++) {
    double pN1[3] = {cos(pts[i].pos.phi), sin(pts[i].pos.phi), 0.0};
    pointNormalsArray->SetTuple(i, pN1);
  }

  particles->SetPoints(points);
  particles->GetPointData()->SetNormals(pointNormalsArray);
  particles->Modified();
  double bounds[6];
  renderer->ComputeVisiblePropBounds(bounds);
  bounds[0] = 0;
  bounds[1] = 1;
  bounds[2] = -0.3;
  bounds[3] = 0.3;
  // std::cout << bounds[0] << " " << bounds[1] << " " << bounds[2] << " " << bounds[3] << std::endl;
  renderer->ResetCamera(bounds);
  renderWindow->Render();
}

void Gui::clearPositionPoints() {
  position->SetPoints(vtkSmartPointer<vtkPoints>::New());
  position->Modified();
  renderWindow->Render();
}

void Gui::setPosition(RobotPosition &pos) {
  vtkSmartPointer<vtkPoints> p = vtkSmartPointer<vtkPoints>::New();
  RawPoints2vtkPoints(robotPosition2point(pos), p);

  // Set point normals
  vtkSmartPointer<vtkDoubleArray> pointNormalsArray =
      vtkSmartPointer<vtkDoubleArray>::New();
  pointNormalsArray->SetNumberOfComponents(3); // 3d normals (ie x,y,z)
  pointNormalsArray->SetNumberOfTuples(1);

  double pN1[3] = {cos(pos.phi), sin(pos.phi), 0.0};
  pointNormalsArray->SetTuple(0, pN1);

  position->SetPoints(p);

  position->GetPointData()->SetNormals(pointNormalsArray);
  position->Modified();
  renderWindow->Render();
}

void Gui::screenshot(std::string filename) {

  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
      vtkSmartPointer<vtkWindowToImageFilter>::New();
  windowToImageFilter->SetInput(renderWindow);
  windowToImageFilter->SetMagnification(1); // set the resolution of the output
                                            // image (3 times the current
                                            // resolution of vtk render window)
  windowToImageFilter->SetInputBufferTypeToRGBA(); // also record the alpha
                                                   // (transparency) channel
  windowToImageFilter->ReadFrontBufferOff();
  windowToImageFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(filename.c_str());
  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
  writer->Write();
}
