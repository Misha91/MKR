#include <iostream>

#include <math.h>  // For lround().

#include <vtkImageData.h>
#include <vtkPNGReader.h>
#include <vtkPNGWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkTextureMapToPlane.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>

#include "gui.h"

namespace imr
{
namespace gui
{

/** Return min and max values, without considering values outside [0, 1].
 */
template <typename T>
void normalizeProbabilities(const T& pts, double& min, double& max)
{
    min = 1;
    max = 0;
    for (auto& pt : pts)
    {
        if ((pt.weight < 0) || (pt.weight > 1))
        {
            continue;
        }
        if (min > pt.weight)
        {
            min = pt.weight;
        }
        if (max < pt.weight)
        {
            max = pt.weight;
        }
    }
    if (min > max)
    {
        // Problem with input, set values that don't make troubles with the lookup table.
        min = 0;
        max = 1;
    }
}

Gui::Gui(vtkSmartPointer<vtkPNGReader> reader)
{
    vtkSmartPointer<vtkImageData> imageData;
    imageData = reader->GetOutput();
    reader->Update();
    int dims[3];
    imageData->GetDimensions(dims);

    double K = 51.03;
    double X = -16.96;
    double Y = -43.25;

    vtkSmartPointer<vtkPlaneSource> plane = vtkSmartPointer<vtkPlaneSource>::New();
    plane->SetOrigin(X, Y, 0);
    plane->SetCenter(0.0, 0.0, 0.0);
    plane->SetNormal(0.0, 0.0, 1.0);
    plane->SetPoint1(X + 1872.0 / K, Y, 0.0);
    plane->SetPoint2(X, Y + 5015.0 / K, 0.0);

    plane->SetResolution(1, 1);

    // Apply the texture
    vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetInputConnection(reader->GetOutputPort());
    vtkSmartPointer<vtkTextureMapToPlane> texturePlane = vtkSmartPointer<vtkTextureMapToPlane>::New();
    texturePlane->SetInputConnection(plane->GetOutputPort());

    vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    planeMapper->SetInputConnection(texturePlane->GetOutputPort());

    vtkSmartPointer<vtkActor> textureActor = vtkSmartPointer<vtkActor>::New();
    textureActor->SetMapper(planeMapper);
    textureActor->SetTexture(texture);

    map = vtkSmartPointer<vtkPolyData>::New();
    map->SetPoints(vtkSmartPointer<vtkPoints>::New());
    mapFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    mapFilter->AddInput(map);
#else
    mapFilter->AddInputData(map);
#endif
    mapFilter->Update();

    mapMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapMapper->SetInputConnection(mapFilter->GetOutputPort());

    mapActor = vtkSmartPointer<vtkActor>::New();
    mapActor->SetMapper(mapMapper);
    mapActor->GetProperty()->SetColor(0, 1, 1);
    mapActor->GetProperty()->SetPointSize(3);

    particles = vtkSmartPointer<vtkPolyData>::New();
    particles->SetPoints(vtkSmartPointer<vtkPoints>::New());
    particlesFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    particlesFilter->AddInput(particles);
#else
    particlesFilter->AddInputData(particles);
#endif
    particlesFilter->Update();

    particlesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    particlesMapper->SetInputConnection(particlesFilter->GetOutputPort());

    particlesActor = vtkSmartPointer<vtkActor>::New();
    particlesActor->SetMapper(particlesMapper);

    scanMeasurement = vtkSmartPointer<vtkPolyData>::New();
    scanMeasurement->SetPoints(vtkSmartPointer<vtkPoints>::New());
    scanFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    scanFilter->AddInput(scanMeasurement);
#else
    scanFilter->AddInputData(scanMeasurement);
#endif
    scanFilter->Update();

    scanMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    scanMapper->SetInputConnection(scanFilter->GetOutputPort());

    scanActor = vtkSmartPointer<vtkActor>::New();
    scanActor->SetMapper(scanMapper);
    scanActor->GetProperty()->SetColor(1, 0, 0);
    scanActor->GetProperty()->SetPointSize(3);

    position = vtkSmartPointer<vtkPolyData>::New();
    position->SetPoints(vtkSmartPointer<vtkPoints>::New());

    positionFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    positionFilter->AddInput(position);
#else
    positionFilter->AddInputData(position);
#endif
    positionFilter->Update();

    positionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    positionMapper->SetInputConnection(positionFilter->GetOutputPort());

    positionActor = vtkSmartPointer<vtkActor>::New();
    positionActor->SetMapper(positionMapper);
    positionActor->GetProperty()->SetColor(0, 1, 0);
    positionActor->GetProperty()->SetPointSize(5);


    probabilityData = vtkSmartPointer<vtkPolyData>::New();
    probabilityData->SetPoints(vtkSmartPointer<vtkPoints>::New());

    probabilityFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    probabilityFilter->AddInput(probabilityData);
#else
    probabilityFilter->AddInputData(probabilityData);
#endif
    probabilityFilter->Update();

    probabilityMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    probabilityMapper->SetInputConnection(probabilityFilter->GetOutputPort());

    probabilityActor = vtkSmartPointer<vtkActor>::New();
    probabilityActor->SetMapper(probabilityMapper);
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);

    interactorStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
    renderWindowInteractor->SetInteractorStyle(interactorStyle);

    renderer->AddActor(textureActor);
    renderer->AddActor(particlesActor);
    renderer->AddActor(mapActor);
    renderer->AddActor(scanActor);
    renderer->AddActor(positionActor);
    renderer->AddActor(probabilityActor);
    renderer->SetBackground(0.5, 0.5, 0.5);
    renderWindow->Render();
}

void Gui::vtkPointsFromRawPoints(const PointList& pts, vtkSmartPointer<vtkPoints> vtkPts)
{
    for (auto& pt : pts)
    {
        vtkPts->InsertNextPoint(pt.x, pt.y, 0.0);
    }
}

void Gui::vtkPointsFromRawPoints(const Point& pts, vtkSmartPointer<vtkPoints> vtkPts)
{
    vtkPts->InsertNextPoint(pts.x, pts.y, 0.0);
}

void Gui::vtkPointsFromParticles(const ParticleVector& pts, bool with_colormap, vtkSmartPointer<vtkPoints> vtkPts, vtkSmartPointer<vtkUnsignedCharArray> vtkColors)
{
    // vtkPts->Reset() and vtkColors->Reset() segfault, so this is up to the
    // caller to ensure that vtkPts and vtkColors are empty or have at least
    // the same number of elements.
    
    vtkSmartPointer<vtkLookupTable> colorLookupTable;
    if (with_colormap)
    {
        // Normalize the colormap.
        colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
        double min;
        double max;
        normalizeProbabilities(pts, min, max);
        colorLookupTable->SetTableRange(min, max);
        colorLookupTable->Build(); 
    }

    vtkColors->SetNumberOfComponents(3);
    for (auto& pt : pts)
    {
        vtkPts->InsertNextPoint(pt.pos.x, pt.pos.y, 0.0);
        double color[3];  // Color within [0, 1].
        if (with_colormap)
        {
            if ((pt.weight < 0) || (pt.weight > 1))
            {
                // Mark a probability outside [0, 1] as grey.
                color[0] = 0.6;
                color[1] = 0.6;
                color[2] = 0.6;
            }
            else
            {
                colorLookupTable->GetColor(pt.weight, color);
            }
        }
        else
        {
            // Uniform color: red.
            color[0] = 1;
            color[1] = 0;
            color[2] = 0;
        }
        unsigned char color_i[3];  // Color within [0, 255].
        for (size_t c = 0; c < 3; c++)
        {
            color_i[c] = static_cast<unsigned char>(lround(color[c] * 255));
        }
        vtkColors->InsertNextTupleValue(color_i);
    }
}

void Gui::vtkPointsFromWeightedPoints(const WeightedPointList& pts, bool normalize, vtkSmartPointer<vtkPoints> vtkPts, vtkSmartPointer<vtkUnsignedCharArray> vtkColors)
{
    // Range for the colormap, without considering values outside [0, 1].
    double min;
    double max;
    if (normalize)
    {
        normalizeProbabilities(pts, min, max);
    }
    else
    {
        min = 0;
        max = 1;
    }
    // Cf. https://www.vtk.org/Wiki/VTK/Examples/Cxx/Meshes/ColoredElevationMap.
    vtkSmartPointer<vtkLookupTable> colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
    colorLookupTable->SetTableRange(min, max);
    colorLookupTable->Build(); 

    // vtkPts->Reset() and vtkColors->Reset() segfault, so this is up to the caller to ensure that vtkPts and vtkColors are empty.
    vtkColors->SetNumberOfComponents(3);
    for (auto& pt : pts)
    {
        vtkPts->InsertNextPoint(pt.pos.x, pt.pos.y, 0.0);
        double color[3];  // Color within [0, 1].
        if ((pt.weight < 0) || (pt.weight > 1))
        {
            // Mark a probability outside [0, 1] as grey.
            color[0] = 0.6;
            color[1] = 0.6;
            color[2] = 0.6;
        }
        else
        {
            colorLookupTable->GetColor(pt.weight, color);
        }
        unsigned char color_i[3];  // Color within [0, 255].
        for (size_t c = 0; c < 3; c++)
        {
            color_i[c] = static_cast<unsigned char>(lround(color[c] * 255));
        }
        vtkColors->InsertNextTupleValue(color_i);
    }
}

void Gui::startInteractor()
{
    renderWindowInteractor->Start();
}

void Gui::setScanPoints(const PointList& pts)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkPointsFromRawPoints(pts, points);

    scanMeasurement->SetPoints(points);
    scanMeasurement->Modified();
    renderWindow->Render();
}

void Gui::setParticlePoints(const ParticleVector& pts, bool with_colormap, double size)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    vtkPointsFromParticles(pts, with_colormap, points, colors);

    particles->SetPoints(points);
    particles->GetPointData()->SetScalars(colors);
    particlesActor->GetProperty()->SetPointSize(size);
    renderWindow->Render();
}

void Gui::clearScanPoints()
{
    scanMeasurement->SetPoints(vtkSmartPointer<vtkPoints>::New());
    scanMeasurement->Modified();
    renderWindow->Render();
}

void Gui::clearMapPoints()
{
    map->SetPoints(vtkSmartPointer<vtkPoints>::New());
    map->Modified();
    renderWindow->Render();
}

void Gui::clearPositionPoints()
{
    position->SetPoints(vtkSmartPointer<vtkPoints>::New());
    position->Modified();
    renderWindow->Render();
}

void Gui::setPosition(const Point& pos)
{
    vtkSmartPointer<vtkPoints> p = position->GetPoints();
    vtkPointsFromRawPoints(pos, p);

    position->SetPoints(p);
    position->Modified();

    renderWindow->Render();
}

void Gui::setPointsToMap(const PointList& pts, const Point& pos)
{
    vtkSmartPointer<vtkPoints> p = position->GetPoints();
    vtkPointsFromRawPoints(pos, p);

    position->SetPoints(p);
    position->Modified();

    vtkSmartPointer<vtkPoints> points = map->GetPoints();
    vtkPointsFromRawPoints(pts, points);

    map->SetPoints(points);
    map->Modified();
    renderWindow->Render();
}

void Gui::clearProbabilityMap()
{
    probabilityData->SetPoints(vtkSmartPointer<vtkPoints>::New());
    probabilityData->Modified();
    renderWindow->Render();
}

void Gui::setProbabilityMap(const WeightedPointList& probs, bool normalize, double size)
{
    probabilityData->SetPoints(vtkSmartPointer<vtkPoints>::New());
    vtkSmartPointer<vtkPoints> p = probabilityData->GetPoints();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    vtkPointsFromWeightedPoints(probs, normalize, p, colors);
    probabilityData->GetPointData()->SetScalars(colors);

    probabilityActor->GetProperty()->SetPointSize(size);
    renderWindow->Render();
}

void Gui::screenshot(const std::string& filename)
{
    renderer->ResetCamera();
    renderWindow->Render();

    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter = vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    // Set the resolution of the output image (5 times the current resolution
    // of vtk render window).
    windowToImageFilter->SetMagnification(5);
    // Also record the alpha (transparency) channel.
    windowToImageFilter->SetInputBufferTypeToRGBA();
    windowToImageFilter->ReadFrontBufferOff();
    windowToImageFilter->Update();

    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(filename.c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
}

} // namespace gui
} // namespace imr

