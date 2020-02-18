#include "gui.h"
#include <vtkRendererCollection.h>

using namespace imr::gui;


Gui::Gui()
{

    measurement = vtkSmartPointer<vtkPolyData>::New();
    measurement->SetPoints(vtkSmartPointer<vtkPoints>::New());

    position = vtkSmartPointer<vtkPolyData>::New();
    position->SetPoints(vtkSmartPointer<vtkPoints>::New());
    
    kfPosition = vtkSmartPointer<vtkPolyData>::New();
    kfPosition->SetPoints(vtkSmartPointer<vtkPoints>::New());
   

    measurementFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        measurementFilter->AddInput(measurement);
    #else
        measurementFilter->AddInputData(measurement);
    #endif
    measurementFilter->Update();

    measurementMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    measurementMapper->SetInputConnection(measurementFilter->GetOutputPort());

    measurementActor = vtkSmartPointer<vtkActor>::New();
    measurementActor->SetMapper(measurementMapper);
    measurementActor->GetProperty()->SetColor(1,0,0);
    measurementActor->GetProperty()->SetPointSize(3);
    
    positionFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        positionFilter->AddInput(position);
    #else
        positionFilter->AddInputData(position);
    #endif
    positionFilter->Update();

    positionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    positionMapper->SetInputConnection(positionFilter->GetOutputPort());
    
    positionLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    #if VTK_MAJOR_VERSION <= 5
      positionLineMapper->SetInput(position);
    #else
      positionLineMapper->SetInputData(position);
    #endif

    positionActor = vtkSmartPointer<vtkActor>::New();
    positionActor->SetMapper(positionMapper);
    positionActor->GetProperty()->SetColor(0,1,0);
    positionActor->GetProperty()->SetPointSize(5);
    
    positionLineActor = vtkSmartPointer<vtkActor>::New();
    positionLineActor->SetMapper(positionLineMapper);
    positionLineActor->GetProperty()->SetColor(0,1,0);

    kfPositionFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        kfPositionFilter->AddInput(kfPosition);
    #else
        kfPositionFilter->AddInputData(kfPosition);
    #endif
    kfPositionFilter->Update();

    kfPositionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    kfPositionMapper->SetInputConnection(kfPositionFilter->GetOutputPort());

    kfPositionLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();

    kfPositionActor = vtkSmartPointer<vtkActor>::New();
    #if VTK_MAJOR_VERSION <= 5
      kfPositionLineMapper->SetInput(kfPosition);
    #else
      kfPositionLineMapper->SetInputData(kfPosition);
    #endif

    kfPositionActor->SetMapper(kfPositionMapper);
    kfPositionActor->GetProperty()->SetColor(0,0,1);
    kfPositionActor->GetProperty()->SetPointSize(5);

    kfPositionLineActor = vtkSmartPointer<vtkActor>::New();
    kfPositionLineActor->SetMapper(kfPositionLineMapper);
    kfPositionLineActor->GetProperty()->SetColor(0,0,1);


    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    
    interactorStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
    renderWindowInteractor->SetInteractorStyle(interactorStyle);

    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);

    renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    
    interactorStyle = vtkSmartPointer<vtkInteractorStyleImage>::New();
    renderWindowInteractor->SetInteractorStyle(interactorStyle);

    renderer->AddActor(measurementActor);
    renderer->AddActor(positionActor);
    renderer->AddActor(positionLineActor);
    renderer->AddActor(kfPositionActor);
    renderer->AddActor(kfPositionLineActor);
    renderer->SetBackground(1,1,1);
    renderWindow->Render();
}

void Gui::drawCovariance(float x, float y , float sx, float sy) {
}

void Gui::RawPoints2vtkPoints(const RawPoints &pts, vtkSmartPointer<vtkPoints> vtkPts)
{
    for(int i=0; i<pts.size(); i++) {
        vtkPts->InsertNextPoint(pts[i].x, pts[i].y, .0);
    }
}
void Gui::RawPoints2vtkPoints(const Point &pts, vtkSmartPointer<vtkPoints> vtkPts)
{
        vtkPts->InsertNextPoint(pts.x/100.0, pts.y/100.0, .0);
}
void Gui::startInteractor()
{
    renderWindowInteractor->Start();
}


void Gui::clearPoints()
{
    measurement->SetPoints(vtkSmartPointer<vtkPoints>::New());
    position->SetPoints(vtkSmartPointer<vtkPoints>::New());
    kfPosition->SetPoints(vtkSmartPointer<vtkPoints>::New());
    measurement->Modified();
    position->Modified();
    kfPosition->Modified();
    renderWindow->Render();
}


void Gui::setPoints(Point truth, Point measured, Point kfPos)
{
    vtkSmartPointer<vtkPoints> p = position->GetPoints();
    RawPoints2vtkPoints(truth,p);

    vtkSmartPointer<vtkPolyLine> polyLine =   vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(position->GetNumberOfPoints());
    for(unsigned int i = 0; i < position->GetNumberOfPoints(); i++)
    {
       
       polyLine->GetPointIds()->SetId(i,i);
    }

    // Create a cell array to store the lines in and add the lines to it
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(polyLine);
    //                        
    //                          // Add the lines to the dataset
    position->SetLines(cells);

    vtkSmartPointer<vtkPoints> ip = kfPosition->GetPoints();
    RawPoints2vtkPoints(kfPos,ip);

    vtkSmartPointer<vtkPolyLine> kfPolyLine =   vtkSmartPointer<vtkPolyLine>::New();
    kfPolyLine->GetPointIds()->SetNumberOfIds(kfPosition->GetNumberOfPoints());
    for(unsigned int i = 0; i < kfPosition->GetNumberOfPoints(); i++)
    {
       
       kfPolyLine->GetPointIds()->SetId(i,i);
    }

    // Create a cell array to store the lines in and add the lines to it
    vtkSmartPointer<vtkCellArray> kfCells = vtkSmartPointer<vtkCellArray>::New();
    kfCells->InsertNextCell(kfPolyLine);
    //                        
    //                          // Add the lines to the dataset
    kfPosition->SetLines(kfCells);



    vtkSmartPointer<vtkPoints> points = measurement->GetPoints();
    RawPoints2vtkPoints(measured,points);
    measurement->Modified();

    renderWindow->GetRenderers()->GetFirstRenderer()->ResetCamera();
    renderWindow->Render();
}
