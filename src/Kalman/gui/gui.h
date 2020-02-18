#ifndef __GUI__
#define __GUI__

#include<vector>
#include<iostream>

#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleImage.h>



namespace imr {
    namespace gui {

        struct Point {
            double x;
            double y;

            Point() : x(0.0), y(0.0) {}
            Point(const double &_x, const double &_y) : x(_x), y(_y) {}
        };

        typedef std::vector<Point> RawPoints;

        class Gui
        {
            private:

                vtkSmartPointer<vtkPolyData> measurement;
                vtkSmartPointer<vtkVertexGlyphFilter> measurementFilter;
                vtkSmartPointer<vtkPolyDataMapper> measurementMapper; 
                vtkSmartPointer<vtkActor> measurementActor;

                vtkSmartPointer<vtkPolyData> position;
                vtkSmartPointer<vtkVertexGlyphFilter> positionFilter;
                vtkSmartPointer<vtkPolyDataMapper> positionMapper; 
                vtkSmartPointer<vtkPolyDataMapper> positionLineMapper; 
                vtkSmartPointer<vtkActor> positionActor;
                vtkSmartPointer<vtkActor> positionLineActor;

		vtkSmartPointer<vtkPolyData> kfPosition;
                vtkSmartPointer<vtkVertexGlyphFilter> kfPositionFilter;
                vtkSmartPointer<vtkPolyDataMapper> kfPositionMapper; 
                vtkSmartPointer<vtkPolyDataMapper> kfPositionLineMapper; 
                vtkSmartPointer<vtkActor> kfPositionActor;
                vtkSmartPointer<vtkActor> kfPositionLineActor;


                vtkSmartPointer<vtkRenderer> renderer; 
                vtkSmartPointer<vtkRenderWindow> renderWindow;
                vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor;
                vtkSmartPointer<vtkInteractorStyleImage> interactorStyle;
                
                void RawPoints2vtkPoints(const RawPoints &pts, vtkSmartPointer<vtkPoints> vtkPts); 
                void RawPoints2vtkPoints(const Point &pts,
                        vtkSmartPointer<vtkPoints> vtkPts); 

            public:
                Gui();
		void demo(RawPoints pts);
                void startInteractor();

                void setPoints(Point truth, Point measurement, Point kfPos);
		void clearPoints();
		void drawCovariance(float x, float y, float sx, float sy);
        };
    } // namespace gui
} // namespace imr

#endif
