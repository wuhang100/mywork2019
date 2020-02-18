/*
 * obj_scan.h
 *
 *  Created on: 2020年2月18日
 *      Author: wuhang
 */

#ifndef OBJ_SCAN_H_
#define OBJ_SCAN_H_

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkImageActor.h>
#include <vtkLight.h>
#include <vtkCamera.h>
#include <vtkProperty.h>
#include <vtkWindowToImageFilter.h>
#include <vtkInteractorObserver.h>
#include <vtkPLYReader.h>
#include <vtkInteractorObserver.h>
#include <vtkCellLocator.h>
#include <vtkOBBTree.h>
#include <vtkLine.h>
#include <vtkWorldPointPicker.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

class obj_scan{
private:
	vtkPolyData *data;
	bool rendered;
	int pixel[2];
	double viewpoint[3];
	double targetpoint[3];
	vtkSmartPointer<vtkPLYReader> reader = vtkPLYReader::New();
	vtkSmartPointer<vtkActor> cylinderActor=vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkPolyDataMapper> cylinderMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();  //管理对象的渲染场景（相机，光照等，世界坐标系
	vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();  //将渲染场景连接至操作系统

public:
	obj_scan(string const& filename, int *imgsize, double * cameraPt, double *targetPt);
	void render(double const& ViewAngle,double const& view_x,double const& view_y,double const& view_z);
	void world2view(double *intersection, double *displayPt);
	void OBBTreeintersect(double *intersection);
	void CellLocatorintersect(double *intersection);
	void vtk2cv2(string const& savename);
	void cvShow (string name, cv::Mat img);
	void showdepth(cv::Mat & renderedImg, double *displayPt);
	~obj_scan(){cout<<"Done "<<endl;};
};



#endif /* OBJ_SCAN_H_ */
