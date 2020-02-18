/*
 * obj_scan.cpp
 *
 *  Created on: 2020年2月18日
 *      Author: wuhang
 */


# include "obj_scan.h"

using namespace std;


obj_scan::obj_scan(string const& filename, int *imgsize, double * cameraPt, double *targetPt){
	reader->SetFileName(filename.c_str());
	reader->Update();
	data = reader->GetOutput();
	cout << "Load data from " << filename << endl;
	cout << "Number of points: " << data->GetNumberOfPoints() << endl;

	cylinderMapper->SetInputConnection(reader->GetOutputPort()); //接入将几何形状的输出接口
	cylinderActor->SetMapper(cylinderMapper);  //渲染对象几何特征
	cylinderActor->GetProperty()->SetColor(0.0,0.0,1.0);  //渲染对象属性
	cylinderActor->GetProperty()->SetOpacity(1.0);
	rendered = false;
	pixel[0] = imgsize[0];
	pixel[1] = imgsize[1];
	viewpoint[0] = cameraPt[0];
	viewpoint[1] = cameraPt[1];
	viewpoint[2] = cameraPt[2];
	targetpoint[0] = targetPt[0];
	targetpoint[1] = targetPt[1];
	targetpoint[2] = targetPt[2];
}

void obj_scan::render(double const& ViewAngle,double const& view_x,double const& view_y,double const& view_z){
	renderer->Clear();
	//（1）渲染对象
	renderer->AddActor(cylinderActor);
	//（2）场景背景
	renderer->SetBackground(1.0,1.0,1.0);
	//（3）相机
	camera->SetViewAngle(ViewAngle);
	camera->SetPosition(view_x,view_y,view_z);
	camera->SetFocalPoint(0.0,0.0,0.0);
	renderer->ResetCameraClippingRange();
	renderer->SetActiveCamera(camera);

	renWin->AddRenderer(renderer);
	renWin->SetSize(pixel[0],pixel[1]);
	renWin->SetWindowName("RenderImage");
	renWin->SetOffScreenRendering(1);
	renWin->Render();  //Render the pipeline
	rendered = true;
}

void obj_scan::world2view(double *intersection, double *displayPt){
	if (rendered==false){
		cerr << "Object not rendered yet, please render first!" << endl;
		exit(0);
	}
	else{
		vtkInteractorObserver::ComputeWorldToDisplay(renderer, intersection[0], intersection[1], intersection[2], displayPt);
		cout << "Display point: "<< round(displayPt[0]) << ", " << round(displayPt[1]) << ", " << round(displayPt[2]) << endl;
	}
}

void obj_scan::OBBTreeintersect(double *intersection){
	vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
	// Create the locator
	vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
	tree->SetDataSet(data);
	tree->BuildLocator();
	tree->IntersectWithLine(viewpoint, targetpoint, intersectPoints, NULL);
	intersectPoints->GetPoint(0, intersection);
	cout << "Intersection " << 0 << ": " << intersection[0] << ", "
			<< intersection[1] << ", " << intersection[2] << endl;
}

void obj_scan::CellLocatorintersect(double *intersection){
	vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
	cellLocator->SetDataSet(data);
	cellLocator->BuildLocator();
	double p_coords[3], t;
	int subId;
	vtkIdType cellId;
	if (cellLocator->IntersectWithLine(viewpoint, targetpoint, 0, t, intersection, p_coords, subId, cellId)>0){
		cout << "Intersection " << ": "<< intersection[0] << ", "<< intersection[1] << ", "<< intersection[2] << endl;
		cout << "t " << ": "<< t << endl;
		cout <<  endl;
	}
	else{
		cout << "No Intersection " << endl;
	}
}

void obj_scan::vtk2cv2(string const& savename){
	int dim[3];
	vtkSmartPointer<vtkWindowToImageFilter> wif = vtkSmartPointer<vtkWindowToImageFilter>::New();
	wif->SetInput(renWin);
	wif->Update();
	wif->GetOutput()->GetDimensions(dim);
	cv::Mat renderedImg(dim[1],dim[0],CV_8UC3,wif->GetOutput()->GetScalarPointer());
	cv::cvtColor(renderedImg, renderedImg, CV_BGR2RGB);
	cv::flip(renderedImg, renderedImg, 0);
	cv::imwrite(savename,renderedImg);
}

void obj_scan::cvShow(string name, cv::Mat img){
	cv::imshow(name, img);
	cv::waitKey();
}

void obj_scan::showdepth(cv::Mat & renderedImg, double *displayPt){
	cv::flip(renderedImg, renderedImg, 0);
	for (int i=round(displayPt[0])-2; i<=round(displayPt[0])+2 ; i++)
	{
	    for (int j=round(displayPt[1])-2; j<=round(displayPt[1])+2 ; j++)
	    {
	    	renderedImg.at<cv::Vec3b>(j,i)[0] = 0;
	    	renderedImg.at<cv::Vec3b>(j,i)[1] = 0;
	    	renderedImg.at<cv::Vec3b>(j,i)[2] = 255;
	    }
	}
	cv::flip(renderedImg, renderedImg, 0);
	obj_scan::cvShow("renderedImg", renderedImg);
}

