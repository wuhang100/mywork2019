#include "obj_scan.h"

using namespace std;

int main(){
    double cameraPt[3] = {0.0, 0.0, 10.0};
	double targetPt[3] = {0.0, 0.0, 0.0};
	int imgsize[3] = {512, 512};
	double intersection[3];
	double depthpt[3];
	string savename = "../0.png";

	// 获取模型并渲染
	obj_scan cube("../cube.ply", imgsize, cameraPt, targetPt);
	cube.render(30, cameraPt[0], cameraPt[1], cameraPt[2]);
	cube.vtk2cv2(savename);
	// 获取深度并显示
	cube.OBBTreeintersect(intersection);
	cv::Mat outimg=cv::imread(savename);
	cube.world2view(intersection, depthpt);
	cube.showdepth(outimg, depthpt);

	return EXIT_SUCCESS;
}
