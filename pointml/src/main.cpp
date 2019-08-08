#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <math.h>

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>


using namespace std;
using namespace pcl;

template <class Type>
Type stringToNum(const string& str){
	istringstream iss(str);
	Type num;
	iss >> num;
	return num;
}

void get2pcl (string file_dir, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud){
	ifstream inFile(file_dir, ios::in);
	string lineStr;
	int line_index;
	vector<vector<float>> strArray;
	while (getline(inFile, lineStr)){
		line_index = 1;
		stringstream ss(lineStr);
		string str;
		vector<float> lineArray;
		while (getline(ss, str, ',')){
		    if (line_index >= 0 && line_index <= 4){
		        lineArray.push_back(stringToNum<float>(str));
		    }
		    line_index++;
		}
		strArray.push_back(lineArray);
	}
	//cout << strArray[0][0] << endl;
	cout << strArray.size() << endl;

	cloud->width = strArray.size();
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	for (int i=0; i<cloud->points.size(); i++){
		 cloud->points[i].x = 3*strArray[i][0];
		 cloud->points[i].y = 3*strArray[i][1];
		 cloud->points[i].z = 3*strArray[i][2];
		 //cout << "x: " << cloud->points[i].x << " y: " << cloud->points[i].y << "z: " << cloud->points[i].z << endl;
	}
}

void twistcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, char axis_twist, float theta){
	float theta_ = M_PI * theta / 180.0;
	Eigen::Matrix3f twist;
	Eigen::Vector3f point;
	switch (axis_twist){
	case 'x':
		twist << 1 , 0 , 0,
			     0 , cos(theta_) , -sin(theta_),
			     0 , sin(theta_) , cos(theta_);
		break;
	case 'y':
		twist << cos(theta_) , 0 , -sin(theta_),
		         0 , 1 , 0,
			     sin(theta_) , 0 , cos(theta_);
		break;
	case 'z':
		twist << cos(theta_) , -sin(theta_) , 0,
			     sin(theta_) , cos(theta_) , 0,
			     0 , 0 , 1;
		break;
	}
	for (int i=0; i<cloud->size();i++){
		point << cloud->points[i].x , cloud->points[i].y , cloud->points[i].z;
		point = twist * point;
		cloud->points[i].x = point(0,0);
		cloud->points[i].y = point(1,0);
		cloud->points[i].z = point(2,0);
	}
}


void viewcloud (pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,string id){
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_color (cloud, r, g, b);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> point_color (cloud);
	viewer.setBackgroundColor (0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud,point_color,id);
	viewer.addCoordinateSystem();
}

int main(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    string file_dir;
    cout << "Enter the file directory" << endl;
    cin >> file_dir;
    int i;
    //  /home/wuhang/tensorwork/pointcloudml/data/data_1_0.csv
    get2pcl (file_dir, cloud);
    cout << "The size of the cloud is: " << cloud->width << " " << cloud->height << endl;
    twistcloud (cloud, 'x', 90);

    char axis_twist;
    float theta;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

	while (!viewer.wasStopped ()){
	    viewcloud (viewer, cloud, "cloud");
	    viewer.spinOnce ();

	    cout << "Enter the twist_axis and theta: ";
	    cin >> axis_twist >> theta;
	    twistcloud (cloud, axis_twist, theta);
	    viewer.removePointCloud();
	}

}
