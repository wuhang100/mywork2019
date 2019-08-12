#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/centroid.h>

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

void get2pcl (string file_dir, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_s){
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
	//cout << strArray.size() << endl;

	cloud_s->width = strArray.size();
	cloud_s->height = 1;
	cloud_s->points.resize (cloud_s->width * cloud_s->height);
	for (int i=0; i<cloud_s->points.size(); i++){
		cloud_s->points[i].x = 3*strArray[i][0];
		cloud_s->points[i].y = 3*strArray[i][1];
		cloud_s->points[i].z = 3*strArray[i][2];
		 //cout << "x: " << cloud->points[i].x << " y: " << cloud->points[i].y << "z: " << cloud->points[i].z << endl;
	}
}

Eigen::Matrix4f gettwistMatrix (char axis_twist, float theta, float x, float y, float z){
	float theta_ = M_PI * theta / 180.0;
	Eigen::Matrix4f twistMatrix;
	Eigen::Vector4f point;
	switch (axis_twist){
	case 'x':
		twistMatrix << 1 , 0 , 0 , x,
			     	   0 , cos(theta_) , -sin(theta_), y,
					   0 , sin(theta_) , cos(theta_), z,
					   0 , 0 , 0 , 1;
		break;
	case 'y':
		twistMatrix << cos(theta_) , 0 , -sin(theta_), x,
		         	   0 , 1 , 0 , y,
					   sin(theta_) , 0 , cos(theta_), z,
					   0 , 0 , 0 , 1;
		break;
	case 'z':
		twistMatrix << cos(theta_) , -sin(theta_) , 0 , x,
			     	   sin(theta_) , cos(theta_) , 0 , y,
					   0 , 0 , 1 , z,
					   0 , 0 , 0 , 1;
		break;
	}
	return twistMatrix;
}

void twistCloud_s (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_s, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,\
				   Eigen::Matrix4f & transMatrix, \
				   char axis_twist, float theta, \
				   float x, float y, float z){
	Eigen::Vector4f point;
	Eigen::Matrix4f twistMatrix;
	twistMatrix = gettwistMatrix (axis_twist, theta, x, y, z);
	transMatrix = twistMatrix * transMatrix;
	for (int i=0; i<cloud_s->size();i++){
		point << cloud_s->points[i].x , cloud_s->points[i].y , cloud_s->points[i].z , 1;
		point = transMatrix * point;
		cloud->points[i].x = point(0,0);
		cloud->points[i].y = point(1,0);
		cloud->points[i].z = point(2,0);
	}
}   // rotate about fixed frame

void twistCloud_b (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_s, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,\
		           Eigen::Matrix4f & transMatrix, \
				   char axis_twist, float theta, \
				   float x, float y, float z){
	Eigen::Vector4f point;
	Eigen::Matrix4f twistMatrix;
	twistMatrix = gettwistMatrix (axis_twist, theta, x, y, z);
	transMatrix = transMatrix * twistMatrix;
	for (int i=0; i<cloud_s->size();i++){
		point << cloud_s->points[i].x , cloud_s->points[i].y , cloud_s->points[i].z , 1;
		point = transMatrix * point;
		cloud->points[i].x = point(0,0);
		cloud->points[i].y = point(1,0);
		cloud->points[i].z = point(2,0);
	}
}   // rotate about body frame

void viewcloud (pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,string id){
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> point_color (cloud, r, g, b);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> point_color (cloud);
	viewer.setBackgroundColor (0.0, 0.0, 0.0);
	//viewer.addPointCloud(cloud,point_color,id);
	viewer.addPointCloud(cloud,id);
	viewer.addCoordinateSystem();
}

void movecloud (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_, char axis_trans, float distance){
	switch(axis_trans){
	case 'x':
		for (int i=0; i<cloud_->size();i++){
			cloud_->points[i].x = cloud_->points[i].x + distance;
		}
		break;
	case 'y':
		for (int i=0; i<cloud_->size();i++){
			cloud_->points[i].y = cloud_->points[i].y + distance;
		}
		break;
	case 'z':
		for (int i=0; i<cloud_->size();i++){
			cloud_->points[i].z = cloud_->points[i].z + distance;
		}
		break;
	}
}

void normcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_){
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cloud_,centroid);
    //cout << "x:" << centroid(0,0) << " y:" << centroid(1,0) << " z:" << centroid(2,0) << " ?:" << centroid(3,0) << endl;
	movecloud (cloud_, 'x', -centroid(0,0));
	movecloud (cloud_, 'y', -centroid(1,0));
	movecloud (cloud_, 'z', -centroid(2,0));
}

void computenormals (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_,
			pcl::PointCloud<pcl::Normal>::Ptr & normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr & fpfhs){
	pcl::search::Search<PointXYZ>::Ptr tree;
	cloud_->clear();
	*cloud_ = *cloud;
	normcloud(cloud_);
    if (cloud_->isOrganized ()){
      tree.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
    }
    else{
      tree.reset (new pcl::search::KdTree<PointXYZ> (false));
    }
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    normals->clear();
    ne.setInputCloud (cloud_);
    ne.setSearchMethod (tree);
    //ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    //ne.setRadiusSearch (0.03);
    ne.setKSearch(20);
    ne.compute (*normals);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfhs->clear();
    fpfh.setInputCloud (cloud_);
    fpfh.setInputNormals (normals);
/*    for (int i =0; i<30; i++)
    cout << "The normal for index 1 (" << cloud->points[i].x <<
    		" " << cloud->points[i].y << " " << cloud->points[i].z <<
			") is: "<< normals->points[i].normal_x << " " << normals->points[i].normal_y << " " <<
			normals->points[i].normal_z << endl;*/
    fpfh.setSearchMethod (tree);
    //fpfh.setRadiusSearch (0.05);
    fpfh.setKSearch(40);
    fpfh.compute (*fpfhs);
}

void plotfphm (pcl::visualization::PCLPlotter & plotter, pcl::PointCloud<pcl::FPFHSignature33>::Ptr & fpfhs){
	plotter.clearPlots();
	plotter.addFeatureHistogram<pcl::FPFHSignature33>(*fpfhs, "fpfh",100);
	plotter.plot();
	plotter.close();
	//plotter.spinOnce();
}

int main(){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
	pcl::visualization::PCLPlotter plotter;
	Eigen::Vector4f centroid;
	Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Identity(4,4);
	//cout << transMatrix << endl;

    string file_dir;
    cout << "Enter the file directory (/home/wuhang/tensorwork/pointcloudml/data/data_1_0.csv)" << endl;
    cin >> file_dir;
    int i = 15,j = 0,k=0;
    //  /home/wuhang/tensorwork/pointcloudml/data/data_1_0.csv
    get2pcl (file_dir, cloud_s);
    normcloud(cloud_s);
    *cloud = *cloud_s;
    cout << "The size of the cloud is: " << cloud->width << " " << cloud->height << endl;
    twistCloud_s (cloud_s, cloud ,transMatrix, 'x', 90, 0, 0, 0);

    char axis_twist,axis_trans;
    float theta=2,distance=0.2;
    //cout << "Enter the twist_axis and theta: ";
    //cin >> axis_twist >> theta;
    //twistCloud_b (cloud_s, cloud, transMatrix, axis_twist, theta, 0 ,0 ,0);
    int demo_mode;
    cout << "Input demo mode: 1. twist 2. rotate and roll: ";
    cin >> demo_mode;

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	while (!viewer.wasStopped ()){
	    viewcloud (viewer, cloud, "cloud");
	    viewer.spinOnce ();
	    computenormals (cloud,cloud_,normals,fpfhs);
	    cout << "The fpfhs for index 1 (" << cloud->points[1].x <<
	    		" " << cloud->points[1].y << " " << cloud->points[1].z <<
				") is: "<< fpfhs->points[1] << endl;
	    plotfphm (plotter, fpfhs);
	    //sleep(0.5);

	    if (demo_mode==1){
	        if (i>29){
	        	distance = -distance;
	        	i = 0;
	        }
	        i++;

	        twistCloud_b (cloud_s,cloud,transMatrix,'z',0,0,0,-0.5);
	        twistCloud_b (cloud_s,cloud,transMatrix,'y',5,0,0,0);
	        twistCloud_s (cloud_s,cloud,transMatrix,'z',0,0,0,distance);

	    }

	    if (demo_mode==2){
	        if (i>29){
	        	theta = -theta;
	        	i = 0;
	        }
	        i++;

	        twistCloud_s (cloud_s,cloud,transMatrix,'x',theta,0,0,0);
	        twistCloud_s (cloud_s,cloud,transMatrix,'y',theta,0,0,0);
	        twistCloud_s (cloud_s,cloud,transMatrix,'z',10,0,0,0);

	    }

	    /*cout << "Enter the twist_axis and theta: ";
	    cin >> axis_twist >> theta;
	    twistcloud (cloud, axis_twist, theta);
	    cout << "Enter the trans_axis and distance: ";
	    cin >> axis_trans >> distance;
	    transcloud (cloud, axis_trans, distance);*/
	    viewer.removePointCloud();
	}

}
