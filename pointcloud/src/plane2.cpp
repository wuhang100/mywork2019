#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

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

	cloud_s->width = strArray.size();
	cloud_s->height = 1;
	cloud_s->points.resize (cloud_s->width * cloud_s->height);
	for (int i=0; i<cloud_s->points.size(); i++){
		cloud_s->points[i].x = 6*strArray[i][0];
		cloud_s->points[i].y = 6*strArray[i][1];
		cloud_s->points[i].z = 6*strArray[i][2];
	}
}




int main(int argc, char** argv){
	ros::init(argc, argv, "plane2");
	ros::Publisher pub;
    ros::Subscriber sub;
    ros::NodeHandle nh;
	tf::TransformListener listener;
	sensor_msgs::PointCloud2 cloud_msg;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_s (new pcl::PointCloud<pcl::PointXYZ>);

	ros::Rate rate(10);

	get2pcl ("/home/wuhang/tensorwork/pointcloudml/data/data_1_0.csv", cloud_s);
	pcl::toROSMsg(*cloud_s,cloud_msg);
	pub = nh.advertise<sensor_msgs::PointCloud2>("pointcluster", 10);
	cloud_msg.header.frame_id = std::string("/base_laser");

	while (ros::ok()){
		listener.waitForTransform(cloud_msg.header.frame_id,"/base_laser",ros::Time(0),ros::Duration(1.0));
		cloud_msg.header.stamp = ros::Time(0);
		pub.publish(cloud_msg);
		rate.sleep();
		ROS_INFO("Successfully publish pointcloud");
	}

    return (0);
}
