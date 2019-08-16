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

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <math.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

using namespace std;
using namespace pcl;


class cloudFPFH
{
public:
cloudFPFH(){
	cloud_b.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    normals.reset(new pcl::PointCloud<pcl::Normal> ());
    fpfhs.reset(new pcl::PointCloud<pcl::FPFHSignature33> ());
    sub = nh.subscribe("pointcluster", 10, &cloudFPFH::computeFPFH, this);
    //pub = nh.advertise......
}

void computeFPFH (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){    
	cloud->clear();
	sensor_msgs::PointCloud cloud_msg_s1;
	sensor_msgs::PointCloud cloud_msg1;
	sensor_msgs::PointCloud2 cloud_msg_s;
	sensor_msgs::convertPointCloud2ToPointCloud(*cloud_msg,cloud_msg1);
    
	listener.transformPointCloud("/base_link", cloud_msg1, cloud_msg_s1);
	sensor_msgs::convertPointCloudToPointCloud2(cloud_msg_s1,cloud_msg_s);
	pcl::fromROSMsg(cloud_msg_s,*cloud);
    this-> computenormals (cloud, cloud_, normals, fpfhs);
    cout << "The fpfhs for index 1 (" << cloud->points[1].x <<
	    		" " << cloud->points[1].y << " " << cloud->points[1].z <<
				") is: "<< fpfhs->points[1] << endl;
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
	this->movecloud (cloud_, 'x', -centroid(0,0));
	this->movecloud (cloud_, 'y', -centroid(1,0));
	this->movecloud (cloud_, 'z', -centroid(2,0));
}

void computenormals (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_,
			pcl::PointCloud<pcl::Normal>::Ptr & normals, pcl::PointCloud<pcl::FPFHSignature33>::Ptr & fpfhs){
	pcl::search::Search<PointXYZ>::Ptr tree;
	cloud_->clear();
	*cloud_ = *cloud;
	this->normcloud(cloud_);
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
    fpfh.setSearchMethod (tree);
    //fpfh.setRadiusSearch (0.05);
    fpfh.setKSearch(40);
    fpfh.compute (*fpfhs);
}

private:
    Eigen::Vector4f centroid;
    Eigen::Matrix4f transMatrix;
    ros::Publisher pub;
    ros::Subscriber sub;
    //sensor_msgs::PointCloud2 cloud_send;
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr normals;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs; 
    tf::TransformListener listener;
};




int main(int argc, char** argv){
    ros::init(argc, argv, "plane");	
    cloudFPFH cloudfpfh;
    ros::spin();
    return (0);
}
