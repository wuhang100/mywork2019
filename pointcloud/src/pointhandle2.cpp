#include <ros/ros.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <pcl/io/io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/pfh.h>
#include <pcl/search/organized.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/filters/filter.h>
#include <pcl/pcl_macros.h>
#include <pcl/filters/extract_indices.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::Normal>::Ptr pointnormal (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

class PointCluster
{
public:
    PointCluster(){
        ROS_INFO("Handle the point cloud");
        width = 100; height = 20; search_width = 5; search_height = 5;
        sub = nh.subscribe("pointcluster", 10, &PointCluster::cloud_cb, this);    
    }
 
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){      
        pcl::fromROSMsg(*cloud_msg,*cloud);
        this-> cloudhandle (cloud);
    }

    void setNormal1 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		             pcl::PointCloud<pcl::Normal>::Ptr normals){
        pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        //ne.setRectSize(5,5);
        ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
        ne.setMaxDepthChangeFactor(3.0f);
        ne.setNormalSmoothingSize(5.0f);
        ne.setInputCloud(cloud);
        ne.compute(*normals);
    }

    void setNormal2 (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		             pcl::PointCloud<pcl::Normal>::Ptr normals, \
		             pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod (tree);
        //ne.setIndices();
        ne.setRadiusSearch (1);
        ne.compute (*normals);
    }

    void viewPoint (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, \
		            pcl::PointCloud<pcl::Normal>::Ptr normals){
        if (!cloud->isOrganized ()){
            ROS_INFO("The cloud is not organized");
    	    this-> setNormal2 (cloud, normals, tree);}
        else{
            ROS_INFO("The cloud is organized");
    	    this-> setNormal1 (cloud, normals);}
    }

    void setindices (std::int32_t search_point, std::vector<int>& search_indices,\
		             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,\
		             std::int32_t search_width, std::int32_t search_height){
	    search_indices.push_back(search_point);
	    std::int32_t i_x = search_point / cloud->width;
	    std::int32_t i_y = search_point % cloud->width;
	    //std::cout << "i_x is " << i_x <<" i_y is " << i_y << std::endl;
	    std::int32_t i_n;
	    for (std::int32_t j = i_x - search_height/2; j <= i_x + search_height/2; j++){
		    for (std::int32_t k = i_y - search_width/2; k <= i_y + search_width/2; k++){
			    //std::cout << "search point " << j <<" " << k << " ";
			    if (j>=0 && j<cloud->height && k>=0 && k<cloud->width &&!(j==i_x && k==i_y)){
			        search_indices.push_back(j* cloud->width + k);
			        //std::cout << "valid " << j* cloud->width + k;
			    }
			//std::cout << std::endl;
		    }
	    }
    }

    void setNormal3 (boost::shared_ptr<std::vector<int>> pointnum_ptr, \
		             pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_f, \
		             pcl::PointCloud<pcl::Normal>::Ptr pointnormal, \
		             pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud_f);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (5);
        ne.setIndices (pointnum_ptr);
        ne.compute (*pointnormal);
    }

    void viewnormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
		             pcl::PointCloud<pcl::Normal>::Ptr normals,\
		             std::int32_t search_width, std::int32_t search_height,\
		             std::vector<int> search_pointnum,\
		             pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,\
		             pcl::PointCloud<pcl::Normal>::Ptr pointnormal,\
		             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f){

	    for (std::int32_t i = 0; i < normals->points.size (); i++){
            if (pcl_isnan(normals->points[i].normal_x)){
    	        search_point = i;
    	        pcl::PointCloud<pcl::PointXYZ> cloud_fi;
    	        std::vector<int> search_indices;
    	        this-> setindices (search_point, search_indices,cloud,search_width,search_height);
    	        boost::shared_ptr<std::vector<int>> indices_ptr = boost::make_shared<std::vector<int>>(search_indices);
    	        boost::shared_ptr<std::vector<int>> pointnum_ptr = boost::make_shared<std::vector<int>>(search_pointnum);
    	        pcl::ExtractIndices<pcl::PointXYZ> extract;
    	        extract.setInputCloud(cloud);
    	        extract.setIndices (indices_ptr);
    	        extract.filter (cloud_fi);
    	        cloud_f=cloud_fi.makeShared();
    	        this-> setNormal3 (pointnum_ptr, cloud_f, pointnormal, tree);
    	        normals->points[i].normal_x = pointnormal->points[0].normal_x;
    	        normals->points[i].normal_y = pointnormal->points[0].normal_y;
    	        normals->points[i].normal_z = pointnormal->points[0].normal_z;
                }
            
            std::cout << " ( " << i / cloud->width + 1 << " , "<< i % cloud->width + 1 << "): "
            << " Points: (" << cloud->points[i].x << ","
		    << cloud->points[i].y << ","
		    << cloud->points[i].z << ")"
		    <<"  Normals: (" << normals->points[i].normal_x << ","
		    << normals->points[i].normal_y << ","
		    << normals->points[i].normal_z << ")"
            << std::endl;
	    }

    }

    void cloudhandle (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
        std::cout << "I receive the cloud: "<<cloud->points.size ()<< std::endl;
        cloud->width  = width;
		cloud->height = height;
		cloud->points.resize (cloud->width * cloud->height);
        this-> viewPoint(cloud, normals);
	    this-> viewnormals(cloud, normals, search_width, search_height, search_pointnum, tree, pointnormal, cloud_f);
    }
    
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    sensor_msgs::PointCloud2 cloud_send;
    ros::NodeHandle nh;
    std::vector<int> search_pointnum {0};
	std::int32_t width, height, search_width, search_height;
    std::int32_t search_point;

};
 
int main(int argc, char **argv){
    ros::init (argc, argv, "pointhandle2");
    PointCluster pointHandle;
    ros::spin();
    return (0);
}