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
#include <pcl/features/fpfh.h>
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
pcl::PointCloud<pcl::Normal>::Ptr pointnormal_pfh (new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
//pcl::PointCloud<pcl::Normal>::Ptr normal_fpfh (new pcl::PointCloud<pcl::Normal>);

class PointCluster
{
public:

    void generateCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
		            std::int32_t width, std::int32_t height )
    {
		cloud->width  = width;
		cloud->height = height;
		cloud->points.resize (cloud->width * cloud->height);
		float j = 0.2; float x_base = 2.0; float y_base = 2.0; float z_base;
		for (std::int32_t i = 0; i < cloud->points.size (); i++){
			if (i%width == 0){
				x_base = 0;
				y_base = y_base + 0.5;
				z_base = sin(j);
				j = j + 0.2;
		        }
		    cloud->points[i].x = x_base;
		    cloud->points[i].y = y_base;
		    cloud->points[i].z = 2*z_base;
		    x_base = x_base + 0.5;
		    //std::cout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
			}

    }

    PointCluster(){
        ROS_INFO("Handle the point cloud");
        width = 100; height = 20; search_width = 15; search_height = 15;
        sub = nh.subscribe("pointcluster", 10, &PointCluster::cloud_cb, this);
        pointnum_ptr = boost::make_shared<std::vector<int>>(search_pointnum);  
    }
 
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){      
        pcl::fromROSMsg(*cloud_msg,*cloud);
        //generateCloud (cloud, 100, 20);
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

    void setfpfh (boost::shared_ptr<std::vector<int>> pointnum_ptr, \
		         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f, \
		         pcl::PointCloud<pcl::Normal>::Ptr pointnormal_pfh, \
		         pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){
	    pointnormal_pfh-> clear();
        ne.setInputCloud(cloud_f);
        ne.setRadiusSearch (30);
        ne.setSearchMethod (tree);
        ne.compute (*pointnormal_pfh);
        fpfh.setInputCloud (cloud_f);
        fpfh.setInputNormals (pointnormal_pfh);
        fpfh.setIndices (pointnum_ptr);
        fpfh.setSearchMethod (tree);
        //fpfh.setRadiusSearch (3.5f);
        fpfh.setKSearch(16);
        fpfh.compute (fpfhs);
        /*for (int i = 0; i < pointnormal_pfh->points.size(); i++)
        {
        if (!pcl::isFinite<pcl::Normal>(pointnormal_pfh->points[i]))
        {
            PCL_WARN("normals[%d] is not finite\n", i);
        }
        }*/
        fpfhs_sum += fpfhs;
        fpfhs.clear();
    }

    void viewnormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
	                 pcl::PointCloud<pcl::Normal>::Ptr normals,\
	                 std::int32_t search_width, std::int32_t search_height,\
	                 std::vector<int> search_pointnum,\
	                 pcl::search::KdTree<pcl::PointXYZ>::Ptr tree,\
	                 pcl::PointCloud<pcl::Normal>::Ptr pointnormal,\
				     pcl::PointCloud<pcl::Normal>::Ptr pointnormal_pfh,\
	                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f){

	    for (std::int32_t i = 0; i < cloud->points.size(); i++){
    	        search_point = i;
                search_indices.clear();
    	        this-> setindices (search_point, search_indices,cloud,search_width,search_height);
    	        indices_ptr = boost::make_shared<std::vector<int>>(search_indices);
    	        pcl::ExtractIndices<pcl::PointXYZ> extract;
                cloud_fi.clear();
    	        extract.setInputCloud(cloud);
    	        extract.setIndices (indices_ptr);
    	        extract.filter (cloud_fi);
    	        cloud_f=cloud_fi.makeShared();
                this-> setfpfh (pointnum_ptr, cloud_f, pointnormal_pfh, tree);
	    }

    }


    void setfpfh_example (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
					      pcl::PointCloud<pcl::Normal>::Ptr normals, \
					      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree){

        normals->clear();
	    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	    normalEstimation.setInputCloud(cloud);
	    normalEstimation.setRadiusSearch(30);
	    normalEstimation.setSearchMethod(tree);
	    normalEstimation.compute(*normals);

	    fpfh.setInputCloud(cloud);
	    fpfh.setInputNormals(normals);
	    fpfh.setSearchMethod(tree);
	    // Search radius, to look for neighbors. Note: the value given here has to be
	    // larger than the radius used to estimate the normals.
	    fpfh.setRadiusSearch(35);
	    //fpfh.setKSearch(10);
	    fpfh.compute(fpfhs);
	    fpfhs_sum = fpfhs;
        fpfhs.clear();
    }



    void cloudhandle (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
        std::cout << "I receive the cloud: "<<cloud->points.size ()<< std::endl;
        //cloud->width  = width;
		//cloud->height = height;
		//cloud->points.resize (cloud->width * cloud->height);
        //this-> viewPoint(cloud, normals);
	    //this-> viewnormals(cloud, normals, search_width, search_height, search_pointnum, tree, pointnormal, pointnormal_pfh, cloud_f);
        this->setfpfh_example(cloud,normals,tree);
        //for (int i=0; i<fpfhs_sum.points.size();i++){
		    std::cout << "The fpfhs for index is " << fpfhs_sum.points[1999].histogram[32] << std::endl;
	    //}
    }
    
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    sensor_msgs::PointCloud2 cloud_send;
    ros::NodeHandle nh;
    std::vector<int> search_pointnum {0};
	std::int32_t width, height, search_width, search_height;
    std::int32_t search_point;

    boost::shared_ptr<std::vector<int>> pointnum_ptr;
    
    pcl::PointCloud<pcl::Normal> normal_empty;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    pcl::PointCloud<pcl::FPFHSignature33> fpfhs;
    pcl::PointCloud<pcl::FPFHSignature33> fpfhs_sum;
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_fi;
    std::vector<int> search_indices;
    boost::shared_ptr<std::vector<int>> indices_ptr;
};
 
int main(int argc, char **argv){
    ros::init (argc, argv, "pointhandle");
    PointCluster pointHandle;
    ros::spin();
    return (0);
}