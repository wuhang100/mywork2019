#include <ros/ros.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>

//ros::Publisher pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void cloudhandle (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    std::cout << "I receive the cloud: "<<cloud->points.size ()<< std::endl;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    //If using pcl::PCLPointCloud2
    //pcl::PCLPointCloud2 cloud2;
    //pcl_conversions::toPCL(*cloud_msg,cloud2);
    //pcl::fromPCLPointCloud2(cloud2,*cloud);
    pcl::fromROSMsg(*cloud_msg,*cloud);
    cloudhandle (cloud);
    ROS_INFO("Handle the point cloud");
}


int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "cloudhandle");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("pointmessage", 10, cloud_cb);

    // Create a ROS publisher for the output point cloud
    //pub = nh.advertise<sensor_msgs::PointCloud2> ("cloudresult", 1);

    // Spin
    ros::spin ();
}