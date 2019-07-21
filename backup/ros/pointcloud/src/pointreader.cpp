#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

void generateCloud (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, \
		std::int32_t width)
{
    cloud->width  = width;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size (); ++i)
	{
        cloud->points[i].x = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
	    cloud->points[i].y = 2 + 1024 * rand () / (RAND_MAX + 1.0f);
	    cloud->points[i].z = 2 + 1024 * rand () / (RAND_MAX + 1.0f) / 100;
	}
}

int main (int argc, char** argv){
   // Generate message
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   //pcl::io::loadPCDFile ("/home/wuhang/pclfile/table_scene_mug_stereo_textured.pcd", *cloud);
   generateCloud (cloud, 20);
   ROS_INFO("Generate a point cloud");
   sensor_msgs::PointCloud2 pointmsg;
   pcl::toROSMsg(*cloud,pointmsg);
   
   // Initialize ROS
   ros::init (argc, argv, "pointreader");
   ros::NodeHandle nh;
   ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("cloudonline", 1);
   ros::Rate loop_rate(1.0);
   std::int16_t count;

   while (ros::ok())
   {
   ROS_INFO("Pubish point cloud %d", count);
   pub.publish(pointmsg);
   loop_rate.sleep();
   count++;
   }
   
   return (0);
}

