#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>\

 
class PointCluster
{
public:
    PointCluster(){
    ROS_INFO("Handle the point cloud");
    cloud_send.header.frame_id = std::string("base_laser");
    pub = nh.advertise<sensor_msgs::PointCloud2>("pointcluster", 10);
    sub = nh.subscribe("pointmessage", 10, &PointCluster::cloudconcentrate, this);
    
    }
 
    void cloudconcentrate (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    //If using pcl::PCLPointCloud2
    //pcl::PCLPointCloud2 cloud2;
    //pcl_conversions::toPCL(*cloud_msg,cloud2);
    //pcl::fromPCLPointCloud2(cloud2,*cloud);
    pcl::fromROSMsg(*cloud_msg,cloud);
    cloud_con +=  cloud;
    ROS_INFO("The size is %lud", cloud_con.points.size ());
    pcl::toROSMsg(cloud_con,cloud_send);
    cloud_send.header.frame_id = std::string("base_laser");
    //std::cout << cloud_send.header.frame_id << std::endl;
    //listener.waitForTransform("base_laser","base_link",\
                              ros::Time(0),\
                              ros::Duration(1.0));
    cloud_send.header.stamp = ros::Time(0);
    pub.publish(cloud_send);
    }
 
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud_send;
    ros::NodeHandle nh;
    tf::TransformListener listener;
    pcl::PointCloud<pcl::PointXYZ> cloud_con;
};
 
int main(int argc, char **argv){
    ros::init (argc, argv, "cloudconcentrate");
    PointCluster pointHandle;
    ros::spin();
    return (0);
}