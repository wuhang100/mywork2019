#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

class PointCluster
{
public:
    PointCluster(){
        ROS_INFO("Handle the point cloud");
        sub = nh.subscribe("pointcluster", 10, &PointCluster::cloud_cb, this);    
    }
 
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){      
        pcl::fromROSMsg(*cloud_msg,*cloud);
        this-> cloudhandle (cloud);
    }

    void cloudhandle (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
        std::cout << "I receive the cloud: "<<cloud->points.size ()<< std::endl;
    }
 
private:
    ros::Publisher pub;
    ros::Subscriber sub;
    sensor_msgs::PointCloud2 cloud_send;
    ros::NodeHandle nh;
    tf::TransformListener listener;
    //pcl::PointCloud<pcl::PointXYZ> cloud;
};
 
int main(int argc, char **argv){
    ros::init (argc, argv, "pointhandle");
    PointCluster pointHandle;
    ros::spin();
    return (0);
}