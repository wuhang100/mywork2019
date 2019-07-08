#include <ros/ros.h>
#include "laser_geometry/laser_geometry.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

int Pt_generate(sensor_msgs::LaserScan& msg){
    float ranges[100];
    float intensities[100];
    float length = 25.0;
    //msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_laser";
    msg.angle_min = -1.57;
    msg.angle_max = 1.57;
    msg.angle_increment = 3.14 / 100;
    msg.time_increment = (1 / 40) / 100;
    msg.range_min = 0.2;
    msg.range_max = 100.0;
    for(unsigned int i = 0; i < 100; i++){
      ranges[i] = length;
      intensities[i] = 10*i;
    }
    msg.ranges.resize(100);
    msg.intensities.resize(100);
    for(unsigned int i = 0; i < 100; ++i){
      msg.ranges[i] = ranges[i];
      msg.intensities[i] = intensities[i];
    }
    return 0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pointpipe");
    ros::NodeHandle nh;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("pointmessage", 100);
    ros::Rate rate(10);
    
    sensor_msgs::PointCloud2 cloud;
    sensor_msgs::LaserScan msg;

    while (ros::ok()){
    Pt_generate(msg);
    listener.waitForTransform(msg.header.frame_id,"/base_link",\
                              ros::Time(0),\
                              ros::Duration(1.0));
    msg.header.stamp = ros::Time(0);
    projector.transformLaserScanToPointCloud("/base_link",msg,cloud,listener);
    pub.publish(cloud);
    rate.sleep();
    ROS_INFO("Successfully publish pointcloud");
    }
    return 0;
}
