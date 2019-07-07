#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

int Pt_generate(sensor_msgs::LaserScan& msg){
    float ranges[100];
    float intensities[100];
    static float lengh = 5;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "base_laser";
    msg.angle_min = -1.57;
    msg.angle_max = 1.57;
    msg.angle_increment = 3.14 / 100;
    msg.time_increment = (1 / 40) / 100;
    msg.range_min = 0.2;
    msg.range_max = 100.0;
    for(unsigned int i = 0; i < 100; i++){
      ranges[i] = lengh;
      intensities[i] = 10*i;
    }
    msg.ranges.resize(100);
    msg.intensities.resize(100);
    for(unsigned int i = 0; i < 100; ++i){
      msg.ranges[i] = ranges[i];
      msg.intensities[i] = intensities[i];
    }
    lengh = lengh + 0.5;
    return 0;
}



int main (int argc, char **argv){
    ros::init(argc, argv, "Pt_generate");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::LaserScan>("Points", 1000);
    ros::Rate rate(10);
    ROS_INFO("Successfully start broadcaster");
    sensor_msgs::LaserScan msg;
    
    while (ros::ok()){
    Pt_generate(msg);
    pub.publish(msg);
    rate.sleep();
    }
    
    return 0;
}