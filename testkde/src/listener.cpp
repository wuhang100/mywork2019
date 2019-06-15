//ROS头文件
#include <ros/ros.h>
//包含自定义msg产生的头文件
#include <testkde/gps.h>
//ROS标准msg头文件
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sstream>

void gpsCallback(const testkde::gps::ConstPtr &msg) //这里传入参数是一个指针
{  
    //计算离原点(0,0)的距离
    std_msgs::Float32 distance;
    std_msgs::Int32 num;
    distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
    num.data = msg->num;
    //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
    ROS_INFO("Listener: Distance to origin = %f, state: %s, number: %d",distance.data,msg->state.c_str(),num.data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ROS_INFO("Node listener start");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gps_infok", 1, gpsCallback);
  //ros::spin()用于调用所有可触发的回调函数。将进入循环，不会返回，类似于在循环里反复调用ros::spinOnce()。
  ros::spin(); 
  return 0;
}
