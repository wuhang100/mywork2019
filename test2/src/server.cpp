#include <ros/ros.h>
#include <test2/myservice.h>
#include <string>
#include <vector>  
#include <string> 
#include <fstream> 
#include <sstream>
#include <cstring>

bool handle_fun(test2::myservice::Request &req, test2::myservice::Response &res)
{
    std::ifstream myfile("/home/wuhang/projects/example.txt");
    res.sum = req.a+req.b;
    std::vector<std::string> temp; 
    std::string myline;
    std::int32_t c;
    if (!myfile.is_open()) 
    { 
        ROS_INFO("open fail");
    }     
    while(getline(myfile,myline)) 
    { 
        temp.push_back(myline);
    }
    myfile.close(); 
    c = stoi(temp[0]);
    res.sum = req.a+req.b+c;
    res.feedback = "My answer is: " + std::to_string(res.sum);
    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test2_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("mytestservice", handle_fun);
    ROS_INFO("server start");
    ros::spin();    
    return 0;
}


