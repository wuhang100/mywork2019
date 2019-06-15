#include <ros/ros.h>
#include <test2/myservice.h>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv ,"test2_client");
    if (argc != 3)
    {
        ROS_INFO("number not match");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<test2::myservice>("mytestservice");
    test2::myservice service_back;
    service_back.request.a = atol(argv[1]);
    service_back.request.b = atol(argv[2]);
    
    if(client.call(service_back)){     //返回handle函数的布尔输出
        //ROS_INFO("%s : %ld",service_back.response.feedback.c_str(), service_back.response.sum);
        ROS_INFO("%s",service_back.response.feedback.c_str());
    }
    else{
        ROS_ERROR("There is an error");
        return 1;
    }
    return 0;
}
