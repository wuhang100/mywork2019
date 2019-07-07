#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int get_twist(tf::Transform& transform){
    static float i[3] = {3.0,3.0,1.0};
    static float theta = 0.0;
    transform.setOrigin(tf::Vector3(i[0],i[1],i[2]));
    tf::Quaternion q;
    q.setRPY(0,theta,0);
    transform.setRotation(q);
    //i[0] = i[0]+0.1;
    //i[1] = i[1]+0.02;
    theta = theta + 0.01;
    return 0;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle nh;
    ros::Rate r(10);
    tf::TransformBroadcaster broadcaster; 
    tf::Transform transform;
    ROS_INFO("Successfully start broadcaster");
    while(nh.ok()){
        get_twist(transform);
        broadcaster.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"base_link","base_laser"));
        r.sleep();
    }
}