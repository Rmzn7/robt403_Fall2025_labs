#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "custom_publisher_square_wave");

    ros::NodeHandle nh;
    ros::Publisher pub_j1;
    ros::Publisher pub_j5;
    pub_j5 = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 10);
    pub_j1 = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
    ros::Rate rate(10.0);
    bool high = false;
    double low = 0.0;
    double high_val = 1.57;
    ros::Time last_switch = ros::Time::now();
    const double period = 0.5;
    
while(ros::ok()){
    ros::Time now = ros::Time::now();
    if((now-last_switch).toSec() > period){
        high = !high;
        last_switch = now;
    }
    std_msgs::Float64 msg;
    msg.data = high ? high_val : low;
    pub_j1.publish(msg);
    pub_j5.publish(msg);
}   

    return 0;
}
