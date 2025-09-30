#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <cmath> 

int main(int argc, char **argv){
    ros::init(argc, argv, "custom_publisher_snake_movement");

    ros::NodeHandle nh;

    ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 10);
    ros::Publisher pub3 = nh.advertise<std_msgs::Float64>("/robot/joint3_position_controller/command", 10);
    ros::Publisher pub4 = nh.advertise<std_msgs::Float64>("/robot/joint4_position_controller/command", 10);
    ros::Publisher pub5 = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 10);

    ros::Rate rate(50.0); 
    double t = 0.0;
    
   
    const double ampl = 1.5;  
    const double freq = 0.25; 
    
   
    const double phase_delay_per_joint = M_PI / 4.0; 
    
    std_msgs::Float64 joint_msg;

    ROS_INFO("Starting movement generation...");

    while(ros::ok()){
       
        t += rate.expectedCycleTime().toSec(); 

       

       
        joint_msg.data = ampl * std::sin(2.0 * M_PI * freq * t);
        pub1.publish(joint_msg);

       
        joint_msg.data = -ampl * std::sin(2.0 * M_PI * freq * t + 1.0 * phase_delay_per_joint);
        pub2.publish(joint_msg);

       
        joint_msg.data = ampl * std::sin(2.0 * M_PI * freq * t + 2.0 * phase_delay_per_joint);
        pub3.publish(joint_msg);

       
        joint_msg.data = -ampl * std::sin(2.0 * M_PI * freq * t + 3.0 * phase_delay_per_joint);
        pub4.publish(joint_msg);

       
        joint_msg.data = ampl * std::sin(2.0 * M_PI * freq * t + 4.0 * phase_delay_per_joint);
        pub5.publish(joint_msg);

       
        ros::spinOnce();

       
        rate.sleep();
    }

    return 0;
}
