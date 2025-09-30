#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "math.h"

int main(int argc, char **argv){
ros::init(argc, argv, "custom_publisher_sin_wave");

ros::NodeHandle nh;
ros::Publisher pub1 = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
ros::Publisher pub5 = nh.advertise<std_msgs::Float64>("/robot/joint5_position_controller/command", 10);



ros::Rate rate(10.0);
double t = 0.0;
double ampl = 2;
double freq = 0.1;
std_msgs::Float64 sin_msg;
while(ros::ok()){
t += rate.expectedCycleTime().toSec(); 
sin_msg.data = ampl * sin(2 * M_PI * freq * t);
pub1.publish(sin_msg);
sin_msg.data = -sin_msg.data;
pub5.publish(sin_msg);
ros::spinOnce();
rate.sleep();
}
return 0;
}