#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <string>

int main(int argc, char **argv)
{
     ros::init(argc, argv, "number_publisher");

    ros::NodeHandle n;

    ros::Publisher number_pub = n.advertise<std_msgs::Int32>("Andarbayev_topic", 1000);

    ros::Rate loop_rate(100.0);

    std::string card_number = "201919031";
    size_t digit_index = 0;

    int count = 0;

    while (ros::ok())
    {
        std_msgs::Int32 msg;

        msg.data = card_number[digit_index] - '0';

        ROS_INFO("Publishing digit: %d", msg.data);

        number_pub.publish(msg);

        digit_index = (digit_index + 1) % card_number.length();

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
