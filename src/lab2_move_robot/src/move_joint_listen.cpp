#include <ros/ros.h>
#include <std_msgs/Float64.h>

class CustomPublisher{
    public: 
        CustomPublisher(): prev_value(0.0){
            sub = nh.subscribe("/input_command", 10, &CustomPublisher::callback, this);
            pub = nh.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
            ROS_INFO("Started listening to /input_command and will publish to the joint");
        }
    private:
        void callback(const std_msgs::Float64::ConstPtr& msg){
            if(msg->data > prev_value){
                std_msgs::Float64 cmd;
                cmd.data = msg->data;
                pub.publish(cmd);
                ROS_INFO("Published %f, previous: %f", cmd.data, prev_value);
            } else {
            ROS_INFO("Ignored %f, previous %f", msg->data, prev_value);
            }
                        prev_value = msg->data;

        }
            ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    double prev_value;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "custom_publisher");
    CustomPublisher node;
    ros::spin();
    return 0;
}
