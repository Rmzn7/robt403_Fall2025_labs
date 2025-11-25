//#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
// Main moveit libraries are included
int main(int argc, char **argv){

ros::init(argc, argv, "move_group_interface_tutorial");
ros::NodeHandle node_handle;
ros::AsyncSpinner spinner(0);

spinner.start(); // For moveit implementation we need AsyncSpinner, we cant use ros::spinOnce()

static const std::string PLANNING_GROUP = "move_group"; /* Now we specify with what group we want work,
here group1 is the name of my group controller*/

moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); // loading move_group

const robot_state::JointModelGroup *joint_model_group =

move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //For joint control


geometry_msgs::PoseStamped target_pose = move_group.getCurrentPose(); /* Retrieving the information about the
current position and orientation of the end effector*/

ros::Rate loop_rate(50); //Frequency

const double SIZE = 0.5;


ros::Duration(1.0).sleep();

    target_pose.pose.position.x -= 1.5*SIZE;
    move_group.setApproximateJointValueTarget(target_pose); // To calculate the trajectory
    move_group.setStartStateToCurrentState();
    move_group.move(); // Move the robot

    target_pose = move_group.getCurrentPose(); 
    target_pose.pose.position.y +=SIZE;
    move_group.setApproximateJointValueTarget(target_pose); // To calculate the trajectory
    move_group.setStartStateToCurrentState();
    move_group.move(); // Move the robot

    target_pose = move_group.getCurrentPose();
    target_pose.pose.position.x +=1.5*SIZE;
    move_group.setApproximateJointValueTarget(target_pose); // To calculate the trajectory
    move_group.setStartStateToCurrentState();
    move_group.move(); // Move the robot

    target_pose = move_group.getCurrentPose();
    target_pose.pose.position.y -=SIZE;
    move_group.setApproximateJointValueTarget(target_pose); // To calculate the trajectory
    move_group.setStartStateToCurrentState();
    move_group.move(); // Move the robot

ROS_INFO("Done");
ros::shutdown();
return 0;
}