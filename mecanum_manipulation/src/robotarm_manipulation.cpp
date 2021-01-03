#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float64.h>
#include <darknet_ros_msgs/Object.h>
#include <math.h>

class Manipulation
{
public:

void callback(const darknet_ros_msgs::Object &object)
{
    Manipulation::Class = object.Class;
    Manipulation::position_x = object.position_x;

    std::string s;
    s = "bottle";
    if(s == Class)
    {
        
        ROS_INFO("U_position_x:%f",position_x);
        
        pose_target = arm.getCurrentPose();
    
        start_pose2.orientation.w = pose_target.pose.orientation.w;
        start_pose2.position.x = pose_target.pose.position.x;
        start_pose2.position.y = pose_target.pose.position.y;
        start_pose2.position.z = pose_target.pose.position.z;

        float Error = Manipulation::position_x;

        //**判断**//
        if (Error > position_x_threshold)
        {
            if (Error > 0)
            {
                start_pose2.position.y -= 0.01;
                arm.setPoseTarget(start_pose2);
                arm.move();
            }
            else
            {
                start_pose2.position.x += 0.01;
                arm.setPoseTarget(start_pose2);
                arm.move();
            }
        }
        else if (Error < position_x_threshold)
        {
            
            arm.setPoseTarget(start_pose2);
            arm.move();
        }

    ROS_INFO("back");
}
public:
    Manipulation()
    {   
        visual_command_sub = nh_.subscribe("/darknet_ros/Object", 1, &Manipulation::callback, this);
        moveit::planning_interface::MoveGroupInterface arm("manipulator_i5");
        //允许误差
        arm.setGoalJointTolerance(0.05);
        //允许的最大速度和加速度
        arm.setMaxAccelerationScalingFactor(0.2);
        arm.setMaxVelocityScalingFactor(0.2);
        
        Manipulation::pose_target = arm.getCurrentPose();
        ROS_INFO("pose_target.pose.position.x:%f",pose_target.pose.position.x);
        start_pose2.orientation.w = pose_target.pose.orientation.w;
        start_pose2.position.x = pose_target.pose.position.x;
        start_pose2.position.y = pose_target.pose.position.y;
        start_pose2.position.z = pose_target.pose.position.z;
        Manipulation::expect_position_x = 0.0;
        Manipulation::position_x_threshold = 10;
    }
    ~Manipulation(){}
private:
ros::NodeHandle                                     nh_;
ros::Subscriber                                     visual_command_sub;
std::string                                         Class;

float                                               position_x;
float                                               expect_position_x;
float                                               position_x_threshold;

geometry_msgs::PoseStamped                          pose_target;
geometry_msgs::Pose                                 start_pose2;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "robotarm_manipulation");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    Manipulation manipulation;

    while(ros::ok())
    {
        //ROS_INFO("IN ROS OK.");
	    ros::spinOnce();
    }
    return 0;

}

