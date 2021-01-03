
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <vector>
#include <ros/ros.h>
#include <stdio.h>
#include <std_msgs/Float64.h>

class Manipulation
{
public:
void callback(const std_msgs::Float64 &msg)
    {
        float c = msg.data;
        start_pose2.orientation.x = pose_target.pose.orientation.x;
        start_pose2.orientation.y = pose_target.pose.orientation.y;
        start_pose2.orientation.z = pose_target.pose.orientation.z;
        start_pose2.orientation.w = pose_target.pose.orientation.w;
        start_pose2.position.x = pose_target.pose.position.x;
        start_pose2.position.y = pose_target.pose.position.y;
        start_pose2.position.z = pose_target.pose.position.z;
        moveit::planning_interface::MoveGroupInterface arm("manipulator_i5");
        ROS_INFO("In callback");
        if(c == 1)
        {
            ROS_INFO("You entered w. Position x UP!!");
            start_pose2.position.x += 0.05;
            arm.setPoseTarget(start_pose2);
            arm.move();
        }
        else if(c == 2)
        {
            ROS_INFO("You entered x. Position x DOWN!!");
            start_pose2.position.x -= 0.05;
            arm.setPoseTarget(start_pose2);
            arm.move();
        }
        else if(c == 3)
        {
            ROS_INFO("You entered a. Position y UP!!");
            start_pose2.position.y += 0.05;
            arm.setPoseTarget(start_pose2);
            arm.move();
        }
        else if(c == 4)
        {
            ROS_INFO("You entered d. Position y DOWN!!");
            start_pose2.position.y -= 0.05;
            arm.setPoseTarget(start_pose2);
            arm.move();
        }
        else if(c == 5)
        {
            ROS_INFO("You entered q. Position z UP!!");
            start_pose2.position.z += 0.05;
            arm.setPoseTarget(start_pose2);
            arm.move();
        }
        else if(c == 6)
        {
            ROS_INFO("Yes, you entered e. Position z DOWN!!");
            start_pose2.position.z -= 0.05;
            arm.setPoseTarget(start_pose2);
            arm.move();
        }
        else if(c == 7)
        {
            ROS_INFO("Yes, you entered s. STOP!!");
            pose_target = arm.getCurrentPose();
            arm.setPoseTarget(start_pose2);
            arm.move();
        }
        else if(c == 8)
        {
            arm.setNamedTarget("home");
            arm.move();
        }
        else if(c == 9)
        {
            arm.setNamedTarget("zero");
            arm.move();
        }
        else
        {
            ROS_INFO("Error Command, Please reinput a new Command.");
        }
        ROS_INFO("OUT callback");
}

public:
    Manipulation()
    {
        
        
        command_sub = nh_.subscribe("cmd_robotarm", 1, &Manipulation::callback, this);
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
    }
    ~Manipulation(){}
private:
ros::NodeHandle                                     nh_;
ros::Subscriber                                     command_sub;
geometry_msgs::PoseStamped                          pose_target;
geometry_msgs::Pose                                 start_pose2;
};
int main(int argc, char **argv)
{
    //初始化，其中ur_test02为节点名
    ros::init(argc, argv, "UR5_test1");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    Manipulation manipulation;
    while (ros::ok())
    {
        ros::spinOnce();
    }
    
    return 0;
}