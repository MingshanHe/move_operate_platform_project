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
//moveit::planning_interface::MoveGroupInterface arm("manipulator_i5");
std::string                                         Class;

float                                               position_x;
float                                               expect_position_x;
float                                               position_x_threshold;
float                                               error_x;

float                                               position_y;
float                                               expect_position_y;
float                                               position_y_threshold;
float                                               error_y;


int flag = 0;
geometry_msgs::PoseStamped                          pose_target;
geometry_msgs::Pose                                 start_pose2;

class Manipulation
{
public:
void callback(const darknet_ros_msgs::Object &object)
{
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    float expect_position_x = 313.0025202958532/2;
    float expect_position_y = 219.3712132622622/2;

    position_x_threshold = 5;
    position_y_threshold = 10;
    ROS_INFO("In callback");

    Class = object.Class;
    position_x = object.alpha;
    std::string s;
    ROS_INFO("%s",Class);
    s = "bottle";
    if(s == Class)
    {
        
        ROS_INFO("U_position_x:%f",position_x);

        error_x = object.alpha;
        error_y = object.beta;
        //**判断**//
        //重心在左边，y+ 重心在上面，z-
        ROS_INFO("error_x:%f",error_x);
        ROS_INFO("error_y:%f",error_y);
        
        if (fabs(error_x) > position_x_threshold || fabs(error_y) > position_y_threshold)
        {
            if (error_x > 0)
            {
                ROS_INFO("need right");
                start_pose2.position.y = start_pose2.position.y + 0.005;
            }
            else
            {
                ROS_INFO("need left");

                start_pose2.position.y = start_pose2.position.y - 0.005;

            }
            if (error_y >0)
            {
                ROS_INFO("nedd down");
                start_pose2.position.x = start_pose2.position.x + 0.005;
            }
            else
            {
                ROS_INFO("nedd up");
                start_pose2.position.x = start_pose2.position.x - 0.005;
            }
        }
        else
        {
            // if(flag == 1)
            // {
            // start_pose2.position.z = 1.3;
            // }
            // else{
                start_pose2.position.z = 0.618129;
    
        }
    }
    //ros::spin();

    ROS_INFO("back");
}

public:
    Manipulation()
    {
        command_sub = nh_.subscribe("/darknet_ros/Object", 1, &Manipulation::callback, this);
    }
    ~Manipulation(){}
private:
ros::NodeHandle                                     nh_;
ros::Subscriber                                     command_sub;

};
int main(int argc, char **argv)
{
    //初始化，其中ur_test02为节点名
    ros::init(argc, argv, "UR5_test1");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator_i5");
        //允许误差
    arm.setGoalJointTolerance(0.05);
        //允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);
        
    pose_target = arm.getCurrentPose();
    ROS_INFO("pose_target.pose.position.x:%f",pose_target.pose.position.x);
    ROS_INFO("pose_target.pose.position.y:%f",pose_target.pose.position.y);
    ROS_INFO("pose_target.pose.position.z:%f",pose_target.pose.position.z);
    start_pose2.orientation.w = pose_target.pose.orientation.w;
    start_pose2.orientation.x = pose_target.pose.orientation.x;
    start_pose2.orientation.y = pose_target.pose.orientation.y;
    start_pose2.orientation.z = pose_target.pose.orientation.z;
    start_pose2.position.x = pose_target.pose.position.x;
    start_pose2.position.y = pose_target.pose.position.y;
    start_pose2.position.z = pose_target.pose.position.z;

    Manipulation manipulation;

    while(ros::ok()){
        ros::spinOnce();
        arm.setPoseTarget(start_pose2);
        arm.move();
    }
    
    return 0;
}