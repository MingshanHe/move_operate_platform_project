#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>
#define PI 3.1415926
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_estimate_2d");
  ros::NodeHandle nh;
  ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  ros::Rate loop_rate(1);
  //define 2d estimate pose
  double alpha = PI/2;//radian value
  double x_pos = -1.9849998951;
  double y_pos = -0.529999732971;
 
  while (ros::ok())
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
 
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = x_pos;
    pose_msg.pose.pose.position.y = y_pos;
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[6 * 1 + 1] = 0.25;
    pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942;
    pose_msg.pose.pose.orientation.z = -0.00289018648695;
    pose_msg.pose.pose.orientation.w = 0.999995823402;
 
    initial_pose_pub.publish(pose_msg);
    ROS_INFO("Setting to :(%f,%f)",x_pos,y_pos);
    ros::spinOnce();
 
    loop_rate.sleep();
  }
 
  return 0;
}