#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <darknet_ros_msgs/Object.h>
#include <math.h>
class Manipulation
{
public:
Manipulation(){
    P_alpha = 0.01;
    I_alpha = 0.0;
    D_alpha = 0.0;

    P_area = 0.000004;
    I_area = 0.0;
    D_area = 0.0;

    Expect_area = 300*250;
    Expect_alpha = 0.0;

    Integral_area = 0.0;
    Integral_alpha = 0.0;

    Error_alpha_previous = 0.0;
    Error_area_previous = 0.0;

    area_threshold_up = 0.8;
    area_threshold_down = 0.01;
    angular_threshold_up = 0.8;
    angular_threshold_down = 0.01;
}

~Manipulation(){}

geometry_msgs::Twist command_(float linear_x, float linear_y, float linear_z, float angular_z)
{
    geometry_msgs::Twist Twist_;
    Twist_.linear.x = linear_x;
    Twist_.linear.y = linear_y;
    Twist_.linear.z = linear_z;
    Twist_.angular.z = angular_z;
    return Twist_;
}
void callback(const darknet_ros_msgs::Object &object)
{
    Class = object.Class;
    std::string s;
    s = "person";
    if(s == Class)
    {
        ROS_INFO("Yes, it is a person");
        len_x = object.len_x;
        len_y = object.len_y;
        position_x = object.position_x;
        position_y = object.position_y;
        alpha = object.alpha;
        area = len_x * len_y;

        //控制器从传感器得到测量结果，然后用需求结果减去测量结果来得到误差
        Error_alpha = Expect_alpha - alpha;
        Integral_alpha += Error_alpha;

        Error_area = Expect_area - area;
        Integral_area += Error_area;
    
        //角度PID输出
        U_alpha = P_alpha*Error_alpha + I_alpha*Integral_alpha + D_alpha*(Error_alpha - Error_alpha_previous);
        Error_alpha_previous = Error_alpha;
        //面积PID输出
        U_area = P_area*Error_area + I_alpha*Integral_area + D_area*(Error_area - Error_area_previous);
        Error_area_previous = Error_area;

        //可以添加一定死区

        //发布
        geometry_msgs::Twist Twist_;
        //Twist_ = command_(U_area, 0.0, 0.0, U_alpha);
        ROS_INFO("U_alpha:%f",U_alpha);
        if (fabs(U_alpha) < angular_threshold_up && fabs(U_alpha) > angular_threshold_down)
        {
            ROS_INFO("In Threshold, Pub the result of PID");
            U_alpha = U_alpha;
        }
        else if (abs(U_alpha) < angular_threshold_down)
        {
            ROS_INFO("Bellow Threshold, Pub the stop");
            U_alpha = 0.0;
        }
        else
        {
            ROS_INFO("Over Threshold, Pub the Max");
            U_alpha = angular_threshold_up;
        }
        ROS_INFO("U_area:%f",U_area);
        if (fabs(U_area) < area_threshold_up && fabs(U_area) > area_threshold_down)
        {
            ROS_INFO("In Threshold, Pub the result of PID");
            Twist_ = command_(U_area, 0.0, 0.0, U_alpha);
        }
        else if (abs(U_area) < area_threshold_down)
        {
            ROS_INFO("Bellow Threshold, Pub the stop");
            Twist_ = command_(0.0, 0.0, 0.0, U_alpha);
        }
        else
        {
            ROS_INFO("Over Threshold, Pub the Max");
            Twist_ = command_(area_threshold_up, 0.0, 0.0, U_alpha);
        }
        command_pub.publish(Twist_);
        ros::spinOnce();
    }
    ROS_INFO("back");
}

private:
ros::NodeHandle             nh_;
ros::Publisher              command_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
ros::Subscriber             visual_command_sub = nh_.subscribe("/darknet_ros/Object",1,&callback, this);

float                       P_alpha;
float                       I_alpha;
float                       D_alpha;

float                       P_area;
float                       I_area;
float                       D_area;


float                       len_x;
float                       len_y;
float                       position_x;
float                       position_y;
float                       alpha;
float                       area;

float                       Expect_alpha;
float                       Expect_area;

float                       Error_alpha;
float                       Error_alpha_previous;
float                       Integral_alpha;

float                       Error_area;
float                       Error_area_previous;
float                       Integral_area;

float                       U_alpha;
float                       U_area;
std::string                 Class;

float                       area_threshold_up;
float                       area_threshold_down;
float                       angular_threshold_up;
float                       angular_threshold_down;
};

int main(int argc, char **argv){

    ros::init(argc, argv, "Nexus_move_test");
    Manipulation manipulation;
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        //ROS_INFO("IN ROS OK.");
	ros::spinOnce();
    }
    return 0;

}

