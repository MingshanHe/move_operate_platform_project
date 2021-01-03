#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import DisplayRobotState
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.msg import CollisionObject

from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from darknet_ros_msgs.msg import Object




#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>




class Manipulator:
    def __init__(self):
        # init settings
        self.group_name = "manipulator_i5"
        self.manipulator = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander(self.group_name)
        self.base = String()
        self.base = "world"
        self.arm.set_pose_reference_frame(self.base)
        self.arm.allow_replanning(True)
        self.arm.set_max_velocity_scaling_factor(0.2)      
        self.arm.set_max_acceleration_scaling_factor(0.2)
        self.arm.set_goal_orientation_tolerance(0.01)
        self.arm.set_goal_position_tolerance(0.02)
        self.arm.set_planning_time(5.0)
        # subscriber
        self.sub = rospy.Subscriber("/darknet_ros/Object", Object, self.callback)
        # params
        self.expect_position_x = 313.0025202958532/2
        self.expect_position_y = 219.3712132622622/2
        self.position_x_threshold = 5
     
    def callback(self, msg):
        goal_pose = self.get_current_state()
        Class = msg.Class
        error_x = msg.alpha
        if Class == "bottle":
            rospy.loginfo("U_position_x:%f", error_x)
            if abs(error_x) > self.position_x_threshold:
                if error_x > 0:
                    rospy.loginfo("Need right.")
                    goal_pose.position.y = goal_pose.position.y + 0.01
                else:
                    rospy.loginfo("Need left.")
                    goal_pose.position.y = goal_pose.position.y - 0.01
        rospy.loginfo("back.")

    def get_current_state(self):
        return self.arm.get_current_pose().pose


