#pragma once
/*
 * File: baxter_moveit_controller.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef BAXTER_MOVEIT_CONTROLLER_H_
#define BAXTER_MOVEIT_CONTROLLER_H_

#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <baxter_core_msgs/EndpointState.h>
// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

class BaxterMoveitController
{
public:
    BaxterMoveitController(ros::NodeHandle nh);
    ~BaxterMoveitController();

    // move Baxter's both arms to neutral position
    int moveToNeutral();

    // move Baxter's hands to..
    int moveLeftHandTo(geometry_msgs::Point set_point);
    int moveLeftHandTo(geometry_msgs::Pose set_pose);
    int moveRightHandTo(geometry_msgs::Point set_point);
    int moveRightHandTo(geometry_msgs::Pose set_pose);

    // move to joint positions
    int moveRightToJointPositions(double set_positions[]);
    int moveLeftToJointPositions(double set_positions[]);

    // for adding/removing objects
    int addObjectIntoWorld(moveit_msgs::CollisionObject object);
    int removeObjectFromWorld(std::string object_id);

    // set planning time
    int setPlanningTime(std::string arm, double time);

private:
    ros::NodeHandle nh;

    // interfaces for control with MoveIt
    move_group_interface::MoveGroup *both_arms_move_group;
    move_group_interface::MoveGroup *left_arm_move_group;
    move_group_interface::MoveGroup *right_arm_move_group;
    // for display in Rviz
    move_group_interface::PlanningSceneInterface plan_scene_interface;
    // motion plans
    move_group_interface::MoveGroup::Plan both_arms_plan;
    move_group_interface::MoveGroup::Plan left_arm_plan;
    move_group_interface::MoveGroup::Plan right_arm_plan;

    // visualizing plans in Rviz
    moveit_msgs::DisplayTrajectory display_trajectory;

    // goal positions of each arm
    geometry_msgs::Pose target_pose_left;
    geometry_msgs::Pose target_pose_right;
    // current positions of each arm    
    geometry_msgs::Pose cur_pose_left;
    geometry_msgs::Pose cur_pose_right;

    ros::Subscriber right_end_sub, left_end_sub;
    ros::Publisher display_pub;

    // neutral joint positions (for both arms)
    std::vector<double> both_arms_neutral_positions;
    // for adding/removing objects
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    // Callback functions
    // state of left end
    void leftEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
    // state of right end
    void rightEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);

};

#endif
