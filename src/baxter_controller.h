#pragma once
/*
 * File: baxter_controller.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef BAXTER_CONTROLLER_H_
#define BAXTER_CONTROLLER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>

class BaxterController
{
public:
    BaxterController(ros::NodeHandle nh);
    ~BaxterController();

    // Callback functions
    // state of right end
    void rightEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
    // state of left end
    void leftEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
    // grippers
    void rightGripCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg);
    void leftGripCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg);

    // enable Baxter
    void enable();

    // move Baxter's hands to..
    int moveRightHandTo(double set_pos[], double set_ori[]);
    int moveLeftHandTo(double set_pos[], double set_ori[]);

    int right_grip();
    int right_release();
    int left_grip();
    int left_release();

private:
    ros::NodeHandle nh;

    ros::Subscriber right_end_sub, left_end_sub;
    ros::Subscriber right_grip_sub, left_grip_sub;

    ros::Publisher enable_baxter_pub;
    ros::Publisher right_gripper_pub, right_joint_pub;
    ros::Publisher left_gripper_pub, left_joint_pub;

    ros::ServiceClient right_ik_client, left_ik_client;

    std_msgs::Bool baxter_enabled;
    baxter_core_msgs::JointCommand right_joint_cmd, left_joint_cmd;

    // current state of right hand
    // positions, orientations
    double cur_right_pos[3], cur_right_ori[4];
    // current state of left hand
    // positions, orientations
    double cur_left_pos[3], cur_left_ori[4];

    // goal state of right hand
    // positions, orientations
    double goal_right_pos[3], goal_right_ori[4];
    // goal state of left hand
    // positions, orientations
    double goal_left_pos[3], goal_left_ori[4];

    unsigned int right_gripper_hid;
    unsigned int left_gripper_hid;

    bool right_has_to_move;
    bool left_has_to_move;
};

#endif
