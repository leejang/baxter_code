#pragma once
/*
 * File: baxter_motion.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef BAXTER_MOTION_H_
#define BAXTER_MOTION_H_

#include <unistd.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "baxter_controller.h"
#include "std_msgs/UInt64.h"
#include "baxter_learning_from_egocentric_video/Target.h"
#include "baxter_learning_from_egocentric_video/TargetJoints.h"

class BaxterMotion
{
public:
    BaxterMotion(ros::NodeHandle nh);
    ~BaxterMotion();

    // Baxter Controller
    BaxterController *baxter_ctrl;
    //BaxterMoveitController *baxter_moveit_ctrl;

    void right_move_test(int img_x, int img_y);

private:
    ros::NodeHandle nh;

    ros::Subscriber right_target_sub;
    ros::Subscriber right_target_joints_sub;
    ros::Subscriber mission_cnt_sub;

    tf::TransformListener tfListener;

    // callback functions
    void rightTargetCB(const baxter_learning_from_egocentric_video::TargetConstPtr &msg);
    void rightTargetJointsCB(const baxter_learning_from_egocentric_video::TargetJointsConstPtr &msg);
    void missionCntCB(const std_msgs::UInt64::ConstPtr &msg);

    // positions, orientations
    double goal_right_pos[3], goal_right_ori[4];

    // test functions
    void right_move_test();
    
    unsigned int center_cnt;
    unsigned int center_left_cnt;
    unsigned int mission_cnt;
};

#endif

