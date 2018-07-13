#pragma once
/*
 * File: baxter_motion.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef BAXTER_MOTION_H_
#define BAXTER_MOTION_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "baxter_controller.h"

class BaxterMotion
{
public:
    BaxterMotion(ros::NodeHandle nh);
    ~BaxterMotion();

    // handle timer interrupt
    void mimicOnTimerTick(const ros::TimerEvent& e);
    void followOnTimerTick(const ros::TimerEvent& e);

    // Baxter Controller
    BaxterController *baxter_ctrl;
private:
    ros::NodeHandle nh;

    tf::TransformListener tfListener;

    //Transforms declared for each joint
    tf::StampedTransform tf_left_hand, tf_right_hand;
    // tf points declaration for storing 3D coordinates of joints
    tf::Point left_hand_pt, right_hand_pt;

    bool tf_exist;

    unsigned int mission_cnt;

    double set_left_hand_pos[3];
    double set_right_hand_pos[3];
};

#endif

