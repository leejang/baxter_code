#pragma once
/*
 * File: mimic_motion.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef MIMIC_MOTION_H_
#define MIMIC_MOTION_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "baxter_controller.h"

class MimicMotion
{
public:
    MimicMotion(ros::NodeHandle nh);
    ~MimicMotion();

    // handle timer interrupt
    void onTimerTick(const ros::TimerEvent& e);
private:
    ros::NodeHandle nh;

    tf::TransformListener tfListener;

    //Transforms declared for each joint
    tf::StampedTransform tf_left_hand, tf_right_hand;
    // tf points declaration for storing 3D coordinates of joints
    tf::Point left_hand_pt, right_hand_pt;

    bool tf_exist;

    // Baxter Controller
    BaxterController *baxter_ctrl;

    double set_left_hand_pos[3];
    double set_right_hand_pos[3];
};

#endif

