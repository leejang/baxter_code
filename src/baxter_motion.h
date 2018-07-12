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
#include "baxter_learning_from_egocentric_video/Target.h"

class BaxterMotion
{
public:
    BaxterMotion(ros::NodeHandle nh);
    ~BaxterMotion();

    // Baxter Controller
    BaxterController *baxter_ctrl;
private:
    ros::NodeHandle nh;

    ros::Subscriber right_target_sub;

    tf::TransformListener tfListener;

    // callback functions
    void rightTargetCB(const baxter_learning_from_egocentric_video::TargetConstPtr &msg);

};

#endif

