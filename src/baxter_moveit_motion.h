#pragma once
/*
 * File: baxter_moveit_motion.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef BAXTER_MOVEIT_MOTION_H_
#define BAXTER_MOVEIT_MOTION_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include "baxter_moveit_controller.h"

class BaxterMoveitMotion
{
public:
    BaxterMoveitMotion(ros::NodeHandle nh);
    ~BaxterMoveitMotion();

    // handle timer interrupt
    void onTimerTick(const ros::TimerEvent& e);
    // maint test function to enable Baxter follow human hands
    void followHands(void);

    // Baxter Moveit Controller
    BaxterMoveitController *baxter_moveit_ctrl;

    // TF
    tf::TransformListener tfListener;
    //Transforms declared for each joint
    tf::StampedTransform tf_left_hand, tf_right_hand;
    // tf points declaration for storing 3D coordinates of joints
    tf::Point left_hand_pt, right_hand_pt;
    // to check before execution
    bool tf_exist;

    // goal positions for each arm
    geometry_msgs::Pose target_pose_left;
    geometry_msgs::Pose target_pose_right;

    geometry_msgs::Point target_point_left;
    geometry_msgs::Point target_point_right;
private:
    ros::NodeHandle nh;
};

#endif

