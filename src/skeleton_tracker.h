#pragma once
/*
 * File: skeleton_tracker.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef SKELETON_TRACKER_H_
#define SKELETON_TRACKER_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>

class SkeletonTracker
{
public:
    SkeletonTracker(ros::NodeHandle nh);
    ~SkeletonTracker();

private:
    ros::NodeHandle nh;
    ros::Publisher head_joint;
    ros::Publisher left_hand_joint, right_hand_joint;

    ros::Subscriber sub_depth;;
    tf::TransformListener tfListener;

    //Transforms declared for each joint
    tf::StampedTransform tf_head;
    tf::StampedTransform tf_left_hand, tf_right_hand;

    // geometry points declaration for storing 3D coordinates of joints
    geometry_msgs::Point head_pose;
    geometry_msgs::Point left_hand_pose, right_hand_pose;

    void onNewDepthCallback(const sensor_msgs::Image& dimage);
};

#endif

