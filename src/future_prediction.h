#pragma once
/*
 * File: future_prediction.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef FUTURE_PREDICTION_H_
#define FUTURE_PREDICTION_H_

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "engine.h"
#include <caffe/caffe.hpp>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/UInt64.h>

using caffe::Caffe;
using caffe::Net;
using caffe::Blob;
using caffe::vector;
using caffe::Layer;
using caffe::shared_ptr;

class FuturePrediction
{
public:
    FuturePrediction(ros::NodeHandle nh);
    ~FuturePrediction();
    void predictFuture();

private:
    ros::NodeHandle nh;

    // subscribers to get hand positons from skeleton tracker
    // in order to reduce number of windows
    ros::Subscriber head_pose_sub;
    ros::Subscriber left_hand_pose_sub;
    ros::Subscriber right_hand_pose_sub;

    // publishcer to send detection completed message
    std_msgs::UInt64 completed_frame_cnt;
    ros::Publisher detected_pub;

    // u, v points of each hand
    geometry_msgs::Pose2D left_hand_pose;
    geometry_msgs::Pose2D right_hand_pose;

    // Caffe
    Net<float> *caffe_net;
    
    void headPoseCB(const geometry_msgs::Pose2D pose);
    void leftHandPoseCB(const geometry_msgs::Pose2D pose);
    void rightHandPoseCB(const geometry_msgs::Pose2D pose);
    
    unsigned int head_pose_cnt;
};
#endif
