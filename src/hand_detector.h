#pragma once
/*
 * File: hand_detector.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef HAND_DETECTOR_H_
#define HAND_DETECTOR_H_

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "engine.h"
#include <caffe/caffe.hpp>
#include <geometry_msgs/Point.h>

using caffe::Caffe;
using caffe::Net;
using caffe::Blob;
using caffe::vector;
using caffe::Layer;
using caffe::shared_ptr;

class HandDetector
{
public:
    HandDetector(ros::NodeHandle nh);
    ~HandDetector();
    void doDetection();

private:
    ros::NodeHandle nh;

    ros::Subscriber left_hand_pose_sub;
    ros::Subscriber right_hand_pose_sub;

    geometry_msgs::Point left_hand_pose;
    geometry_msgs::Point right_hand_pose;

    // Caffe
    Net<float> *caffe_net;
    // MATLAB Engine
    Engine *matlab_ep;

    int initMatlabEngine();
    void generateWindowProposals();
    int parseWindowInputFile();

    void leftHandPoseCB(const geometry_msgs::Point pose);
    void rightHandPoseCB(const geometry_msgs::Point pose);
 
};
#endif
