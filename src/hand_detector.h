#pragma once
/*
 * File: hand_detector.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef HAND_DETECTOR_H_
#define HAND_DETECTOR_H_

#include <stdlib.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <caffe/caffe.hpp>

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
    void generateWindowProposals();
    void doDetection();

private:
    ros::NodeHandle nh;
    ros::Subscriber detected_sub;
    ros::Publisher detected_pub;

    // Caffe
    Net<float> *caffe_net;
};
#endif
