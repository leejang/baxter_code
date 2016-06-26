#pragma once
/*
 * File: train_representation.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef TRAIN_REPRESENTATION_H_
#define TRAIN_REPRESENTATION_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class TrainRepresentation
{
public:
    TrainRepresentation(ros::NodeHandle nh);
    ~TrainRepresentation();

    int doTraining();

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport *it;
    image_transport::Publisher image_pub;

    // to open video file
    cv::VideoCapture capture_video;
};

#endif
