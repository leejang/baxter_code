#pragma once
/*
 * File: train_representation.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef TRAIN_REPRESENTATION_H_
#define TRAIN_REPRESENTATION_H_

#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>
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
    ros::Subscriber detected_sub;

    std::ofstream detection_result;
    unsigned int frame_cnt;
    // callback functions
    void objectsDetectedCallback(const std_msgs::Float32MultiArray & msg);

};

#endif
