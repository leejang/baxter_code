#pragma once
/*
 * File: object_detector.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef OBJECT_DETECTOR_H_
#define OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>

class ObjectDetector
{
public:
    ObjectDetector(ros::NodeHandle nh);
    ~ObjectDetector();

    tf::StampedTransform pose;
    tf::StampedTransform poseCam;
private:
    ros::NodeHandle nh;
    ros::Subscriber detected_sub;
    ros::Subscriber detected_w_depth_sub;
    ros::Publisher detected_w_depth_pub;
    tf::TransformListener tfListener_;
    void objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr & msg);
};

#endif
