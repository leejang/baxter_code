#pragma once
/*
 * File: kinect_viewer.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef KINECT_VIEWER_H_
#define KINECT_VIEWER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class KinectViewer
{
public:
    KinectViewer(ros::NodeHandle nh);
    ~KinectViewer();

private:
    ros::NodeHandle nh;
    image_transport::ImageTransport *it;

    image_transport::Subscriber img_sub_rgb;
    message_filters::Subscriber< stereo_msgs::DisparityImage> img_sub_disparity;
  
    void rgbCB(const sensor_msgs::ImageConstPtr &msg);
    void disparityCB(const stereo_msgs::DisparityImageConstPtr& msg);
};

#endif
