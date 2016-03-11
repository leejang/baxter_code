/*
 * File: kinect_viewer.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "kinect_viewer.h"

using namespace std;

KinectViewer::KinectViewer(ros::NodeHandle nh)
{
    this->nh = nh;

    it = new image_transport::ImageTransport(nh);
    img_sub_rgb = it->subscribe("/camera/rgb/image_color", 1, &KinectViewer::rgbCB, this);
    img_sub_disparity.subscribe(nh, "/camera/depth/disparity", 1);

    img_sub_disparity.registerCallback(&KinectViewer::disparityCB, this);
}

KinectViewer::~KinectViewer()
{
    if (it)
        delete it;
}


void KinectViewer::disparityCB(const stereo_msgs::DisparityImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr_disp;

    try
    {
        cv_ptr_disp = cv_bridge::toCvShare(msg->image, msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }

    cv::imshow("Kinect disparity Cam", cv_ptr_disp->image);
    cv::waitKey(1);
}

void KinectViewer::rgbCB(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }

    cv::imshow("Kinect RGB Cam", cv_ptr->image);
    cv::waitKey(1);
}

