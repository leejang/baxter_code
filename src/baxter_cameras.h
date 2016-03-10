#pragma once
/*
 * File: baxter_cameras.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef BAXTER_CAMERAS_H_
#define BAXTER_CAMERAS_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/CameraSettings.h>
#include <baxter_core_msgs/CameraControl.h>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/CloseCamera.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class BaxterCameras
{
public:
    enum CameraId {HEAD, LEFT_HAND, RIGHT_HAND};

    struct CameraParams
    {
        unsigned int width;
        unsigned int height;
        unsigned int fps;
    };

    BaxterCameras(ros::NodeHandle nh);
    ~BaxterCameras();

    void open(CameraId id, CameraParams cam_param);

private:
    ros::NodeHandle nh;
    baxter_core_msgs::OpenCamera open_camera;
    baxter_core_msgs::CloseCamera close_camera;
    baxter_core_msgs::CameraControl camera_control;
    image_transport::ImageTransport *it_head;
    image_transport::ImageTransport *it_l_hand, *it_r_hand;

    image_transport::Subscriber img_sub_head;
    image_transport::Subscriber img_sub_l_hand, img_sub_r_hand;

    void headCB(const sensor_msgs::ImageConstPtr &msg);
    void leftHandCB(const sensor_msgs::ImageConstPtr &msg);
    void rightHandCB(const sensor_msgs::ImageConstPtr &msg);
};

#endif
