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
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class SkeletonTracker
{
public:
    SkeletonTracker(ros::NodeHandle nh);
    ~SkeletonTracker();

private:
    ros::NodeHandle nh;
    ros::Publisher head_joint;
    ros::Publisher left_hand_joint, right_hand_joint;

    tf::TransformListener tfListener;

    //Transforms declared for each joint
    tf::StampedTransform tf_head;
    tf::StampedTransform tf_left_hand, tf_right_hand;
    tf::StampedTransform tf_torso;

    // tf points declaration for storing 3D coordinates of joints
    tf::Point head_pt;
    tf::Point left_hand_pt, right_hand_pt;
    tf::Point torso_pt;

    image_transport::ImageTransport *it;
    image_transport::CameraSubscriber sub_depth;

    image_geometry::PinholeCameraModel cam_model;

    cv::Point2d head_uv;
    cv::Point2d left_hand_uv, right_hand_uv;
    cv::Point2d torso_uv;

    // to publish a position(u,v) of each joint in image
    geometry_msgs::Pose2D head_pose;
    geometry_msgs::Pose2D left_hand_pose, right_hand_pose;
    geometry_msgs::Pose2D torso_pose;

    void onNewImageCallback(const sensor_msgs::ImageConstPtr& image_msg,
                            const sensor_msgs::CameraInfoConstPtr& info_msg);

    // to save video output
    cv::VideoWriter output_video;
};

#endif

