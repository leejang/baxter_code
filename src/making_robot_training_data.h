#pragma once
/*
 * File: making_robot_training_data.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef MAKING_ROBOT_TRAINING_DATA_H_
#define MAKING_ROBOT_TRAINING_DATA_H_

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_listener.h>
#include <baxter_core_msgs/EndpointState.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

#define NUM_OF_JOINTS 7

class MakingRobotTrainingData
{
public:
    MakingRobotTrainingData(ros::NodeHandle nh);
    ~MakingRobotTrainingData();

private:
    ros::NodeHandle nh;

    image_transport::ImageTransport *it;
    image_transport::CameraSubscriber cam_sub;
    image_geometry::PinholeCameraModel cam_model;

    tf::TransformListener tfListener;

    // Baster's joint states
    ros::Subscriber joint_states_sub;

    // to display skeleton tracking image on Baxter's screen
    ros::Publisher robot_screen;

    // Transforms declared for Baxter's each joint
    tf::StampedTransform tf_my_left_hand, tf_my_right_hand;

    // tf points declaration for storing 3D coordinates of joints
    tf::Point my_left_hand_pt, my_right_hand_pt;

    // to display on the Baxter's screen with full size (1024 * 600)
    cv_bridge::CvImage full_screen;

    // u, v points of each hand (x,y) current positions
    geometry_msgs::Pose2D my_left_hand_pose;
    geometry_msgs::Pose2D my_right_hand_pose;

    // predicted positions
    // future my left hand pose
    geometry_msgs::Pose2D f_my_left_hand_pose;
    geometry_msgs::Pose2D f_my_right_hand_pose;

    // current joint states
    double cur_left_joint_states[NUM_OF_JOINTS];
    double cur_right_joint_states[NUM_OF_JOINTS];

    // to save joint angles and hand positions in 2D
    // that will be used as training data   
    std::ofstream robot_train_d;

    // callback functions
    void onNewImageCB(const sensor_msgs::ImageConstPtr& image_msg,
                      const sensor_msgs::CameraInfoConstPtr& info_msg);

    void jointStatesCB(const sensor_msgs::JointStateConstPtr& joint_state_msg);
};

#endif
