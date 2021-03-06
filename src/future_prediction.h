#pragma once
/*
 * File: future_prediction.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef FUTURE_PREDICTION_H_
#define FUTURE_PREDICTION_H_

#include <stdio.h>
#include <string>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "engine.h"
#include <ros/ros.h>
#include <caffe/caffe.hpp>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <find_object_2d/ObjectsStamped.h>
#include <baxter_core_msgs/EndpointState.h>

#include "baxter_controller.h"
#include "baxter_moveit_controller.h"

#include "hdf5.h"

#define DATA_DIM 4
#define LABEL_DIM 2
#define DIM0 2
#define DIM1 10 /* number of variables in state representation */
#define DIM2 10 /* number of concatenated vectors */
#define CHUNK_SZ 1

#define NUM_OF_MOTIONS 27
#define NUM_OF_JOINTS 7

using caffe::Caffe;
using caffe::Net;
using caffe::Blob;
using caffe::vector;
using caffe::Layer;
using caffe::shared_ptr;

class FuturePrediction
{
public:
    FuturePrediction(ros::NodeHandle nh);
    ~FuturePrediction();
    void predictFuture();

private:
    ros::NodeHandle nh;

    // subscribers to get hand positons from skeleton tracker
    // in order to reduce number of windows
    ros::Subscriber head_pose_sub;
    ros::Subscriber left_hand_pose_sub;
    ros::Subscriber right_hand_pose_sub;

    // subscriber to get detected objects positions
    ros::Subscriber obj_pose_sub;
    ros::Subscriber obj_pose_w_depth_sub;

    // robot both arm's endpoints
    ros::Subscriber right_end_sub;
    ros::Subscriber left_end_sub;

    // to display skeleton tracking image on Baxter's screen
    ros::Publisher robot_screen;

    // publishcer to send detection completed message
    std_msgs::UInt64 completed_frame_cnt;
    ros::Publisher detected_pub;

    // u, v points of each hand (x,y) current positions
    geometry_msgs::Pose2D my_left_hand_pose;
    geometry_msgs::Pose2D my_right_hand_pose;
    geometry_msgs::Pose2D yr_left_hand_pose;
    geometry_msgs::Pose2D yr_right_hand_pose;
    geometry_msgs::Pose2D obj_pan_pose;
    geometry_msgs::Pose2D obj_trivet_pose;
    geometry_msgs::Pose2D obj_beer_box_pose;
    geometry_msgs::Pose2D obj_oatmeal_pose;
    geometry_msgs::Pose2D obj_butter_pose;
    geometry_msgs::Pose2D obj_coffee_pose;

    tf::StampedTransform obj_trivet_pose_w_d;

    // ref_pose (table)
    geometry_msgs::Pose2D ref_pose;

    // predicted positions
    // future my left hand pose
    geometry_msgs::Pose2D f_my_left_hand_pose;
    geometry_msgs::Pose2D f_my_right_hand_pose;
    geometry_msgs::Pose2D f_yr_left_hand_pose;
    geometry_msgs::Pose2D f_yr_right_hand_pose;
    geometry_msgs::Pose2D f_obj_pan_pose;
    geometry_msgs::Pose2D f_obj_trivet_pose;

    // Caffe
    Net<float> *caffe_net;
    
    void headPoseCB(const geometry_msgs::Pose2D pose);
    void leftHandPoseCB(const geometry_msgs::Pose2D pose);
    void rightHandPoseCB(const geometry_msgs::Pose2D pose);

    void objPoseCB(const std_msgs::Float32MultiArray & msg);
    void objPoseWDepthCB(const find_object_2d::ObjectsStampedConstPtr & msg);
    unsigned int head_pose_cnt;

    void onNewImageCB(const sensor_msgs::ImageConstPtr& image_msg,
                            const sensor_msgs::CameraInfoConstPtr& info_msg);
    // state of left end
    void leftEndCB(const baxter_core_msgs::EndpointStateConstPtr &msg);
    // state of right end
    void rightEndCB(const baxter_core_msgs::EndpointStateConstPtr &msg);

    // To write HDF5 file
    // Handles
    hid_t file, dspace, lspace, dset, lset, dcpl, lcpl;
    herr_t status;
    H5D_layout_t layout;
    hsize_t d_dims[DATA_DIM];
    hsize_t l_dims[LABEL_DIM];

    float dataset[CHUNK_SZ][1][DIM0][DIM1*DIM2];
    float labelset[CHUNK_SZ][DIM0*DIM1];

    bool save_hdf_file;
    bool got_ref_pose;

    image_transport::ImageTransport *it;
    image_transport::CameraSubscriber cam_sub;
    image_geometry::PinholeCameraModel cam_model;

    tf::TransformListener tfListener;

    // Transforms declared for Baxter's each joint
    tf::StampedTransform tf_my_left_hand, tf_my_right_hand;

    // tf points declaration for storing 3D coordinates of joints
    tf::Point my_left_hand_pt, my_right_hand_pt;

    // to display on the Baxter's screen with full size (1024 * 600)
    cv_bridge::CvImage full_screen;

    // Baxter Controller
    BaxterController *baxter_ctrl;
    BaxterMoveitController *baxter_moveit_ctrl;

    // goal positions for each arm
    geometry_msgs::Pose target_pose_left;
    geometry_msgs::Pose target_pose_right;

    geometry_msgs::Point target_point_left;
    geometry_msgs::Point target_point_right;

    geometry_msgs::Pose cur_pose_left;
    geometry_msgs::Pose cur_pose_right;

    // to generate Baxter Motions
    void generateRobotMotion();

    // Util functions
    int getDistance(int x1, int y1, int x2, int y2);

    void loadMotionMappingTable();
    double motionMapping[NUM_OF_MOTIONS][9];
};
#endif
