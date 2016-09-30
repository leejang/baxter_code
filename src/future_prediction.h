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

#include "hdf5.h"

#define DATA_DIM 4
#define LABEL_DIM 2
#define DIM0 2
#define DIM1 10
#define CHUNK_SZ 10

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

    // publishcer to send detection completed message
    std_msgs::UInt64 completed_frame_cnt;
    ros::Publisher detected_pub;

    // u, v points of each hand
    geometry_msgs::Pose2D left_hand_pose;
    geometry_msgs::Pose2D right_hand_pose;

    // Caffe
    Net<float> *caffe_net;
    
    void headPoseCB(const geometry_msgs::Pose2D pose);
    void leftHandPoseCB(const geometry_msgs::Pose2D pose);
    void rightHandPoseCB(const geometry_msgs::Pose2D pose);
    
    unsigned int head_pose_cnt;

    // To write HDF5 file
    // Handles
    hid_t file, dspace, lspace, dset, lset, dcpl, lcpl;
    herr_t status;
    H5D_layout_t layout;
    hsize_t d_dims[DATA_DIM];
    hsize_t l_dims[LABEL_DIM];

    double dataset[CHUNK_SZ][1][DIM0][DIM1*CHUNK_SZ];
    double labelset[CHUNK_SZ][DIM0*CHUNK_SZ];

};
#endif
