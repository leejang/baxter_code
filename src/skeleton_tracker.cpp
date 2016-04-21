/*
 * File: skeleton_tracker.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "skeleton_tracker.h"
#define ROS_DEPTH_PATH "/camera/depth/image_raw"

using namespace std;

SkeletonTracker::SkeletonTracker(ros::NodeHandle nh){
    this->nh = nh;
    
    head_joint = nh.advertise<geometry_msgs::Point>("skeleton/head_joint", 1);
    left_hand_joint = nh.advertise<geometry_msgs::Point>("skeleton/left_hand_joint", 1);
    right_hand_joint = nh.advertise<geometry_msgs::Point>("skeleton/right_hand_joint", 1);

    sub_depth = nh.subscribe(ROS_DEPTH_PATH, 10, &SkeletonTracker::onNewDepthCallback, this);
}

SkeletonTracker::~SkeletonTracker()
{
}

void SkeletonTracker::onNewDepthCallback(const sensor_msgs::Image& dimage)
{
    if (&dimage == NULL) {
        ROS_WARN_THROTTLE(1,"NULL image received.");
        return;
    } else {
       // ROS_INFO("Received Depth Info");

       try
       {
           tfListener.lookupTransform("/head_1", "/camera_depth_optical_frame", ros::Time(0), tf_head);
           tfListener.lookupTransform("/left_hand_1", "/camera_depth_optical_frame", ros::Time(0), tf_left_hand);
           tfListener.lookupTransform("/right_hand_1", "/camera_depth_optical_frame", ros::Time(0), tf_right_hand);
       }
       catch (tf::TransformException & ex)
       {
            //ROS_WARN("%s",ex.what());
       }

       // Head joint
       head_pose.x = tf_head.getOrigin().x();
       head_pose.y = tf_head.getOrigin().y();
       head_pose.z = tf_head.getOrigin().z();

       // Left Hand joint
       left_hand_pose.x = tf_left_hand.getOrigin().x();
       left_hand_pose.y = tf_left_hand.getOrigin().y();
       left_hand_pose.z = tf_left_hand.getOrigin().z();

       // Right Hand joint
       right_hand_pose.x = tf_right_hand.getOrigin().x();
       right_hand_pose.y = tf_right_hand.getOrigin().y();
       right_hand_pose.z = tf_right_hand.getOrigin().z();

       // Publish each joint positions
       head_joint.publish(head_pose);
       left_hand_joint.publish(left_hand_pose);
       right_hand_joint.publish(right_hand_pose);
    }
}

