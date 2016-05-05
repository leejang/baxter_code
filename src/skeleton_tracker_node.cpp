/*
 * File: skeleton_tracker_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "skeleton_tracker.h"

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "skeleton_tracker");
    ros::NodeHandle nh;

    ROS_INFO("Skeleton Tracker");

    // skeleton tracker node
    SkeletonTracker skeleton_tracker(nh);
    ros::spin();

    // shutdown ROS
    ros::shutdown();

    return 0;
}
