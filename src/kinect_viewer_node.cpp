/*
 * File: kinect_viewer_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "kinect_viewer.h"

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "kinect_viewer");
    ros::NodeHandle nh;

    ROS_INFO("kinect viewer");

    // kinect viewer node
    KinectViewer kinect_view(nh);

    ros::spin();

    return 0;
}

