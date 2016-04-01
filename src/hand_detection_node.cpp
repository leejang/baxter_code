/*
 * File: hand_detection_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "hand_detector.h"

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "hand_detection");
    ros::NodeHandle nh;

    ROS_INFO("Hand detection");

    // hand detector node
    HandDetector hand_detector(nh);

    // do detection
    hand_detector.doDetection();

    ros::spin();

    return 0;
}

