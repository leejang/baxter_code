/*
 * File: hand_detection_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "hand_detector.h"
#include <ctime>

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "hand_detection");
    ros::NodeHandle nh;

    ROS_INFO("Hand detection");

    // hand detector node
    HandDetector hand_detector(nh);

    // do hand detection

    int start_s = clock();

    hand_detector.doDetection();

    int stop_s = clock();
    std::cout << "time: " << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << std::endl;
    ros::spin();

    return 0;
}

