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
    //ros::Rate r(10); // 10 hz

    ROS_INFO("Hand detection");

    // hand detector node
    HandDetector hand_detector(nh);

#if 0
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
#endif

#if 0
    // do hand detection
    int start_s = clock();

    //hand_detector.parseWindowInputFile();
    hand_detector.doDetection();

    int stop_s = clock();
    std::cout << "time: " << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << std::endl;
#endif

    ros::spin();

    // shutdown ROS
    ros::shutdown();

    return 0;
}

