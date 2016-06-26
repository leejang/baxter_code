/*
 * File: train_representation_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "train_representation.h"

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "train_representation");
    ros::NodeHandle nh;

    ROS_INFO("train representation");

    // train representation node
    TrainRepresentation train_representation(nh);

    // start training
    train_representation.doTraining();

    ros::spin();

    return 0;
}

