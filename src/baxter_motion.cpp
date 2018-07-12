/*
 * File: baxter_motion.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "baxter_motion.h"

using namespace std;

BaxterMotion::BaxterMotion(ros::NodeHandle nh)
{
    this->nh = nh;

    // Baxter controller
    //baxter_ctrl = new BaxterController(nh);

    right_target_sub =
        nh.subscribe("/detection/right/target_pos", 10, &BaxterMotion::rightTargetCB, this);
}

BaxterMotion::~BaxterMotion()
{
    //delete baxter_ctrl;
}

void BaxterMotion::rightTargetCB(const baxter_learning_from_egocentric_video::TargetConstPtr &msg)
{
    ROS_INFO("I hread: [%d %d]", msg->x, msg->y);

}

int main(int argc, char **argv)
{
    ROS_INFO("Baxter Motion");
 
   // initialize node
    ros::init(argc, argv, "baxter_motion");
    ros::NodeHandle nh;

    // mimic motion node
    BaxterMotion baxter_motion(nh);

    ros::spin();

    // shutdown ROS
    ros::shutdown();
    return 0;
}
