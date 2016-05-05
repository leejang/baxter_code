/*
 * File: mimic_motion.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "mimic_motion.h"

#define CONTROL_FREQ 100 // Hz
#define DEBUG 0

using namespace std;

MimicMotion::MimicMotion(ros::NodeHandle nh)
{
    this->nh = nh;
    tf_exist = false;   

    // Baxter controller
    baxter_ctrl = new BaxterController(nh);
}

MimicMotion::~MimicMotion()
{
    delete baxter_ctrl;
}

void MimicMotion::onTimerTick(const ros::TimerEvent& e)
{
    try
    {
        // tfansform each body position to camera frame
        // get the transform from torso frame to each hand frame
        tfListener.lookupTransform("/torso_1", "/left_hand_1", ros::Time(0), tf_left_hand);
        tfListener.lookupTransform("/torso_1", "/right_hand_1", ros::Time(0), tf_right_hand);
        tf_exist = true;
    }
    catch (tf::TransformException & ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.5).sleep();
        tf_exist = false;
    }

    if (tf_exist) {
        // left Hand
        left_hand_pt = tf_left_hand.getOrigin();
        // right Hand
        right_hand_pt = tf_right_hand.getOrigin();

#if DEBUG
        cout << "Left Hand Pt: " << left_hand_pt.x() << ", "
             << left_hand_pt.y() << ", " << left_hand_pt.z() << endl;

        cout << "Right Hand Pt: " << right_hand_pt.x() << ", "
             << right_hand_pt.y() << ", " << right_hand_pt.z() << endl;
#endif

        // mapping user hand position to baxter's end effetor position
        // x, y, z respectively
        set_left_hand_pos[0] = - (2 * left_hand_pt.z());
        set_left_hand_pos[1] = 2 * left_hand_pt.x();
        set_left_hand_pos[2] = 3 * left_hand_pt.y();
        // x, y, z respectively
        set_right_hand_pos[0] = - (2 * right_hand_pt.z());
        set_right_hand_pos[1] = 2 * right_hand_pt.x();
        set_right_hand_pos[2] = 3 * right_hand_pt.y();

        // send commands to move hands
        baxter_ctrl->moveLeftHandTo(set_left_hand_pos);
        baxter_ctrl->moveRightHandTo(set_right_hand_pos);
    }
}

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "mimic_motion");
    ros::NodeHandle nh;

    ROS_INFO("Mimic Human Motion");

    // mimic motion node
    MimicMotion mimic_motion(nh);

    // control timer
    ros::Timer timer;
    timer = nh.createTimer(ros::Duration(1/CONTROL_FREQ), boost::bind(&MimicMotion::onTimerTick, &mimic_motion, _1));

    ros::spin();

    // shutdown ROS
    ros::shutdown();
    return 0;
}
