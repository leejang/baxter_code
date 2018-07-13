/*
 * File: baxter_motion.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "baxter_motion.h"

#define CONTROL_FREQ 100.0 // Hz
#define DEBUG 0
#define MISSION_START 5 * CONTROL_FREQ

using namespace std;

BaxterMotion::BaxterMotion(ros::NodeHandle nh)
{
    this->nh = nh;
    tf_exist = false;   

    mission_cnt = 0;

    // Baxter controller
    baxter_ctrl = new BaxterController(nh);
}

BaxterMotion::~BaxterMotion()
{
    delete baxter_ctrl;
}

void BaxterMotion::mimicOnTimerTick(const ros::TimerEvent& e)
{
    //ROS_INFO("mimicOnTimerTick");
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

void BaxterMotion::followOnTimerTick(const ros::TimerEvent& e)
{
    //ROS_INFO("followOnTimerTick");

    if (mission_cnt < MISSION_START) {
        mission_cnt++;
        return;
    } else if (mission_cnt < MISSION_START + 300) {
        double left_approach[] = {-0.1783, 0.0107, -0.8229, 0.4456, -0.1940, 1.012, -0.015};
        baxter_ctrl->moveLeftToJointPositions(left_approach);
        mission_cnt++;
        return;
    }

    try {
        // tfansform each body position to camera frame
        // get the transform from torso frame to each hand frame
        tfListener.lookupTransform("/base", "/left_hand_1", ros::Time(0), tf_left_hand);
        tfListener.lookupTransform("/base", "/right_hand_1", ros::Time(0), tf_right_hand);
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
        set_left_hand_pos[0] = left_hand_pt.x();
        set_left_hand_pos[1] = left_hand_pt.y();
        set_left_hand_pos[2] = left_hand_pt.z();
        // x, y, z respectively
        set_right_hand_pos[0] = right_hand_pt.x();
        set_right_hand_pos[1] = right_hand_pt.y();
        set_right_hand_pos[2] = right_hand_pt.z();

        // send commands to move hands
        baxter_ctrl->moveLeftHandTo(set_left_hand_pos);
        //baxter_ctrl->moveRightHandTo(set_right_hand_pos);
    }

}

int main(int argc, char **argv)
{
    unsigned int input;

    // initialize node
    ros::init(argc, argv, "mimic_motion");
    ros::NodeHandle nh;

    // mimic motion node
    BaxterMotion baxter_motion(nh);

#if 0
    ROS_INFO("Baxter Motion! (demo)");
    ROS_INFO("Type the number you would like to do");
    ROS_INFO("1: Mimicry human motion");
    ROS_INFO("2: Following human hands");

    cin >> input;
#endif

    // control timer
    ros::Timer timer;

    double left_lo_constrained[] = {-0.93, 0, 0, 0, 0, 0, 0};
    double left_up_constrained[] = {0, 0.5, 0, 0, 0, 0, 0};
    baxter_motion.baxter_ctrl->setLeftConstrainedPositions(left_up_constrained, left_lo_constrained);

    double right_up_constrained[] = {0, 0, 0, 0, 0, 0, 0};
    double right_lo_constrained[] = {0, 0, 0, 0, 0, 0, 0};
    baxter_motion.baxter_ctrl->setRightConstrainedPositions(right_up_constrained, right_lo_constrained);

    //baxter_motion.baxter_ctrl->moveToNeutral();
    //ros::Duration(1.0).sleep();

#if 0
    if (input == 1) {
        timer = nh.createTimer(ros::Duration(1/CONTROL_FREQ),
                               boost::bind(&BaxterMotion::mimicOnTimerTick, &baxter_motion, _1));
    } else if (input == 2) {
        timer = nh.createTimer(ros::Duration(1/CONTROL_FREQ),
                               boost::bind(&BaxterMotion::followOnTimerTick, &baxter_motion, _1));
    } else {
        ROS_ERROR("!!! Choose the right test scenario");
    }
#endif
    ros::spin();

    // shutdown ROS
    ros::shutdown();
    return 0;
}
