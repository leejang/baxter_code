/*
 * File: baxter_motion.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "baxter_motion.h"

// 0.1 ~ 0.13
#define TABLE_Z 0.10
#define FIXED_ORI_X 0.73
#define FIXED_ORI_Y 0.32
#define FIXED_ORI_Z 0.6
#define FIXED_ORI_W -0.1

#define img_x_min 330
#define img_x_max 1000
#define img_y_min 500
#define img_y_max 640

// img_x corresponds to arm_y
// img_y corresponds to arm_x
// here arm uess the robot base coordinate
// x:front, y:left, z: height
// put - to fit it to the right direction
#define arm_y_min -0.48
#define arm_y_max 0.66
#define arm_x_min -1.15
#define arm_x_max -0.85

using namespace std;

BaxterMotion::BaxterMotion(ros::NodeHandle nh)
{
    this->nh = nh;

    // Baxter controller
    baxter_ctrl = new BaxterController(nh);

    // Baxter moveit controller
    //baxter_moveit_ctrl = new BaxterMoveitController(nh);

    right_target_sub =
        nh.subscribe("/detection/right/target_pos", 10, &BaxterMotion::rightTargetCB, this);
}

BaxterMotion::~BaxterMotion()
{
    delete baxter_ctrl;
    //delete baxter_moveit_ctrl;
}

void BaxterMotion::rightTargetCB(const baxter_learning_from_egocentric_video::TargetConstPtr &msg)
{
    ROS_INFO("I hread: [%d %d]", msg->x, msg->y);
}

void BaxterMotion::right_move_test(int img_x, int img_y)
{
    ROS_INFO("Right Move Test Function");

    double new_arm_x = 0;
    double new_arm_y = 0;

    new_arm_x = ((arm_x_max - arm_x_min)/(img_y_max - img_y_min))*(img_y - img_y_max) + arm_x_max;
    new_arm_y = ((arm_y_max - arm_y_min)/(img_x_max - img_x_min))*(img_x - img_x_max) + arm_y_max;

    ROS_INFO("Right Goal Pos: [%f %f]", -new_arm_x, -new_arm_y);

    goal_right_pos[0] = -new_arm_x;
    goal_right_pos[1] = -new_arm_y;
    goal_right_pos[2] = TABLE_Z;

    goal_right_ori[0] = FIXED_ORI_X;
    goal_right_ori[1] = FIXED_ORI_Y;
    goal_right_ori[2] = FIXED_ORI_Z;
    goal_right_ori[3] = FIXED_ORI_W;

    // send commands to move hands
    baxter_ctrl->moveRightHandTo(goal_right_pos, goal_right_ori);
}

int main(int argc, char **argv)
{
    ROS_INFO("Baxter Motion");
 
   // initialize node
    ros::init(argc, argv, "baxter_motion");
    ros::NodeHandle nh;

    // mimic motion node
    BaxterMotion baxter_motion(nh);

    baxter_motion.right_move_test(800, 600);

    ros::spin();

    // shutdown ROS
    ros::shutdown();
    return 0;
}
