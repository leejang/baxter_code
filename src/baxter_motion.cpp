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

    right_target_joints_sub =
        nh.subscribe("/detection/right/target_joints", 10, &BaxterMotion::rightTargetJointsCB, this);

    center_left_cnt = 0;

    total_target_cnt = 0;
}

BaxterMotion::~BaxterMotion()
{
    delete baxter_ctrl;
    //delete baxter_moveit_ctrl;
}

void BaxterMotion::rightTargetCB(const baxter_learning_from_egocentric_video::TargetConstPtr &msg)
{
    ROS_INFO("[%d] I hread: [%d %d]", total_target_cnt, msg->x, msg->y);

    if ((msg->x > 600) && (msg->x < 900)) {
        center_left_cnt++;
    }

    ROS_INFO("center_left_cnt: [%d]", center_left_cnt);

    if (total_target_cnt > 100) {
      baxter_ctrl->moveToNeutral();
    } else if ((center_left_cnt > 1) && (center_left_cnt <= 4)) {

      double right_end[] = {0.8509758420794333, -0.5763932810479442,0.0337475773334791, 1.3004322129298596, 1.363708920430133, 0.5000777368506448, 0.4693981210929366};
      baxter_ctrl->moveRightToJointPositions(right_end);
    } else if ((center_left_cnt > 4) && (center_left_cnt <= 7)) {
      double right_end[] = {0.9970875121255189, -0.10162622719740866, 0.02454369260616662, 0.4751505490475069, 1.3675438723998463, 0.2151408055009293, 0.47246608266870743};
      //baxter_ctrl->moveRightToJointPositions(right_end);
    } else {
      right_move_test(msg->x, msg->y);
    }

    total_target_cnt++;
    usleep(500);
}

void BaxterMotion::rightTargetJointsCB(const baxter_learning_from_egocentric_video::TargetJointsConstPtr &msg)
{
    ROS_INFO("[%d] I hread: [%f %f %f %f %f %f %f]",
              msg->joints[0], msg->joints[1], msg->joints[2],
              msg->joints[3], msg->joints[4], msg->joints[5], msg->joints[6]);
}

// for scenario 1
# if 0
void BaxterMotion::rightTargetCB(const baxter_learning_from_egocentric_video::TargetConstPtr &msg)
{
    ROS_INFO("I hread: [%d %d]", msg->x, msg->y);

    // 600, the center value in image x
    if (msg->x > 600) {
        center_left_cnt++;
    }

    ROS_INFO("center_left_cnt: [%d]", center_left_cnt);

    if ((center_left_cnt > 5) && (center_left_cnt <= 15)) {
      baxter_ctrl->moveRightHandTo(goal_right_pos, goal_right_ori);
      // move to right end to left most
      double right_end[] = {1.3334127998693959, 0.14764565083397108, 0.1284708909854034, 0.09050486648523941, 1.6225681783857964, 0.08628641931855452, 0.2231942046373277};
      baxter_ctrl->moveRightToJointPositions(right_end);
      
      ROS_INFO("Here I am!!");

    } else if (center_left_cnt > 15) {
      // move to right end to left most
      double right_end[] = {0.49854375606275947, 0.21207284392515846, -0.4195437454866607, -0.041033986075934815, 0.32597091742565043, 0.026461168591023387, -3.0591411862404865};
      baxter_ctrl->moveRightToJointPositions(right_end);
    } 
    else {
      right_move_test(msg->x, msg->y);
    }

    usleep(500);
}
#endif

void BaxterMotion::right_move_test(int img_x, int img_y)
{
    //ROS_INFO("Right Move Test Function");

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

    // right most
    //double right_end[] = {0.49854375606275947, 0.21207284392515846, -0.4195437454866607, -0.041033986075934815, 0.32597091742565043, 0.026461168591023387, -3.0591411862404865};

    // left most
    //double right_end[] = {1.4442429117941171, 0.19980099762207515, -0.28723790253154374, 0.023776702212223912, 3.057607205452601, -0.012655341500054663, -2.954446997467307};

    // appraching center
    //double right_end[] = {0.8509758420794333, -0.5763932810479442,0.0337475773334791, 1.3004322129298596, 1.363708920430133, 0.5000777368506448, 0.4693981210929366};
    //baxter_ctrl->moveRightToJointPositions(right_end);

}

int main(int argc, char **argv)
{
    ROS_INFO("Baxter Motion");
 
   // initialize node
    ros::init(argc, argv, "baxter_motion");
    ros::NodeHandle nh;

    // mimic motion node
    BaxterMotion baxter_motion(nh);

    //baxter_motion.right_move_test(800, 600);
    // table center
    //baxter_motion.right_move_test(607, 504);

    ros::spin();

    // shutdown ROS
    ros::shutdown();
    return 0;
}
