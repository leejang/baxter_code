#pragma once
/*
 * File: baxter_controller.h
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#ifndef BAXTER_CONTROLLER_H_
#define BAXTER_CONTROLLER_H_

#include <vector>
#include <string>
#include <boost/unordered_map.hpp>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/SolvePositionIK.h>

typedef boost::unordered_map<std::string, float> hash;

class BaxterController
{
public:
    BaxterController(ros::NodeHandle nh);
    ~BaxterController();

    // enable Baxter
    void enable();

    // set Baxter control frequency (Hz)
    void setCtrlFreq(unsigned int new_freq);

    // move Baxter's both arms to neutral position
    int moveToNeutral();

    // set joint constrained positions
    int setRightConstrainedPositions(double up_bounds[], double lo_bounds[]);
    int setLeftConstrainedPositions(double up_bounds[], double lo_bounds[]);

    // move to joint positions
    int moveRightToJointPositions(double set_positions[]);
    int moveLeftToJointPositions(double set_positions[]);

    // move Baxter's hands to..
    int moveRightHandTo(double set_pos[]);
    int moveRightHandTo(double set_pos[], double set_ori[]);
    int moveLeftHandTo(double set_pos[]);
    int moveLeftHandTo(double set_pos[], double set_ori[]);

    // functions to grip and release
    int right_grip();
    int right_release();
    int left_grip();
    int left_release();

private:
    ros::NodeHandle nh;

    ros::Subscriber right_end_sub, left_end_sub;
    ros::Subscriber right_grip_sub, left_grip_sub;
    ros::Subscriber joint_state_sub;

    ros::Publisher enable_baxter_pub;
    ros::Publisher right_gripper_pub, right_joint_pub;
    ros::Publisher left_gripper_pub, left_joint_pub;

    ros::ServiceClient right_ik_client, left_ik_client;

    std_msgs::Bool baxter_enabled;
    baxter_core_msgs::JointCommand right_joint_cmd, left_joint_cmd;

    // control timer
    ros::Timer ctrl_timer;
    double ctrl_freq;
    unsigned int timeout;
    unsigned int right_timeout_cnt;
    unsigned int left_timeout_cnt;

    // joint names of left and right arm
    std::vector<std::string> right_joint_names;
    std::vector<std::string> left_joint_names;

    // neutral joint positions (for both arms)
    std::vector<double> neutral_positions;

    // constrained joint positions to avoid collision
    // uppper bound and lower bound
    std::vector<double> right_constrained_positions_up;
    std::vector<double> right_constrained_positions_lo;
    std::vector<double> left_constrained_positions_up;
    std::vector<double> left_constrained_positions_lo;

    // current and goal joint positions
    hash current_positions;

    // current state of right hand
    // positions, orientations
    double cur_right_pos[3], cur_right_ori[4];
    // current state of left hand
    // positions, orientations
    double cur_left_pos[3], cur_left_ori[4];

    // goal state of right hand
    // positions, orientations
    double goal_right_pos[3], goal_right_ori[4];
    // goal state of left hand
    // positions, orientations
    double goal_left_pos[3], goal_left_ori[4];

    unsigned int right_gripper_hid;
    unsigned int left_gripper_hid;

    bool right_has_to_move;
    bool left_has_to_move;

    // check joint differences between goal positions and current positions
    bool checkRightJointDiff(std::vector<double> goal_positions);
    bool checkLeftJointDiff(std::vector<double> goal_positions);

    // check constrained positions to avoid collisions
    bool checkRightConstrainedPos(std::vector<double> positions_to_move);
    bool checkLeftConstrainedPos(std::vector<double> positions_to_move);

    // timer function
    void onCtrlTimerTick(const ros::TimerEvent& e);

    // Callback functions
    // state of right end
    void rightEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
    // state of left end
    void leftEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg);
    // grippers
    void rightGripCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg);
    void leftGripCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg);
    // joint state
    void jointStateCallback(const sensor_msgs::JointStateConstPtr &msg);
};

#endif
