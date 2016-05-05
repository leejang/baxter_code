/*
 * File: baxter_controller.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "baxter_controller.h"

using namespace std;

BaxterController::BaxterController(ros::NodeHandle nh)
{
    // subscribers
    // endpoints
    right_end_sub =
        nh.subscribe("/robot/limb/right/endpoint_state", 2, &BaxterController::rightEndCallback, this);
    left_end_sub =
        nh.subscribe("/robot/limb/left/endpoint_state", 2, &BaxterController::leftEndCallback, this);
    // grippers
    right_grip_sub =
        nh.subscribe("/robot/end_effector/right_gripper/state", 2, &BaxterController::rightGripCallback, this);
    left_grip_sub =
        nh.subscribe("/robot/end_effector/left_gripper/state", 2, &BaxterController::leftGripCallback, this);

    // publishers
    // enable/disable Baxter
    enable_baxter_pub =
        nh.advertise<std_msgs::Bool>("robot/set_super_enable", 5);
    // right hand
    right_joint_pub = 
        nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 2);
    right_gripper_pub =
        nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 2);
    // left hand
    left_joint_pub = 
        nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 2);
    left_gripper_pub =
        nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 2);
    // IL clients
    right_ik_client =
        nh.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");
    left_ik_client =
        nh.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/left/PositionKinematicsNode/IKService");
    
    baxter_enabled.data = false;
 
    right_gripper_hid = 0;
    left_gripper_hid = 0;

    right_has_to_move = false;
    left_has_to_move = false;
}

BaxterController::~BaxterController()
{
    // disable Baxter
    baxter_enabled.data = false;
    enable_baxter_pub.publish(baxter_enabled);
}

void BaxterController::enable()
{
    cout << "Enable Baxter" << endl;
    // enable Baxter
    baxter_enabled.data = true;
    enable_baxter_pub.publish(baxter_enabled);
}

void BaxterController::rightEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    cur_right_pos[0] = msg->pose.position.x;
    cur_right_pos[1] = msg->pose.position.y;
    cur_right_pos[2] = msg->pose.position.z;

    cur_right_ori[0] = msg->pose.orientation.x;
    cur_right_ori[1] = msg->pose.orientation.y;
    cur_right_ori[2] = msg->pose.orientation.z;
    cur_right_ori[3] = msg->pose.orientation.w;

#if 0
    cout << right << "right hand positions: " <<
        cur_right_pos[0] << ","  << cur_right_pos[1] << "," << cur_right_pos[2] << endl;
    cout << right << "right hand orientations: " <<
        cur_right_ori[0] << "," << cur_right_ori[1] << "," << cur_right_ori[2] << "," << cur_right_ori[3] << endl;
#endif

    if (right_has_to_move)
        right_joint_pub.publish(right_joint_cmd);
}

void BaxterController::leftEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    cur_left_pos[0] = msg->pose.position.x;
    cur_left_pos[1] = msg->pose.position.y;
    cur_left_pos[2] = msg->pose.position.z;

    cur_left_ori[0] = msg->pose.orientation.x;
    cur_left_ori[1] = msg->pose.orientation.y;
    cur_left_ori[2] = msg->pose.orientation.z;
    cur_left_ori[3] = msg->pose.orientation.w;

#if 0
    cout << right << "left hand positions: " <<
        cur_left_pos[0] << ","  << cur_left_pos[1] << "," << cur_left_pos[2] << endl;
    cout << right << "left hand orientations: " <<
        cur_left_ori[0] << "," << cur_left_ori[1] << "," << cur_left_ori[2] << "," << cur_left_ori[3] << endl;
#endif

    if (left_has_to_move)
        left_joint_pub.publish(left_joint_cmd);
}

void BaxterController::rightGripCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg)
{
    if (right_gripper_hid == 0) {
        right_gripper_hid = msg->id;
        baxter_core_msgs::EndEffectorCommand cmd;
        cmd.id = right_gripper_hid;
        cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
        right_gripper_pub.publish(cmd);
    }
}

void BaxterController::leftGripCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg)
{
    if (left_gripper_hid == 0) {
        left_gripper_hid = msg->id;
        baxter_core_msgs::EndEffectorCommand cmd;
        cmd.id = left_gripper_hid;
        cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
        left_gripper_pub.publish(cmd);
    }
}

int BaxterController::moveRightHandTo(double set_pos[])
{
    cout << right << "right hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;

    baxter_core_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "base";
    pose_stamped.pose.position.x = set_pos[0];
    pose_stamped.pose.position.y = set_pos[1];
    pose_stamped.pose.position.z = set_pos[2];
    pose_stamped.pose.orientation.x = cur_right_ori[0];
    pose_stamped.pose.orientation.y = cur_right_ori[1];
    pose_stamped.pose.orientation.z = cur_right_ori[2];
    pose_stamped.pose.orientation.w = cur_right_ori[3];
    srv.request.pose_stamp.push_back(geometry_msgs::PoseStamped());
    srv.request.pose_stamp.push_back(pose_stamped);

    if (!right_ik_client.call(srv)) {
       cout << "\033[1;31m[right hand]Call to inverse kinematic solver service failed\033[0m" << endl;
       return -1;
    } else if(!srv.response.isValid[1]) {
       cout << "\033[1;31m[right hand]Inverse kinematic solver found no solution for that movement\033[0m" << endl;
       return -1;
    } else {
       right_has_to_move = false;
       right_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       right_joint_cmd.names = srv.response.joints[1].name;
       right_joint_cmd.command = srv.response.joints[1].position;
       right_has_to_move = true;
    }

    return 0;
}

int BaxterController::moveRightHandTo(double set_pos[], double set_ori[])
{
    cout << right << "right hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;
    cout << right << "right hand orientations: " <<
        set_ori[0] << "," << set_ori[1] << "," << set_ori[2] << "," << set_ori[3] << endl;

    baxter_core_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "base";
    pose_stamped.pose.position.x = set_pos[0];
    pose_stamped.pose.position.y = set_pos[1];
    pose_stamped.pose.position.z = set_pos[2];
    pose_stamped.pose.orientation.x = set_ori[0];
    pose_stamped.pose.orientation.y = set_ori[1];
    pose_stamped.pose.orientation.z = set_ori[2];
    pose_stamped.pose.orientation.w = set_ori[3];
    srv.request.pose_stamp.push_back(geometry_msgs::PoseStamped());
    srv.request.pose_stamp.push_back(pose_stamped);

    if (!right_ik_client.call(srv)) {
       cout << "\033[1;31m[right hand]Call to inverse kinematic solver service failed\033[0m" << endl;
       return -1;
    } else if(!srv.response.isValid[1]) {
       cout << "\033[1;31m[right hand]Inverse kinematic solver found no solution for that movement\033[0m" << endl;
       return -1;
    } else {
       right_has_to_move = false;
       right_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       right_joint_cmd.names = srv.response.joints[1].name;
       right_joint_cmd.command = srv.response.joints[1].position;
       right_has_to_move = true;
    }

    return 0;
}

int BaxterController::moveLeftHandTo(double set_pos[])
{
    cout << right << "left hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;

    baxter_core_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "base";
    pose_stamped.pose.position.x = set_pos[0];
    pose_stamped.pose.position.y = set_pos[1];
    pose_stamped.pose.position.z = set_pos[2];
    pose_stamped.pose.orientation.x = cur_left_ori[0];
    pose_stamped.pose.orientation.y = cur_left_ori[1];
    pose_stamped.pose.orientation.z = cur_left_ori[2];
    pose_stamped.pose.orientation.w = cur_left_ori[3];
    srv.request.pose_stamp.push_back(geometry_msgs::PoseStamped());
    srv.request.pose_stamp.push_back(pose_stamped);

    if (!left_ik_client.call(srv)) {
       cout << "\033[1;31m[left hand]Call to inverse kinematic solver service failed\033[0m" << endl;
       return -1;
    } else if(!srv.response.isValid[1]) {
       cout << "\033[1;31m[left hand]Inverse kinematic solver found no solution for that movement\033[0m" << endl;
       return -1;
    } else {
       left_has_to_move = false;
       left_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       left_joint_cmd.names = srv.response.joints[1].name;
       left_joint_cmd.command = srv.response.joints[1].position;
       left_has_to_move = true;
    }

    return 0;
}

int BaxterController::moveLeftHandTo(double set_pos[], double set_ori[])
{
    cout << right << "left hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;
    cout << right << "left hand orientations: " <<
        set_ori[0] << "," << set_ori[1] << "," << set_ori[2] << "," << set_ori[3] << endl;

    baxter_core_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped pose_stamped;

    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "base";
    pose_stamped.pose.position.x = set_pos[0];
    pose_stamped.pose.position.y = set_pos[1];
    pose_stamped.pose.position.z = set_pos[2];
    pose_stamped.pose.orientation.x = set_ori[0];
    pose_stamped.pose.orientation.y = set_ori[1];
    pose_stamped.pose.orientation.z = set_ori[2];
    pose_stamped.pose.orientation.w = set_ori[3];
    srv.request.pose_stamp.push_back(geometry_msgs::PoseStamped());
    srv.request.pose_stamp.push_back(pose_stamped);

    if (!left_ik_client.call(srv)) {
       cout << "\033[1;31m[left hand]Call to inverse kinematic solver service failed\033[0m" << endl;
       return -1;
    } else if(!srv.response.isValid[1]) {
       cout << "\033[1;31m[left hand]Inverse kinematic solver found no solution for that movement\033[0m" << endl;
       return -1;
    } else {
       left_has_to_move = false;
       left_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       left_joint_cmd.names = srv.response.joints[1].name;
       left_joint_cmd.command = srv.response.joints[1].position;
       left_has_to_move = true;
    }

    return 0;
}

int BaxterController::right_grip()
{
    cout << "right_grip" << endl;
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = right_gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
    right_gripper_pub.publish(cmd);

    return 0;
}

int BaxterController::right_release()
{
    cout << "right_release" << endl;
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = right_gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    right_gripper_pub.publish(cmd);

    return 0;
}

int BaxterController::left_grip()
{
    cout << "left_grip" << endl;
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = left_gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
    left_gripper_pub.publish(cmd);

    return 0;
}

int BaxterController::left_release()
{
    cout << "left_release" << endl;
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = left_gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    left_gripper_pub.publish(cmd);

    return 0;
}
