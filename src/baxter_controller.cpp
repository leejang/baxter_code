/*
 * File: baxter_controller.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "baxter_controller.h"

#define DEBUG 0

// position threshold in radians across each joint
// when move is considered successful
#define POSITION_THRESHOLD 0.008726646 // equals to 0.5 degree
#define CONTROL_FREQ 100
#define TIMEOUT 15 // sec
#define NUM_OF_ALL_JOINTS 17
#define NUM_OF_JOINTS_PER_EACH_ARM 7
#define INIT_TIMEOUT_CNT 60 * CONTROL_FREQ

using namespace std;

template <typename T, std::size_t N>
inline std::size_t size_of_array( T (&)[N] ) {
   return N;
}

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
    // joint state
    joint_state_sub =
        nh.subscribe("/robot/joint_states", 2, &BaxterController::jointStateCallback, this);

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

    // Control timer
    ctrl_freq = CONTROL_FREQ;
    ctrl_timer =
        nh.createTimer(ros::Duration(1/ctrl_freq), &BaxterController::onCtrlTimerTick, this);
    timeout = TIMEOUT * CONTROL_FREQ;
    right_timeout_cnt = INIT_TIMEOUT_CNT;
    left_timeout_cnt = INIT_TIMEOUT_CNT;

    baxter_enabled.data = false;

    right_gripper_hid = 0;
    left_gripper_hid = 0;

    right_has_to_move = false;
    left_has_to_move = false;

    // initialize left & right joint commands
    // joint names
    const char *right_joints[] =
               {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
    const char *left_joints[] =
               {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};

    right_joint_names.insert(right_joint_names.end(), &right_joints[0], &right_joints[size_of_array(right_joints)]);
    left_joint_names.insert(left_joint_names.end(), &left_joints[0], &left_joints[size_of_array(left_joints)]);

    // neutral positions
    double init_values[] = {0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};
    neutral_positions.insert(neutral_positions.end(), &init_values[0], &init_values[size_of_array(init_values)]);

    // move to neutral positions (both arms)
    moveToNeutral();
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

void BaxterController::setCtrlFreq(unsigned int new_freq)
{
   ctrl_freq = new_freq;
}

int BaxterController::moveToNeutral()
{
    cout << "move to neutral" << endl;

    // right arm
    right_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
    right_joint_cmd.names = right_joint_names;
    right_joint_cmd.command = neutral_positions;
    right_has_to_move = true;
    right_timeout_cnt = 0;

    // left arm
    left_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
    left_joint_cmd.names = left_joint_names;
    left_joint_cmd.command = neutral_positions;
    left_has_to_move = true;
    left_timeout_cnt = 0;

    return 0;
}

int BaxterController::setRightConstrainedPositions(double up_bounds[], double lo_bounds[])
{
    //cout << "set Right constrained positions" << endl;
    right_constrained_positions_up.insert(
        right_constrained_positions_up.end(), &up_bounds[0], &up_bounds[NUM_OF_JOINTS_PER_EACH_ARM]);

    right_constrained_positions_lo.insert(
        right_constrained_positions_lo.end(), &lo_bounds[0], &lo_bounds[NUM_OF_JOINTS_PER_EACH_ARM]);
    return 0;
}

int BaxterController::setLeftConstrainedPositions(double up_bounds[], double lo_bounds[])
{
    //cout << "set Left constrained positions" << endl;
    left_constrained_positions_up.insert(
        left_constrained_positions_up.end(), &up_bounds[0], &up_bounds[NUM_OF_JOINTS_PER_EACH_ARM]);

    left_constrained_positions_lo.insert(
        left_constrained_positions_lo.end(), &lo_bounds[0], &lo_bounds[NUM_OF_JOINTS_PER_EACH_ARM]);
    return 0;
}

int BaxterController::moveRightToJointPositions(double set_positions[])
{
    std::vector<double> joint_positions;
    joint_positions.insert(joint_positions.end(), &set_positions[0], &set_positions[NUM_OF_JOINTS_PER_EACH_ARM]);

    // right arm
    right_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
    right_joint_cmd.names = right_joint_names;
    right_joint_cmd.command = joint_positions;
    right_has_to_move = true;
    right_timeout_cnt = 0;

    return 0;
}

int BaxterController::moveLeftToJointPositions(double set_positions[])
{
    std::vector<double> joint_positions;
    joint_positions.insert(joint_positions.end(), &set_positions[0], &set_positions[NUM_OF_JOINTS_PER_EACH_ARM]);

    // left arm
    left_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
    left_joint_cmd.names = left_joint_names;
    left_joint_cmd.command = joint_positions;
    left_has_to_move = true;
    left_timeout_cnt = 0;

    return 0;
}

int BaxterController::moveRightHandTo(double set_pos[])
{
#if DEBUG
    cout << right << "right hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;
#endif

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
    } else if (!checkRightConstrainedPos(srv.response.joints[1].position)) {
       cout << "\033[1;31m[right hand]Found solution has constrained positions\033[0m" << endl;
       return -1;
    } else {
       right_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       right_joint_cmd.names = srv.response.joints[1].name;
       right_joint_cmd.command = srv.response.joints[1].position;
       right_has_to_move = true;
       right_timeout_cnt = 0;
    }

    return 0;
}

int BaxterController::moveRightHandTo(double set_pos[], double set_ori[])
{
#if DEBUG
    cout << right << "right hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;
    cout << right << "right hand orientations: " <<
        set_ori[0] << "," << set_ori[1] << "," << set_ori[2] << "," << set_ori[3] << endl;
#endif

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
    } else if (!checkRightConstrainedPos(srv.response.joints[1].position)) {
       cout << "\033[1;31m[right hand]Found solution has constrained positions\033[0m" << endl;
       return -1;
    } else {
       right_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       right_joint_cmd.names = srv.response.joints[1].name;
       right_joint_cmd.command = srv.response.joints[1].position;
       right_has_to_move = true;
       right_timeout_cnt = 0;
    }

    return 0;
}

int BaxterController::moveLeftHandTo(double set_pos[])
{
#if DEBUG
    cout << right << "left hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;
#endif

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
    } else if (!srv.response.isValid[1]) {
       cout << "\033[1;31m[left hand]Inverse kinematic solver found no solution for that movement\033[0m" << endl;
       return -1;
    } else if (!checkLeftConstrainedPos(srv.response.joints[1].position)) {
       cout << "\033[1;31m[left hand]Found solution has constrained positions\033[0m" << endl;
       return -1;
    } else {
       left_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       left_joint_cmd.names = srv.response.joints[1].name;
       left_joint_cmd.command = srv.response.joints[1].position;
       left_has_to_move = true;
       left_timeout_cnt = 0;
    }

    return 0;
}

int BaxterController::moveLeftHandTo(double set_pos[], double set_ori[])
{
#if DEBUG
    cout << right << "left hand positions: " <<
        set_pos[0] << ","  << set_pos[1] << "," << set_pos[2] << endl;
    cout << right << "left hand orientations: " <<
        set_ori[0] << "," << set_ori[1] << "," << set_ori[2] << "," << set_ori[3] << endl;
#endif

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
    } else if (!checkLeftConstrainedPos(srv.response.joints[1].position)) {
       cout << "\033[1;31m[left hand]Found solution has constrained positions\033[0m" << endl;
       return -1;
    } else {
       left_joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
       left_joint_cmd.names = srv.response.joints[1].name;
       left_joint_cmd.command = srv.response.joints[1].position;
       left_has_to_move = true;
       left_timeout_cnt = 0;
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

void BaxterController::onCtrlTimerTick(const ros::TimerEvent& e)
{
    // right limb
    if ((right_timeout_cnt < timeout) && right_has_to_move)  {
        //cout << "right timeout count: " << right_timeout_cnt << endl;

        // check the lim reaches commnded joint position
        if (checkRightJointDiff(right_joint_cmd.command))
            right_has_to_move = false;

        // move right limb
        right_joint_pub.publish(right_joint_cmd);
        // increat timeout_cnt
        right_timeout_cnt++;
    } else {
        right_timeout_cnt = INIT_TIMEOUT_CNT;
    }

    // right limb
    if ((left_timeout_cnt < timeout) && left_has_to_move)  {
        //cout << "left timeout count: " << right_timeout_cnt << endl;

        // check the lim reaches commnded joint position
        if (checkLeftJointDiff(left_joint_cmd.command))
            left_has_to_move = false;

        // move left limb
        left_joint_pub.publish(left_joint_cmd);
        // increat timeout_cnt
        left_timeout_cnt++;
    } else {
        left_timeout_cnt = INIT_TIMEOUT_CNT;
    }
}

bool BaxterController::checkRightJointDiff(std::vector<double> goal_positions)
{
    bool retVal = false;

    retVal = (abs(goal_positions[0] - current_positions["right_s0"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[1] - current_positions["right_s1"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[2] - current_positions["right_e0"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[3] - current_positions["right_e1"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[4] - current_positions["right_w0"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[5] - current_positions["right_w1"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[6] - current_positions["right_w2"]) < POSITION_THRESHOLD);

#if DEBUG
    cout << "[checkRightJointDiff] retVal: " << retVal << endl;
#endif

    return retVal;
}

bool BaxterController::checkLeftJointDiff(std::vector<double> goal_positions)
{
    bool retVal = false;

    retVal = (abs(goal_positions[0] - current_positions["left_s0"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[1] - current_positions["left_s1"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[2] - current_positions["left_e0"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[3] - current_positions["left_e1"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[4] - current_positions["left_w0"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[5] - current_positions["left_w1"]) < POSITION_THRESHOLD);
    retVal &= (abs(goal_positions[6] - current_positions["left_w2"]) < POSITION_THRESHOLD);

#if DEBUG
    cout << "[checkLeftJointDiff] retVal: " << retVal << endl;
#endif

    return retVal;
}

bool BaxterController::checkRightConstrainedPos(std::vector<double> positions_to_move)
{
    cout << "check right constrained pos" << endl;

    return true;
}

bool BaxterController::checkLeftConstrainedPos(std::vector<double> positions_to_move)
{
    cout << "check left constrained pos" << endl;

    // left_s0
    if (positions_to_move[0] < left_constrained_positions_lo[0])
        return false;
    // left_s1
    else if (positions_to_move[1] > left_constrained_positions_up[1])
        return false;

    return true;
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

#if DEBUG
    cout << right << "right hand positions: " <<
        cur_right_pos[0] << ","  << cur_right_pos[1] << "," << cur_right_pos[2] << endl;
    cout << right << "right hand orientations: " <<
        cur_right_ori[0] << "," << cur_right_ori[1] << "," << cur_right_ori[2] << "," << cur_right_ori[3] << endl;
#endif

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

#if DEBUG
    cout << right << "left hand positions: " <<
        cur_left_pos[0] << ","  << cur_left_pos[1] << "," << cur_left_pos[2] << endl;
    cout << right << "left hand orientations: " <<
        cur_left_ori[0] << "," << cur_left_ori[1] << "," << cur_left_ori[2] << "," << cur_left_ori[3] << endl;
#endif

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

void BaxterController::jointStateCallback(const sensor_msgs::JointStateConstPtr &msg)
{
    if (msg->name.size() == NUM_OF_ALL_JOINTS) {
        for (int i = 0; i < msg->name.size(); i++) {
            //cout << msg->name[i] << ": " << msg->position[i] << endl;
            current_positions[msg->name[i]] = msg->position[i];
        }
    }

#if DEBUG
    if (msg->name.size() == NUM_OF_ALL_JOINTS) {
        cout << msg->name[4] << ": " << current_positions["left_s0"] << endl;
        cout << msg->name[5] << ": " << current_positions["left_s1"] << endl;
        cout << msg->name[2] << ": " << current_positions["left_e0"] << endl;
        cout << msg->name[3] << ": " << current_positions["left_e1"] << endl;
        cout << msg->name[6] << ": " << current_positions["left_w0"] << endl;
        cout << msg->name[7] << ": " << current_positions["left_w1"] << endl;
        cout << msg->name[8] << ": " << current_positions["left_w2"] << endl;
    }
#endif
}
