/*
 * File: baxter_moveit_controller.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "baxter_moveit_controller.h"

#define DEBUG 0
#define NUM_OF_JOINTS_PER_EACH_ARM 7

using namespace std;

template <typename T, std::size_t N>
inline std::size_t size_of_array( T (&)[N] ) {
   return N;
}

BaxterMoveitController::BaxterMoveitController(ros::NodeHandle nh)
{
    this->nh = nh;

    // set up for Moveit! (both arms)
    both_arms_move_group = new move_group_interface::MoveGroup("both_arms");
    left_arm_move_group = new move_group_interface::MoveGroup("left_arm");
    right_arm_move_group = new move_group_interface::MoveGroup("right_arm");

#if DEBUG
    ROS_INFO("[both] Planning Frame: %s", both_arms_move_group->getPlanningFrame().c_str());
    ROS_INFO("[both] End Effector Link: %s", both_arms_move_group->getEndEffectorLink().c_str());
    ROS_INFO("[both] End Effector: %s", both_arms_move_group->getEndEffector().c_str());

    ROS_INFO("[left] Planning Frame: %s", left_arm_move_group->getPlanningFrame().c_str());
    ROS_INFO("[left] End Effector Link: %s", left_arm_move_group->getEndEffectorLink().c_str());
    ROS_INFO("[left] End Effector: %s", left_arm_move_group->getEndEffector().c_str());

    ROS_INFO("[right] Planning Frame: %s", right_arm_move_group->getPlanningFrame().c_str());
    ROS_INFO("[right] End Effector Link: %s", right_arm_move_group->getEndEffectorLink().c_str());
    ROS_INFO("[right] End Effector: %s", right_arm_move_group->getEndEffector().c_str());
#endif

    both_arms_move_group->setGoalJointTolerance(0.1);
    left_arm_move_group->setGoalJointTolerance(0.1);
    right_arm_move_group->setGoalJointTolerance(0.1);

    both_arms_move_group->setPlanningTime(30.0);
    left_arm_move_group->setPlanningTime(30.0);
    right_arm_move_group->setPlanningTime(30.0);

#if DEBUG
    ROS_INFO_STREAM("[both] Joint tolerance: " << both_arms_move_group->getGoalJointTolerance());
    ROS_INFO_STREAM("[left] Joint tolerance: " << left_arm_move_group->getGoalJointTolerance());
    ROS_INFO_STREAM("[right] Joint tolerance: " << right_arm_move_group->getGoalJointTolerance());
#endif

    // subscribers
    // endpoints
    right_end_sub =
        nh.subscribe("/robot/limb/right/endpoint_state", 2, &BaxterMoveitController::rightEndCallback, this);
    left_end_sub =
        nh.subscribe("/robot/limb/left/endpoint_state", 2, &BaxterMoveitController::leftEndCallback, this);

    // publishers
    display_pub =
        nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

#if 0
    // move both arms to the random position for test!
    both_arms_move_group->setRandomTarget();
    both_arms_move_group->move();
#endif

    // neutral positions
    double init_values[] = {
        // left arm (7 joints)
        0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0,
        // right arm (7 joints)
        0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0};
    both_arms_neutral_positions.insert(both_arms_neutral_positions.end(), &init_values[0], &init_values[size_of_array(init_values)]);
}

BaxterMoveitController::~BaxterMoveitController()
{
    delete both_arms_move_group;
    delete left_arm_move_group;
    delete right_arm_move_group;
}

int BaxterMoveitController::moveToNeutral()
{
    cout << "move To Neutral with Moveit!" << endl;

    both_arms_move_group->setJointValueTarget(both_arms_neutral_positions);
    both_arms_move_group->move();

#if 0
    const robot_state::JointModelGroup* joint_model_group =
        both_arms_move_group->getCurrentState()->getJointModelGroup(both_arms_move_group->getName());
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    std::vector<double> joint_values;
    both_arms_move_group->getCurrentState()->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i = 0; i < joint_names.size(); ++i)
    {
        cout << "Joint: " <<joint_names[i].c_str() << " value: " << joint_values[i] << endl;
    }
#endif
    return 0;
}

int BaxterMoveitController::moveLeftHandTo(geometry_msgs::Point set_point)
{
    //cout << "move To Left Hand to (point)!" << endl;
    left_arm_move_group->setStartStateToCurrentState();

    target_pose_left.position.x = set_point.x;
    target_pose_left.position.y = set_point.y;
    target_pose_left.position.z = set_point.z;

    target_pose_left.orientation.x = cur_pose_left.orientation.x;
    target_pose_left.orientation.y = cur_pose_left.orientation.y;
    target_pose_left.orientation.z = cur_pose_left.orientation.z;
    target_pose_left.orientation.w = cur_pose_left.orientation.w;

    left_arm_move_group->setPoseTarget(target_pose_left);
    left_arm_move_group->setGoalTolerance(0.1); // 10 cm
    left_arm_move_group->move();

    return 0;
}

int BaxterMoveitController::moveLeftHandTo(geometry_msgs::Pose set_pose)
{
    //cout << "move To Left Hand to (pose)!" << endl;
    left_arm_move_group->setStartStateToCurrentState();

    target_pose_left.position.x = set_pose.position.x;
    target_pose_left.position.y = set_pose.position.y;
    target_pose_left.position.z = set_pose.position.z;

    target_pose_left.orientation.x = set_pose.orientation.x;
    target_pose_left.orientation.y = set_pose.orientation.y;
    target_pose_left.orientation.z = set_pose.orientation.z;
    target_pose_left.orientation.w = set_pose.orientation.w;

    left_arm_move_group->setPoseTarget(target_pose_left);
    left_arm_move_group->setGoalTolerance(0.1); // 10 cm
    left_arm_move_group->move();

    return 0;
}

int BaxterMoveitController::moveRightHandTo(geometry_msgs::Point set_point)
{
    //cout << "move To Right Hand to (point)!" << endl;
    right_arm_move_group->setStartStateToCurrentState();

    target_pose_right.position.x = set_point.x;
    target_pose_right.position.y = set_point.y;
    target_pose_right.position.z = set_point.z;

    target_pose_right.orientation.x = cur_pose_left.orientation.x;
    target_pose_right.orientation.y = cur_pose_left.orientation.y;
    target_pose_right.orientation.z = cur_pose_left.orientation.z;
    target_pose_right.orientation.w = cur_pose_left.orientation.w;

    right_arm_move_group->setPoseTarget(target_pose_right);
    right_arm_move_group->setGoalTolerance(0.01); // 1 cm
    right_arm_move_group->move();

    return 0;
}

int BaxterMoveitController::moveRightHandTo(geometry_msgs::Pose set_pose)
{
    //cout << "move To Right Hand to (pose)!" << endl;
    right_arm_move_group->setStartStateToCurrentState();

    target_pose_right.position.x = set_pose.position.x;
    target_pose_right.position.y = set_pose.position.y;
    target_pose_right.position.z = set_pose.position.z;

    target_pose_right.orientation.x = set_pose.orientation.x;
    target_pose_right.orientation.y = set_pose.orientation.y;
    target_pose_right.orientation.z = set_pose.orientation.z;
    target_pose_right.orientation.w = set_pose.orientation.w;

    right_arm_move_group->setPoseTarget(target_pose_right);
    right_arm_move_group->setGoalTolerance(0.01); // 1 cm
    right_arm_move_group->move();

    return 0;
}

int BaxterMoveitController::moveLeftToJointPositions(double set_positions[])
{
    std::vector<double> joint_positions;
    joint_positions.insert(joint_positions.end(), &set_positions[0], &set_positions[NUM_OF_JOINTS_PER_EACH_ARM]);

    // left arm
    left_arm_move_group->setStartStateToCurrentState();
    left_arm_move_group->setJointValueTarget(joint_positions);
    left_arm_move_group->move();

    return 0;
}

int BaxterMoveitController::moveRightToJointPositions(double set_positions[])
{
    std::vector<double> joint_positions;
    joint_positions.insert(joint_positions.end(), &set_positions[0], &set_positions[NUM_OF_JOINTS_PER_EACH_ARM]);

    // right arm
    right_arm_move_group->setStartStateToCurrentState();
    right_arm_move_group->setJointValueTarget(joint_positions);
    right_arm_move_group->move();

    return 0;
}

int BaxterMoveitController::addObjectIntoWorld(moveit_msgs::CollisionObject object)
{
    cout << "Add an object into the world" << endl;

    collision_objects.push_back(object);

    plan_scene_interface.addCollisionObjects(collision_objects);

    /* Sleep so we have time to see the object in RViz */
    sleep(2.0);

#if 0
    // to make a link between object and motion group
    cout << object.id << endl;
    left_arm_move_group->attachObject("table");
    sleep(4.0);
#endif

    return 0;
}

int BaxterMoveitController::removeObjectFromWorld(std::string object_id)
{ 
    std::vector<std::string> objects;
    objects.push_back(object_id);

    cout << "Remove an object from the world" << endl;

    plan_scene_interface.removeCollisionObjects(objects);

    /* Sleep so we have time to see the object in RViz */
    sleep(3.0);
}

int BaxterMoveitController::setPlanningTime(std::string arm, double time)
{
    if (arm == "left")
        //cout << "left: " << time << endl;
        left_arm_move_group->setPlanningTime(time);
    else if (arm == "right")
        //cout << "right: " << time << endl;
        right_arm_move_group->setPlanningTime(time);
    else if (arm == "both")
        //cout << "both: " << time << endl;
        both_arms_move_group->setPlanningTime(time);
    else
        ROS_ERROR("Choose correct type of arm(left, right, both)!");

}

//////////////////////////////////////////////////
// Callback functions
//////////////////////////////////////////////////
void BaxterMoveitController::leftEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    cur_pose_left.position.x = msg->pose.position.x;
    cur_pose_left.position.y = msg->pose.position.y;
    cur_pose_left.position.z = msg->pose.position.z;

    cur_pose_left.orientation.x = msg->pose.orientation.x;
    cur_pose_left.orientation.y = msg->pose.orientation.y;
    cur_pose_left.orientation.z = msg->pose.orientation.z;
    cur_pose_left.orientation.w = msg->pose.orientation.w;
#if DEBUG
    cout << right << "left hand positions: " <<
        cur_pose_left.position.x << ","  << cur_pose_left.position.y << "," << cur_pose_left.position.z << endl;
    cout << right << "left hand orientations: " <<
        cur_pose_left.orientation.x << "," << cur_pose_left.orientation.y << "," <<
        cur_pose_left.orientation.z << "," << cur_pose_left.orientation.w << endl;
#endif
}

void BaxterMoveitController::rightEndCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    cur_pose_right.position.x = msg->pose.position.x;
    cur_pose_right.position.y = msg->pose.position.y;
    cur_pose_right.position.z = msg->pose.position.z;

    cur_pose_right.orientation.x = msg->pose.orientation.x;
    cur_pose_right.orientation.y = msg->pose.orientation.y;
    cur_pose_right.orientation.z = msg->pose.orientation.z;
    cur_pose_right.orientation.w = msg->pose.orientation.w;
#if DEBUG
    cout << right << "right hand positions: " <<
        cur_pose_right.position.x << ","  << cur_pose_right.position.y << "," << cur_pose_right.position.z << endl;
    cout << right << "right hand orientations: " <<
        cur_pose_right.orientation.x << "," << cur_pose_right.orientation.y << "," <<
        cur_pose_right.orientation.z << "," << cur_pose_right.orientation.w << endl;
#endif
}

