/*
 * File: baxter_moveit_motion.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include "baxter_moveit_motion.h"
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>

#define CONTROL_FREQ 30.0 // Hz
#define DEBUG 0

using namespace std;

BaxterMoveitMotion::BaxterMoveitMotion(ros::NodeHandle nh)
{
    this->nh = nh;

    // Baxter moveit controller
    baxter_moveit_ctrl = new BaxterMoveitController(nh);
}

BaxterMoveitMotion::~BaxterMoveitMotion()
{
    delete baxter_moveit_ctrl;
}

void BaxterMoveitMotion::onTimerTick(const ros::TimerEvent& e)
{
    //ROS_INFO("BaxterMoveitMotion::OnTimerTick");

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
        ros::Duration(0.2).sleep();
        tf_exist = false;
    }

    if (tf_exist) {
        // left Hand
        left_hand_pt = tf_left_hand.getOrigin();
        // right Hand
        right_hand_pt = tf_right_hand.getOrigin();

#if 0
        cout << "Left Hand Pt: " << left_hand_pt.x() << ", "
             << left_hand_pt.y() << ", " << left_hand_pt.z() << endl;

        cout << "Right Hand Pt: " << right_hand_pt.x() << ", "
             << right_hand_pt.y() << ", " << right_hand_pt.z() << endl;
#endif

        // mapping user hand position to baxter's end effetor position
        // x, y, z respectively
        target_point_left.x = left_hand_pt.x();
        target_point_left.y = left_hand_pt.y();
        target_point_left.z = left_hand_pt.z();

        // send commands to move hands
        //baxter_moveit_ctrl->moveLeftHandTo(target_point_left);
    }

}

void BaxterMoveitMotion::followHands(void)
{
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
        ros::Duration(0.2).sleep();
        tf_exist = false;
    }

    if (tf_exist) {
        // left Hand
        left_hand_pt = tf_left_hand.getOrigin();
        // right Hand
        right_hand_pt = tf_right_hand.getOrigin();

        cout << "Left Hand Pt: " << left_hand_pt.x() << ", "
             << left_hand_pt.y() << ", " << left_hand_pt.z() << endl;

#if 0
        cout << "Left Hand Pt: " << left_hand_pt.x() << ", "
        cout << "Right Hand Pt: " << right_hand_pt.x() << ", "
             << right_hand_pt.y() << ", " << right_hand_pt.z() << endl;
#endif

        // mapping user hand position to baxter's end effetor position
        // x, y, z respectively
        target_point_left.x = left_hand_pt.x();
        target_point_left.y = left_hand_pt.y();
        target_point_left.z = left_hand_pt.z();

        // send commands to move hands
        baxter_moveit_ctrl->moveLeftHandTo(target_point_left);
    }
}

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "baxter_moveit_motion");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    // moveit motion node
    BaxterMoveitMotion baxter_moveit_motion(nh);

    ROS_INFO("Baxter Moveit Motion! (demo)");

    // control timer
    ros::Timer timer;
    ros::Rate r(CONTROL_FREQ);

#if 0 
    timer = nh.createTimer(ros::Duration(1/CONTROL_FREQ),
                               boost::bind(&BaxterMoveitMotion::onTimerTick, &baxter_moveit_motion, _1));
#endif
    // move both arms to neutral positions
    baxter_moveit_motion.baxter_moveit_ctrl->moveToNeutral();

    // adding objects into the environment
    // object1: table
    moveit_msgs::CollisionObject table;
    table.header.frame_id = "/base";

    /* The id of the object is used to identify it. */
    table.id = "table";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.762; // 30 inches (around 0.762 meter)
    table_primitive.dimensions[1] = 1.524; // 60 inches
    table_primitive.dimensions[2] = 0.762; // 30 inchea

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose table_pose;
    table_pose.position.x =  1.0;
    table_pose.position.y =  0.0;
    table_pose.position.z =  -0.508; // 20 inches
    table_pose.orientation.w = 1.0;

    table.primitives.push_back(table_primitive);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD; 
    baxter_moveit_motion.baxter_moveit_ctrl->addObjectIntoWorld(table);

    // object2: Kinect sensor
    moveit_msgs::CollisionObject kinect;
    kinect.header.frame_id = "/base";

    /* The id of the object is used to identify it. */
    kinect.id = "kinect";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive kinect_primitive;
    kinect_primitive.type = kinect_primitive.BOX;
    kinect_primitive.dimensions.resize(3);
    kinect_primitive.dimensions[0] = 0.127; // 5 inches
    kinect_primitive.dimensions[1] = 0.305; // 12 inches
    kinect_primitive.dimensions[2] = 0.127; // 5 inches

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose kinect_pose;
    kinect_pose.position.x =  0.280;
    kinect_pose.position.y =  0.0;
    kinect_pose.position.z =  0.230;
    kinect_pose.orientation.w = 1.0;

    kinect.primitives.push_back(kinect_primitive);
    kinect.primitive_poses.push_back(kinect_pose);
    kinect.operation = kinect.ADD; 
    baxter_moveit_motion.baxter_moveit_ctrl->addObjectIntoWorld(kinect);

#if 0
    // test for moving each arm
    baxter_moveit_motion.target_point_left.x = 0.892;
    baxter_moveit_motion.target_point_left.y = 0.260;
    baxter_moveit_motion.target_point_left.z = -0.001;
    baxter_moveit_motion.baxter_moveit_ctrl->moveLeftHandTo(baxter_moveit_motion.target_point_left);

    baxter_moveit_motion.target_pose_left.position.x = 1.129;
    baxter_moveit_motion.target_pose_left.position.y = 0.438;
    baxter_moveit_motion.target_pose_left.position.z = 0.128;
    baxter_moveit_motion.target_pose_left.orientation.x = -0.309;
    baxter_moveit_motion.target_pose_left.orientation.y = 0.838;
    baxter_moveit_motion.target_pose_left.orientation.z = -0.250;
    baxter_moveit_motion.target_pose_left.orientation.w = 0.373;
    baxter_moveit_motion.baxter_moveit_ctrl->moveLeftHandTo(baxter_moveit_motion.target_pose_left);

    baxter_moveit_motion.target_point_right.x = 0.862;
    baxter_moveit_motion.target_point_right.y = -0.471;
    baxter_moveit_motion.target_point_right.z = 0.469;
    baxter_moveit_motion.baxter_moveit_ctrl->moveRightHandTo(baxter_moveit_motion.target_point_right);

    baxter_moveit_motion.target_pose_right.position.x = 0.072;
    baxter_moveit_motion.target_pose_right.position.y = -1.445;
    baxter_moveit_motion.target_pose_right.position.z = 0.576;
    baxter_moveit_motion.target_pose_right.orientation.x = -0.403;
    baxter_moveit_motion.target_pose_right.orientation.y = -0.506;
    baxter_moveit_motion.target_pose_right.orientation.z = 0.592;
    baxter_moveit_motion.target_pose_right.orientation.w = -0.478;
    baxter_moveit_motion.baxter_moveit_ctrl->moveRightHandTo(baxter_moveit_motion.target_pose_right);
#endif

    double left_approach[] = {-0.1783, 0.0107, -0.8229, 0.4456, -0.1940, 1.012, -0.015};
    baxter_moveit_motion.baxter_moveit_ctrl->moveLeftToJointPositions(left_approach);

    baxter_moveit_motion.baxter_moveit_ctrl->setPlanningTime("left", 3.0);
    baxter_moveit_motion.baxter_moveit_ctrl->setPlanningTime("right", 3.0);
    baxter_moveit_motion.baxter_moveit_ctrl->setPlanningTime("both", 3.0);

    while(ros::ok())
    {
       baxter_moveit_motion.followHands();

       //ros::Duration(0.2).sleep();
       r.sleep();
    }

    //ros::spin();

    // removing objects from the environment
    baxter_moveit_motion.baxter_moveit_ctrl->removeObjectFromWorld("table");
    baxter_moveit_motion.baxter_moveit_ctrl->removeObjectFromWorld("kinect");

    spinner.stop();

    // shutdown ROS
    ros::shutdown();
    return 0;
}
