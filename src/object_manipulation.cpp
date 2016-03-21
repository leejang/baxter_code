/*
 * File: object_manipulation.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */


#include <ros/ros.h>
#include "baxter_controller.h"
#include "baxter_cameras.h"
#include "object_detector.h"

using namespace std;

unsigned int mission_start = 0;

void detectedObjCB(const find_object_2d::ObjectsStampedConstPtr& msg)
{
    //ROS_INFO("detected Obj CB");
   
    mission_start++;
}

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "object_manipulation");
    ros::NodeHandle nh;

    ROS_INFO("object manipulation");

    ros::AsyncSpinner spinner(1);

    // Baxter controller
    BaxterController *controller = new BaxterController(nh);

    spinner.start();

    // Enable Baxter
    controller->enable();

#if 0
    // Open Baxter Cameras
    BaxterCameras *cameras =  new BaxterCameras(nh);

    BaxterCameras::CameraParams head_cam_set;
    BaxterCameras::CameraParams left_hand_cam_set;

    head_cam_set.width = 1280;
    head_cam_set.height = 600;
    head_cam_set.fps = 10;
    cameras->open(BaxterCameras::HEAD, head_cam_set);

    left_hand_cam_set.width = 640;
    left_hand_cam_set.height = 480;
    left_hand_cam_set.fps = 30;
    cameras->open(BaxterCameras::LEFT_HAND, left_hand_cam_set);
#endif
    ObjectDetector *obj_detector = new ObjectDetector(nh);
    ros::Subscriber sub = nh.subscribe("objectsStamped/pose", 1, detectedObjCB);
 
    // test
    bool test = true;
    char input;
    double set_right_pos[3] = {};
    double set_right_ori[4] = {};

    double set_left_pos[3] = {};
    double set_left_ori[4] = {};

    char r_grip;
    char l_grip;

    cout << "waiting mission start signal.." << endl;
    while (mission_start < 5) { }

    set_left_pos[0] = obj_detector->pose.getOrigin().x();
    set_left_pos[1] = obj_detector->pose.getOrigin().y();
    set_left_pos[2] = obj_detector->pose.getOrigin().z();
#if 0
    set_left_ori[0] = obj_detector->pose.getRotation().x();
    set_left_ori[1] = obj_detector->pose.getRotation().y();
    set_left_ori[2] = obj_detector->pose.getRotation().z();
    set_left_ori[3] = obj_detector->pose.getRotation().w();
#endif
    set_left_ori[0] = 0.5072;
    set_left_ori[1] = -0.5125;
    set_left_ori[2] = 0.4321;
    set_left_ori[3] = 0.5415;

#if 0
    for (int i=0; i<3; i++) {
        cout << "Set right positions[" << i << "]" << endl;
        cin >> set_right_pos[i];
    }

    for (int i=0; i<4; i++) {
        cout << "Set right orientations[" << i << "]" <<endl;
        cin >> set_right_ori[i];
    }

    controller->moveRightHandTo(set_right_pos, set_right_ori);
#endif

    for (int i=0; i<3; i++) {
        cout << "Set left positions[" << i << "]" << endl;
        cout << set_left_pos[i] << endl;
        //cin >> set_left_pos[i];
    }

    for (int i=0; i<4; i++) {
        cout << "Set left orientations[" << i << "]" <<endl;
        cout << set_left_ori[i] << endl;
        //cin >> set_left_ori[i];
    }

    controller->moveLeftHandTo(set_left_pos, set_left_ori);

    while (test) {
        cout << "Type 'Q' if you want to stop" << endl;
        cin >> input;

        if (input == 'Q')
            test = false;
        else {
            cout << "Type 'G' if you want to grip (right)" << endl;
            cout << "Type 'R' if you want to release (right)" << endl;
            cin >> r_grip;

            if (r_grip == 'G')
                controller->right_grip();
            else if (r_grip == 'R')
                controller->right_release();

            cout << "Type 'g' if you want to grip (left)" << endl;
            cout << "Type 'r' if you want to release (left)" << endl;
            cin >> l_grip;

            if (l_grip == 'g')
                controller->left_grip();
            else if (l_grip == 'r')
                controller->left_release();
        }
    }

    cout << "stop!" << endl;

    spinner.stop();

#if 0
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
#endif
 

    delete controller;
    //delete cameras;
    delete obj_detector;

    // shutdown ROS
    ros::shutdown();

    return 0;
}

