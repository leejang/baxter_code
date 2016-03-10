/*
 * File: baxter_cameras.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "baxter_cameras.h"

using namespace std;

BaxterCameras::BaxterCameras(ros::NodeHandle nh)
{
    this->nh = nh;

    // Close all cameras
    // because we only can use two Baxter cameras at once
    close_camera.request.name = "head_camera";
    cout << setw(80) << left << "Closing head camera: ";

    if (!ros::service::call("/cameras/close", close_camera) || open_camera.response.err != 0) {
        cout << right << "\033[1;31m[Failed]\033[0m" << endl;
        return;
    } else {
        cout << right << "\033[1;32m[OK]\033[0m" << endl;
    }

    close_camera.request.name = "left_hand_camera";
    cout << setw(80) << left << "Closing left hand camera: ";
    if (!ros::service::call("/cameras/close", close_camera) || open_camera.response.err != 0) {
        cout << right << "\033[1;31m[Failed]\033[0m" << endl;
        return;
    } else {
        cout << right << "\033[1;32m[OK]\033[0m" << endl;
    }

    close_camera.request.name = "right_hand_camera";
    cout << setw(80) << left << "Closing right hand camera: ";
    if (!ros::service::call("/cameras/close", close_camera) || open_camera.response.err != 0) {
        cout << right << "\033[1;31m[Failed]\033[0m" << endl;
        return;
    } else {
        cout << right << "\033[1;32m[OK]\033[0m" << endl;
    }

}

BaxterCameras::~BaxterCameras()
{
    if (it_head)
        delete it_head;

    if (it_l_hand)
        delete it_l_hand;
 
    if (it_r_hand)
        delete it_r_hand;
}

void BaxterCameras::open(CameraId id, CameraParams cam_param)
{
    // Open the camera
    if (id == HEAD) {
        open_camera.request.name = "head_camera";
        cout << setw(80) << left << "Opening head camera: ";
    } else if (id == LEFT_HAND) {
        open_camera.request.name = "left_hand_camera";
        cout << setw(80) << left << "Opening left hand camera: ";
    } else if (id == RIGHT_HAND) {
        open_camera.request.name = "right_hand_camera";
        cout << setw(80) << left << "Opening right hand camera: ";
    } else {
        cout << right << "\033[1;31m[Failed]: Check the camera ID\033[0m" << endl; 
        return;
    }

    open_camera.request.settings.width = cam_param.width;
    open_camera.request.settings.height = cam_param.height;
    open_camera.request.settings.fps = cam_param.fps;
    camera_control.id = baxter_core_msgs::CameraControl::CAMERA_CONTROL_GAIN;
    camera_control.value = 10;
    open_camera.request.settings.controls.push_back(camera_control);

    if (!ros::service::call("/cameras/open", open_camera) || open_camera.response.err != 0) {
        cout << right << "\033[1;31m[Failed]\033[0m" << endl;
        return;
    } else {
        cout << right << "\033[1;32m[OK]\033[0m" << endl;
    }

    // Register the camera callback that receive the images
    cout << setw(80) << left << "Registering camera callback: ";

    if (id == HEAD) {
        //cv::namedWindow("Baxter Head Cam");
        it_head = new image_transport::ImageTransport(nh);
        img_sub_head = it_head->subscribe("/cameras/head_camera/image", 1, &BaxterCameras::headCB, this);
    } else if (id == LEFT_HAND) {
        //cv::namedWindow("Baxter L-Hand Cam");
        it_l_hand = new image_transport::ImageTransport(nh);
        img_sub_l_hand = it_l_hand->subscribe("/cameras/left_hand_camera/image", 1, &BaxterCameras::leftHandCB, this);
    } else if (id == RIGHT_HAND) {
        //cv::namedWindow("Baxter R-Hand Cam");
        it_r_hand = new image_transport::ImageTransport(nh);
        img_sub_r_hand = it_r_hand->subscribe("/cameras/right_hand_camera/image", 1, &BaxterCameras::rightHandCB, this);
    } else {
        cout << right << "\033[1;31m[Failed]: Check the camera ID\033[0m" << endl; 
        return;
    }
     
    if (id == HEAD && img_sub_head == NULL) {
        cout << right << "\033[1;31m[Failed]\033[0m" << endl;
        return;
    } else if (id == LEFT_HAND && img_sub_l_hand == NULL) {
        cout << right << "\033[1;31m[Failed]\033[0m" << endl;
        return;
    } else if (id == RIGHT_HAND && img_sub_r_hand == NULL) {
        cout << right << "\033[1;31m[Failed]\033[0m" << endl;
        return;
    } else {
        cout << right << "\033[1;32m[OK]\033[0m" << endl;
    }
}

void BaxterCameras::headCB(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }

    cv::imshow("Baxter Head Cam", cv_ptr->image);
    cv::waitKey(1);
}

void BaxterCameras::leftHandCB(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }

    cv::imshow("Baxter L-Hand Cam", cv_ptr->image);
    cv::waitKey(1);
}

void BaxterCameras::rightHandCB(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }

    cv::imshow("Baxter R-Hand Cam", cv_ptr->image);
    cv::waitKey(1);
}

