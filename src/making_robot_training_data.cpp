/*
 * File: making_robot_training_data.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */


#include "making_robot_training_data.h"

using namespace std;

#define DEBUG 0

// Kinect Camera
#define ROS_CAM_RGB_COLOR_PATH "/camera/rgb/image_color"
#define ROBOT_TRAIN_TXT "rt_0215_raw.txt"

MakingRobotTrainingData::MakingRobotTrainingData(ros::NodeHandle nh)
{
    this->nh = nh;

    it = new image_transport::ImageTransport(nh);
    cam_sub = it->subscribeCamera(ROS_CAM_RGB_COLOR_PATH, 10, &MakingRobotTrainingData::onNewImageCB, this);
    robot_screen = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 1, true);
    joint_states_sub = nh.subscribe("/robot/joint_states", 2, &MakingRobotTrainingData::jointStatesCB, this);
    
    full_screen.image = cv::Mat(600, 1024, CV_8UC3);

    // intialize all hands/objects poses
    my_left_hand_pose.x = 0, my_left_hand_pose.y = 0;
    my_right_hand_pose.x = 0, my_right_hand_pose.y = 0;

    robot_train_d.open(ROBOT_TRAIN_TXT);

    cout << "Construction done!" <<endl;
}


MakingRobotTrainingData::~MakingRobotTrainingData()
{
    robot_train_d.close();

    delete it;
}

void MakingRobotTrainingData::onNewImageCB(const sensor_msgs::ImageConstPtr& image_msg,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cam_model.fromCameraInfo(info_msg);

    if (&image_msg == NULL) {
        ROS_WARN_THROTTLE(1,"NULL image received.");
        return;
    } else {

       try
       {
           cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
           ROS_ERROR("Image acquisition failed: %s", e.what());
           return;
       }

       try
       {
           // tfansform each body position to camera frame
           // get the transform from camera frame to the frame of each joint
           tfListener.lookupTransform(cam_model.tfFrame(), "/left_gripper_base", ros::Time(0), tf_my_left_hand);
           tfListener.lookupTransform(cam_model.tfFrame(), "/right_gripper_base", ros::Time(0), tf_my_right_hand);
       }
       catch (tf::TransformException & ex)
       {
            //ROS_WARN("%s",ex.what());
            return;
       }

       static const int RADIUS = 10;
       // Baxter's left hand
       my_left_hand_pt = tf_my_left_hand.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d my_left_hand_cv_3d_pt(my_left_hand_pt.x(), my_left_hand_pt.y(), my_left_hand_pt.z());
       cv::Point2d my_left_hand_cv_pt = cam_model.project3dToPixel(my_left_hand_cv_3d_pt);
       my_left_hand_pose.x = my_left_hand_cv_pt.x;
       my_left_hand_pose.y = my_left_hand_cv_pt.y;
       cv::circle(cv_ptr->image, my_left_hand_cv_pt, RADIUS, CV_RGB(255,255,0), -1);

       // Baxter's right hand
       my_right_hand_pt = tf_my_right_hand.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d my_right_hand_cv_3d_pt(my_right_hand_pt.x(), my_right_hand_pt.y(), my_right_hand_pt.z());
       cv::Point2d my_right_hand_cv_pt = cam_model.project3dToPixel(my_right_hand_cv_3d_pt);
       my_right_hand_pose.x = my_right_hand_cv_pt.x;
       my_right_hand_pose.y = my_right_hand_cv_pt.y;
       //cout << "x: " << my_right_hand_pose.x << ", y: " << my_right_hand_pose.y <<endl;
       cv::circle(cv_ptr->image, my_right_hand_cv_pt, RADIUS, CV_RGB(0,255,255), -1);

       CvPoint my_left_h_origin = cvPoint(my_left_hand_pose.x - 20, my_left_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "My Left Hand", my_left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,255,0));

       CvPoint my_right_h_origin = cvPoint(my_right_hand_pose.x - 20, my_right_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "My Right Hand", my_right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,255,255));

       // display on Baxter's screen
       full_screen.header = cv_ptr->header;
       full_screen.encoding = cv_ptr->encoding;
       cv::resize(cv_ptr->image, full_screen.image, full_screen.image.size(), 0, 0, cv::INTER_NEAREST);
       robot_screen.publish(full_screen.toImageMsg());

#if 0 
       for (int i = 0; i < NUM_OF_JOINTS; i++) {
           cout << "Left Joint States[" << i << "]: " << cur_left_joint_states[i] << endl;
           cout << "Right Joint States[" << i << "]: " << cur_right_joint_states[i] << endl;
       }
#endif
    } // else (image_msg == NULL)

    robot_train_d <<  my_left_hand_pose.x << " " <<  my_left_hand_pose.y << " ";
    robot_train_d <<  my_right_hand_pose.x << " " <<  my_right_hand_pose.y << " ";
    robot_train_d <<  cur_left_joint_states[0] << " " << cur_left_joint_states[1] << " ";
    robot_train_d <<  cur_left_joint_states[2] << " " << cur_left_joint_states[3] << " ";
    robot_train_d <<  cur_left_joint_states[4] << " " << cur_left_joint_states[5] << " ";
    robot_train_d <<  cur_left_joint_states[6] << " " << " ";
    robot_train_d <<  cur_right_joint_states[0] << " " << cur_right_joint_states[1] << " ";
    robot_train_d <<  cur_right_joint_states[2] << " " << cur_right_joint_states[3] << " ";
    robot_train_d <<  cur_right_joint_states[4] << " " << cur_right_joint_states[5] << " ";
    robot_train_d <<  cur_right_joint_states[6] << " " << endl;
}

void MakingRobotTrainingData::jointStatesCB(const sensor_msgs::JointStateConstPtr& joint_state_msg)
{
    if (&joint_state_msg == NULL) {
        ROS_WARN_THROTTLE(1,"NULL image received.");
        return;
    } else {

        if (joint_state_msg->name[0] == "head_nod") {
            // left_e0
            cur_left_joint_states[0] = joint_state_msg->position[2];
            // left_e1
            cur_left_joint_states[1] = joint_state_msg->position[3];
            // left_s0
            cur_left_joint_states[2] = joint_state_msg->position[4];
            // left_s1
            cur_left_joint_states[3] = joint_state_msg->position[5];
            // left_w0
            cur_left_joint_states[4] = joint_state_msg->position[6];
            // left_w1
            cur_left_joint_states[5] = joint_state_msg->position[7];
            // left_w1
            cur_left_joint_states[6] = joint_state_msg->position[8];

            // right_e0
            cur_right_joint_states[0] = joint_state_msg->position[9];
            // right_e1
            cur_right_joint_states[1] = joint_state_msg->position[10];
            // right_s0
            cur_right_joint_states[2] = joint_state_msg->position[11];
            // right_s1
            cur_right_joint_states[3] = joint_state_msg->position[12];
            // right_w0
            cur_right_joint_states[4] = joint_state_msg->position[13];
            // right_w1
            cur_right_joint_states[5] = joint_state_msg->position[14];
            // right_w1
            cur_right_joint_states[6] = joint_state_msg->position[15];
        }
    }
}
