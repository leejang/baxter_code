/*
 * File: skeleton_tracker.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "skeleton_tracker.h"

// Kinect Camera
#define ROS_CAM_DEPTH_RAW_PATH "/camera/depth/image_raw"
#define ROS_CAM_RGB_COLOR_PATH "/camera/rgb/image_color"

// To record skeleton tracker video
#define REC_VIDEO 0
// To play recorded video
#define PLAY_VIDEO 0

unsigned int test_cnt = 0;

using namespace std;

SkeletonTracker::SkeletonTracker(ros::NodeHandle nh){
    this->nh = nh;
    
    it = new image_transport::ImageTransport(nh);

    head_joint = nh.advertise<geometry_msgs::Pose2D>("skeleton/head_joint_uv", 1);
    left_hand_joint = nh.advertise<geometry_msgs::Pose2D>("skeleton/left_hand_joint_uv", 1);
    right_hand_joint = nh.advertise<geometry_msgs::Pose2D>("skeleton/right_hand_joint_uv", 1);
    torso_joint = nh.advertise<geometry_msgs::Pose2D>("skeleton/torso_joint_uv", 1);

    robot_screen = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 1, true);

    sub_depth = it->subscribeCamera(ROS_CAM_RGB_COLOR_PATH, 10, &SkeletonTracker::onNewImageCallback, this);

#if REC_VIDEO
    // Setup output video to save video file
    output_video.open("skeleton_track.avi", CV_FOURCC('I', 'Y', 'U', 'V'), 30, cv::Size(640, 480), true);

    if (!output_video.isOpened())
        ROS_ERROR("!!! Output video could not be opened");
#endif

#if PLAY_VIDEO
    // open video file for test
    capture_video.open("skeleton_track.avi");
    cv::Mat frame;

    if (!capture_video.isOpened())
        ROS_ERROR("!!! Error when opening video file");
    else
        cv::namedWindow("Play recorded video", 1);

    while(1) {
        capture_video >> frame;
        if (frame.empty())
            break;
        else {
            cv::imshow("Play recorded video", frame);
            cv::waitKey(20);
        }
    }
    // close window
    cv::waitKey(0);
    cv::destroyWindow("Play recorded video");
#endif

    full_screen.image = cv::Mat(600, 1024, CV_8UC3);
}

SkeletonTracker::~SkeletonTracker()
{
    if (it != NULL)
        delete it;

#if REC_VIDEO
    output_video.release();
#endif

#if PLAY_VIDEO
    capture_video.release();
#endif
}

void SkeletonTracker::onNewImageCallback(const sensor_msgs::ImageConstPtr& image_msg,
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
           tfListener.lookupTransform(cam_model.tfFrame(), "/head_1", ros::Time(0), tf_head);
           tfListener.lookupTransform(cam_model.tfFrame(), "/left_hand_1", ros::Time(0), tf_left_hand);
           tfListener.lookupTransform(cam_model.tfFrame(), "/right_hand_1", ros::Time(0), tf_right_hand);
           tfListener.lookupTransform(cam_model.tfFrame(), "/torso_1", ros::Time(0), tf_torso);
       }
       catch (tf::TransformException & ex)
       {
            //ROS_WARN("%s",ex.what());
            return;
       }

       //ROS_INFO_STREAM(cam_model.tfFrame());

       // Head joint
       head_pt = tf_head.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d head_cv_pt(head_pt.x(), head_pt.y(), head_pt.z());
       head_uv = cam_model.project3dToPixel(head_cv_pt);
       head_pose.x = head_uv.x;
       head_pose.y = head_uv.y;
       head_pose.theta = 0;

       // Left Hand joint
       left_hand_pt = tf_left_hand.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d left_hand_cv_pt(left_hand_pt.x(), left_hand_pt.y(), left_hand_pt.z());
       left_hand_uv = cam_model.project3dToPixel(left_hand_cv_pt);
       left_hand_pose.x = left_hand_uv.x;
       left_hand_pose.y = left_hand_uv.y;
       left_hand_pose.theta = 0;

       // Right Hand joint
       right_hand_pt = tf_right_hand.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d right_hand_cv_pt(right_hand_pt.x(), right_hand_pt.y(), right_hand_pt.z());
       right_hand_uv = cam_model.project3dToPixel(right_hand_cv_pt);
       right_hand_pose.x = right_hand_uv.x;
       right_hand_pose.y = right_hand_uv.y;
       right_hand_pose.theta = 0;

       // Torso joint
       torso_pt = tf_torso.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d torso_cv_pt(torso_pt.x(), torso_pt.y(), torso_pt.z());
       torso_uv = cam_model.project3dToPixel(torso_cv_pt);
       torso_pose.x = torso_uv.x;
       torso_pose.y = torso_uv.y;
       torso_pose.theta = 0;

       // Publish each joint positions
       head_joint.publish(head_pose);
       // Note that Openni tracker uses camera point of view
       // thus, left hand is user's right hand and vice versa.
       left_hand_joint.publish(left_hand_pose);
       right_hand_joint.publish(right_hand_pose);
       torso_joint.publish(torso_pose);

#if 0
       cout << "(In-Skel) Test cnt: " << test_cnt << endl;
       test_cnt++;
#endif
       static const int RADIUS = 10;
       cv::circle(cv_ptr->image, head_uv, RADIUS, CV_RGB(0,255,0), -1);
       cv::circle(cv_ptr->image, left_hand_uv, RADIUS, CV_RGB(255,0,0), -1);
       cv::circle(cv_ptr->image, right_hand_uv, RADIUS, CV_RGB(0,0,255), -1);
       cv::circle(cv_ptr->image, torso_uv, RADIUS, CV_RGB(0,0,0), -1);

       CvPoint head_origin = cvPoint(head_uv.x - 20, head_uv.y - 20);
       cv::putText(cv_ptr->image, "Head", head_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,255,0));
       CvPoint left_h_origin = cvPoint(left_hand_uv.x - 20, left_hand_uv.y - 20);
       cv::putText(cv_ptr->image, "Left Hand", left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0));
       CvPoint right_h_origin = cvPoint(right_hand_uv.x - 20, right_hand_uv.y - 20);
       cv::putText(cv_ptr->image, "Right Hand", right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,0,255));
       CvPoint torso_origin = cvPoint(torso_uv.x - 20, torso_uv.y - 20);
       cv::putText(cv_ptr->image, "Torso", torso_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,0,0));

#if REC_VIDEO
       output_video.write(cv_ptr->image);
#endif

#if 0
       cv::imshow("Skeletton Tracker Projection(3D -> 2D)", cv_ptr->image);
       cv::waitKey(1);
#endif

       // display on Baxter's screen
       full_screen.header = cv_ptr->header;
       full_screen.encoding = cv_ptr->encoding;
       cv::resize(cv_ptr->image, full_screen.image, full_screen.image.size(), 0, 0, cv::INTER_NEAREST);
       robot_screen.publish(full_screen.toImageMsg());
    }
}

