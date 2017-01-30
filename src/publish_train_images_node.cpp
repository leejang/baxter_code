/*
 * File: publish_train_images_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt64.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

#define SAVED_IMG "/home/leejang/data/hands/current_frame.jpg"

using namespace std;

// global flag to publish each frame after all recognition processing has been completed
bool has_to_publish = false;

// frame count
std_msgs::UInt64 frame_cnt;

void handDetectionCB(const std_msgs::UInt64::ConstPtr& msg)
{
    ROS_INFO("I heard frame: [%lu]", msg->data);
    has_to_publish = true;
}

void objDetectionCB(const std_msgs::Float32MultiArray & msg)
{
    has_to_publish = true;
}

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "publish_train_images");
    ros::NodeHandle nh;
    ros::Rate r(10); // 10 hz

    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub;

    ROS_INFO("publish training images");

    // subscribers
    ros::Subscriber hand_sub = nh.subscribe("hand_detector/done", 1, handDetectionCB);
    ros::Subscriber obj_sub = nh.subscribe("objects", 1, objDetectionCB);
    // publishers
    ros::Publisher frame_cnt_pub = nh.advertise<std_msgs::UInt64>("frame_cnt", 10);
    //image_pub = it.advertise("train/image", 1);
    image_pub = it.advertise("image", 1);
    ros::Publisher image_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    // to open video file
    cv::VideoCapture capture_video;

    cv_bridge::CvImage out_msg;
    std_msgs::Header img_header;
    sensor_msgs::CameraInfo img_info;

    // open video file
    //string videoFile = "/home/leejang/data/recorded_videos_on_0920_2016/scenario1/090720_0111.avi";
    string videoFile = "/data/share/8/X/%05d.jpg";
    capture_video.open(videoFile);
    cv::Mat frame;

    // read image file
    //string imageFile = "/home/leejang/data/recorded_videos_on_0907_2016/coffee2.jpg";
    //frame = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
    //cv::namedWindow("Play recorded video", 1);

    // initialize frame count
    frame_cnt.data = 1;

    // waiting count
    unsigned int waiting_cnt = 0;

    if (!capture_video.isOpened()) {
        ROS_ERROR("!!! Error when opening video file");
        return -1;
    } else {
        //cv::namedWindow("Play recorded video", 1);

        has_to_publish = true;

        while (ros::ok()) {

            if (!has_to_publish) {
                //cout << "I'm waiting.. [" << waiting_cnt << "]" << endl;

                // waiting around 5 sec (in case of use 10 hz loop rate)
                if (waiting_cnt >= 50) {
                    waiting_cnt = 0;
                    has_to_publish = true;
                } else {
                    waiting_cnt++;
                }
            } else {
                // read each video frame
                capture_video >> frame;
                if (frame.empty()) {
                    cout << "done!!" << endl;
                    break;
                } else {
                    //cv::imshow("Play recorded video", frame);

                    //cv::waitKey(0);
                    // to check image encoding
                    //C1	C2	C3	C4
                    //CV_8U  	0	8	16	24
                    //CV_8S 	1	9	17	25
                    //CV_16U	2	10	18	26
                    //CV_16S	3	11	19	27
                    //CV_32S	4	12	20	28
                    //CV_32F	5	13	21	29
                    //CV_64F	6	14	22	30
                    //cout << frame.type();
#if 0
                    printf( "CV_CAP_PROP_POS_MSEC:   %ld \n", (long) capture_video.get( CV_CAP_PROP_POS_MSEC) );
                    printf( "CV_CAP_PROP_POS_FRAMES:  %ld \n", (long) capture_video.get( CV_CAP_PROP_POS_FRAMES) );
                    printf( "CV_CAP_PROP_FPS:  %f\n", capture_video.get( CV_CAP_PROP_FPS));
#endif
                    char filename[80];
                    sprintf(filename, "frame_%lu.jpg", frame_cnt.data);
                    //imwrite(filename, frame);
                    // for hand detector (saving current frame (overwrite))
                    imwrite(SAVED_IMG, frame);
                    cout << frame_cnt.data << endl;

                    // Output video stream as ROS sensor image format
                    // sensor image header
                    img_header.seq = (double)capture_video.get(CV_CAP_PROP_POS_FRAMES);
                    img_header.stamp = ros::Time::now();
                    // camera info
                    img_info.header = img_header;
                    img_info.width = frame.size().width;
                    img_info.height = frame.size().height;
                    img_info.distortion_model = "plumb_bob";

                    // Qlippie Camera calibration
                    //double intrinsic_params [] = {671.722, 0, 633.281, 0, 673.785, 378.107, 0, 0, 1};
                    // JK Camera calibration
                    double intrinsic_params [] = {870.105, 0, 970.753, 0, 880.417, 541.962, 0, 0, 1};
                    for (int i = 0; i < 9; i++) {
                        img_info.K[i] = intrinsic_params[i];
                    }

                    // Qlippie Camera calibration
                    //double projection_params [] = {671.722, 0, 633.281, 0, 0, 673.785, 378.107, 0, 0, 0, 1, 0};
                    double projection_params [] = {870.105, 0, 970.753, 0, 0, 880.417, 541.962, 0, 0, 0, 1, 0};
                    for (int i = 0; i < 12; i++) {
                        img_info.P[i] = projection_params[i];
                    }

                    out_msg.header = img_header;
                    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                    out_msg.image = frame;

                    // publish
                    //ros::Duration(5).sleep();
                    image_pub.publish(out_msg.toImageMsg());
                    image_info_pub.publish(img_info);
                    frame_cnt_pub.publish(frame_cnt);
                    //has_to_publish = false;
                    frame_cnt.data++;
                }
            } // else

            ros::spinOnce();
            r.sleep();
        } // while(1)
    }

    // shutdown ROS
    ros::shutdown();

    // release video handler
    capture_video.release();
    return 0;
}

