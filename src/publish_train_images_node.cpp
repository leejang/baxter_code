/*
 * File: publish_train_images_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <fstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "publish_train_images");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub;

    ROS_INFO("publish training images");

    image_pub = it.advertise("train/image", 1);

    // to open video file
    cv::VideoCapture capture_video;

    while (ros::ok())
    {
        cv_bridge::CvImage out_msg;
        std_msgs::Header img_header;

        // open video file
        string videoFile = "/home/leejang/data/recorded_videos_on_0603_2016/scenario3/0311.mp4"; 
        capture_video.open(videoFile);
        cv::Mat frame;

#if 0
        // read image file
        string imageFile = "/home/leejang/data/recorded_videos_on_0603_2016/water_cup.jpg";
        frame = cv::imread(imageFile, CV_LOAD_IMAGE_COLOR);
        //cv::namedWindow("Play recorded video", 1);
#endif

        if (!capture_video.isOpened()) {
            ROS_ERROR("!!! Error when opening video file");
            break;
        } else
            //cv::namedWindow("Play recorded video", 1);

        while(1) {
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
#if 0
                char filename[80];
                static int cnt = 0;
                sprintf(filename, "test_%d.jpg", cnt);
                imwrite(filename, frame);
                cnt++;
                cout << cnt << endl;
#endif
                // Output video stream as ROS sensor image format
                // sensor image header
                img_header.seq = (double)capture_video.get(CV_CAP_PROP_POS_FRAMES);
                img_header.stamp = ros::Time::now(); 

                out_msg.header = img_header;
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = frame;

                // publish
                image_pub.publish(out_msg.toImageMsg());

            }
        }
    }

    // release video handler
    capture_video.release();

    return 0;
}

