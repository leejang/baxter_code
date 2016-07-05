/*
 * File: train_representation.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <math.h>
#include "train_representation.h"

#define DETECTION_RESULT "/home/leejang/data/hands/object_detection.txt"

using namespace std;

TrainRepresentation::TrainRepresentation(ros::NodeHandle nh)
{
    this->nh = nh;

    detected_sub = nh.subscribe("objects", 1, &TrainRepresentation::objectsDetectedCallback, this);
    detection_result.open(DETECTION_RESULT);
    frame_cnt = 0;
}

TrainRepresentation::~TrainRepresentation()
{
    cout << "done!!!!!!" << endl;
    detection_result.close();
}


int TrainRepresentation::doTraining()
{
    return 0;
}

void TrainRepresentation::objectsDetectedCallback(const std_msgs::Float32MultiArray & msg)
{
    cout << "frame_cnt: " << frame_cnt << endl;
    detection_result << "# " << frame_cnt << endl;

    if (msg.data.size()) {
        for (unsigned int i=0; i < msg.data.size(); i+=12) {

            // get data
            int id = (int)msg.data[i];
            float objectWidth = msg.data[i+1];
            float objectHeight = msg.data[i+2];

            //cout << "object detected!" << endl;

            cv::Mat cvHomography(3, 3, CV_32F);
            cvHomography.at<float>(0,0) = msg.data[i+3];
            cvHomography.at<float>(1,0) = msg.data[i+4];
            cvHomography.at<float>(2,0) = msg.data[i+5];
            cvHomography.at<float>(0,1) = msg.data[i+6];
            cvHomography.at<float>(1,1) = msg.data[i+7];
            cvHomography.at<float>(2,1) = msg.data[i+8];
            cvHomography.at<float>(0,2) = msg.data[i+9];
            cvHomography.at<float>(1,2) = msg.data[i+10];
            cvHomography.at<float>(2,2) = msg.data[i+11];

            std::vector<cv::Point2f> inPts, outPts;
            inPts.push_back(cv::Point2f(0,0));
            inPts.push_back(cv::Point2f(objectWidth,0));
            inPts.push_back(cv::Point2f(0,objectHeight));
            inPts.push_back(cv::Point2f(objectWidth,objectHeight));
            cv::perspectiveTransform(inPts, outPts, cvHomography);

#if 0
            printf("Object %d detected, CV corners at (%d,%d) (%d,%d) (%d,%d) (%d,%d)\n",
                    id,
                    (int)round(outPts.at(0).x), (int)round(outPts.at(0).y),
                    (int)round(outPts.at(1).x), (int)round(outPts.at(1).y),
                    (int)round(outPts.at(2).x), (int)round(outPts.at(2).y),
                    (int)round(outPts.at(3).x), (int)round(outPts.at(3).y)
            );
#endif
           detection_result << id << " "
                            << (int)round(outPts.at(0).x) << " " << (int)round(outPts.at(0).y) << " "
                            << (int)round(outPts.at(1).x) << " " << (int)round(outPts.at(1).y) << " "
                            << (int)round(outPts.at(2).x) << " " << (int)round(outPts.at(2).y) << " "
                            << (int)round(outPts.at(3).x) << " " << (int)round(outPts.at(3).y) << " "
                            << endl;
        }
    } else {
        //printf("no objects\n");
    }

    frame_cnt++;
}

