/*
 * File: train_representation.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <fstream>
#include "train_representation.h"

using namespace std;

TrainRepresentation::TrainRepresentation(ros::NodeHandle nh)
{
    this->nh = nh;

    it = new image_transport::ImageTransport(nh);
    image_pub = it->advertise("train/image", 1);

    detected_sub = nh.subscribe("objects", 1, &TrainRepresentation::objectsDetectedCallback, this);
}

TrainRepresentation::~TrainRepresentation()
{
    delete it;
}


int TrainRepresentation::doTraining()
{
    return 0;
}

void TrainRepresentation::objectsDetectedCallback(const std_msgs::Float32MultiArray & msg)
{
    if (msg.data.size()) {
        for (unsigned int i=0; i<msg.data.size(); i+=12) {

            // get data
            int id = (int)msg.data[i];
            float objectWidth = msg.data[i+1];
            float objectHeight = msg.data[i+2];

            cout << "object detected!" << endl;
        }
    }
}

