/*
 * File: object_detector.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <string>
#include <sstream>
#include "object_detector.h"

using namespace std;

template <typename T>
std::string to_string(T value)
{
    std::ostringstream os;
    os << value;
    return os.str() ;
}

ObjectDetector::ObjectDetector(ros::NodeHandle nh)
{
    this->nh = nh;

    detected_w_depth_sub = nh.subscribe("objectsStamped", 1, &ObjectDetector::objectsDetectedCallback, this);
    detected_w_depth_pub = nh.advertise<find_object_2d::ObjectsStamped>("objectsStamped/pose", 1);
}

ObjectDetector::~ObjectDetector()
{
}



void ObjectDetector::objectsDetectedCallback(const find_object_2d::ObjectsStampedConstPtr& msg)
{
    //ROS_INFO("ObjectsDetectedCallback");
    if (msg->objects.data.size()) {
        for (unsigned int i = 0; i < msg->objects.data.size(); i += 12) {

            // get data
            int id = (int)msg->objects.data[i];
            string objectFrameId = "object_" + to_string(id);
            //cout << objectFrameId << endl;

            try
            {
                // Get transformation from "object_#" frame to target frame "world"
                // The timestamp matches the one sent over TF
                tfListener_.lookupTransform("/world", objectFrameId, msg->header.stamp, pose);
                tfListener_.lookupTransform(msg->header.frame_id, objectFrameId, msg->header.stamp, poseCam);
            }
            catch (tf::TransformException & ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }

            // Here "pose" is the position of the object "id" in "/map" frame.
            ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                        id, "/world",
                        pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z(),
                        pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z(), pose.getRotation().w());
#if 0
            ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                        id, msg->header.frame_id.c_str(),
                        poseCam.getOrigin().x(), poseCam.getOrigin().y(), poseCam.getOrigin().z(),
                        poseCam.getRotation().x(), poseCam.getRotation().y(), poseCam.getRotation().z(), poseCam.getRotation().w());
#endif
        }
    detected_w_depth_pub.publish(msg);
    }
}
