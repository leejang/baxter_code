/*
 * File: record_visual_odometry_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <fstream>
#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <nav_msgs/Odometry.h>

#define REC_VISO "/home/leejang/data/8_X_visual_odometry.txt"

using namespace std;
std::ofstream record_file;
unsigned long frame_cnt = 0;

void odometryCB(const nav_msgs::OdometryConstPtr& msg)
{
    //ROS_INFO("I heard odometry frame: [%d]", msg->header.seq);
    record_file << frame_cnt << " ";
    // pose
    // pose/position(x,y,z)
    record_file << msg->pose.pose.position.x << " ";
    record_file << msg->pose.pose.position.y << " ";
    record_file << msg->pose.pose.position.z << " ";
    // pose/orientation(x,y,z,w)
    record_file << msg->pose.pose.orientation.x << " ";
    record_file << msg->pose.pose.orientation.y << " ";
    record_file << msg->pose.pose.orientation.z << " ";
    record_file << msg->pose.pose.orientation.w << " ";
// NOT published
#if 0
    // pose/covariance
    for (int i = 0; i < 36; i++) {
        record_file << msg->pose.covariance[i] << " ";
    }
#endif
    // twist 
    // twist/linear(x,y,z)
    record_file << msg->twist.twist.linear.x << " ";
    record_file << msg->twist.twist.linear.y << " ";
    record_file << msg->twist.twist.linear.z << " ";
    // twist/angular(x,y,z)
    record_file << msg->twist.twist.angular.x << " ";
    record_file << msg->twist.twist.angular.y << " ";
    record_file << msg->twist.twist.angular.z << " ";
// NOT published
#if 0
    // twist/covariance
    for (int i = 0; i < 36; i++) {
        record_file << msg->twist.covariance[i] << " ";
    }
#endif
    record_file << endl;
   
}

void frameCntCB(const std_msgs::UInt64::ConstPtr& msg)
{
    ROS_INFO("I heard frame count CB: [%lu]", msg->data);
    frame_cnt = msg->data;
}  

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "record_visual_odometry");
    ros::NodeHandle nh;

    ROS_INFO("recording visual odometry");

    record_file.open(REC_VISO);

    // subscribers
    ros::Subscriber viso_sub = nh.subscribe("mono_odometer/odometry", 1, odometryCB);
    ros::Subscriber frame_cnt_sub = nh.subscribe("frame_cnt", 10, frameCntCB);

    ros::spin();

    record_file.close();

    // shutdown ROS
    ros::shutdown();

    return 0;
}
