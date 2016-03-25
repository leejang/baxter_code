/*
 * File: hand_detection_node.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <ros/ros.h>
#include <iostream>
#include <caffe/caffe.hpp>
#include <vector>

using namespace std;
using caffe::Caffe;
using caffe::Net;
using caffe::Blob;

#define GPU_DEVICE_ID 0
//#define CPU_ONLY

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "hand_detection");

#ifdef CPU_ONLY
    cout << "CPU_ONLY" << endl;
    Caffe::set_mode(Caffe::CPU);    
#else
    Caffe::set_mode(Caffe::GPU);
    Caffe::SetDevice(GPU_DEVICE_ID);
#endif

    string model_path = "/home/leejang/data/caffe_models/hand_type_classifier.prototxt";
    string weights_path = "/home/leejang/data/caffe_models/hand_type_classifier.caffemodel";
    //string image_path = "../caffe/images/cat.jpg";

#if 0
    // Initial
    Net<float> *caffe_net;
    caffe_net = new Net<float>(model_path, caffe::TEST);
    caffe_net->CopyTrainedLayersFrom(weights_path);
#endif
    //vector<Blob<float>*> dummy_bottom_vec;
    //float loss;
    //const std::vector<Blob<float>*>& result = caffe_net->Forward(dummy_bottom_vec, &loss);

    cout << "Here" << endl;

    return 0;
}

