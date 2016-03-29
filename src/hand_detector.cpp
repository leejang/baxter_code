/*
 * File: hand_detector.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "hand_detector.h"

using namespace std;

///////////////////////////////////
#define GPU_DEVICE_ID 0
//#define CPU_ONLY
#define MODEL_PATH "/home/leejang/data/caffe_models/hand_type_classifier.prototxt"
#define WEIGHTS_PATH "/home/leejang/data/caffe_models/hand_type_classifier.caffemodel"
///////////////////////////////////

HandDetector::HandDetector(ros::NodeHandle nh)
{
    this->nh = nh;

#ifdef CPU_ONLY
    cout << "CPU_ONLY" << endl;
    Caffe::set_mode(Caffe::CPU);    
#else
    Caffe::set_mode(Caffe::GPU);
    Caffe::SetDevice(GPU_DEVICE_ID);
#endif

    string model_path = MODEL_PATH;
    string weights_path = WEIGHTS_PATH;

    // Caffe Initialize
    caffe_net = new Net<float>(model_path, caffe::TEST);
    caffe_net->CopyTrainedLayersFrom(weights_path);
}

HandDetector::~HandDetector()
{
    if (caffe_net != NULL)
        delete caffe_net;
}


