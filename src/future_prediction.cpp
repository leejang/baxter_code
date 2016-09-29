/*
 * File: future_prediction.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include "future_prediction.h"

using namespace std;

#define DEBUG 0
//#define CPU_ONLY

///////////////////////////////////
#define GPU_DEVICE_ID 1
//#define CPU_ONLY
#define MODEL_PATH "/home/leejang/data/caffe_models/robot_learning.prototxt"
//#define WEIGHTS_PATH "/home/leejang/data/caffe_models/robot_learning.caffemodel"
#define WEIGHTS_PATH "/home/leejang/data/caffe_models/tmp_iter_500000.caffemodel"
///////////////////////////////////


FuturePrediction::FuturePrediction(ros::NodeHandle nh)
{
    this->nh = nh;

    // subscribers to get each joint position of user
    head_pose_sub = nh.subscribe("skeleton/head_joint_uv", 1, &FuturePrediction::headPoseCB, this);
    left_hand_pose_sub = nh.subscribe("skeleton/left_hand_joint_uv", 1, &FuturePrediction::leftHandPoseCB, this);
    right_hand_pose_sub = nh.subscribe("skeleton/right_hand_joint_uv", 1, &FuturePrediction::rightHandPoseCB, this);

    // publishcer to send detection completed message
    detected_pub = nh.advertise<std_msgs::UInt64>("future_prediction/done", 1);


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

    head_pose_cnt = 0;
    completed_frame_cnt.data = 0;

    cout << "Initialization done!" <<endl;
}

FuturePrediction::~FuturePrediction()
{
    if (caffe_net != NULL)
        delete caffe_net;

}


void FuturePrediction::headPoseCB(const geometry_msgs::Pose2D pose)
{
#if DEBUG
    cout << "FP: headPoseCB Count: " << head_pose_cnt << endl;
    head_pose_cnt++;
#endif
    //doDetection();
}

void FuturePrediction::leftHandPoseCB(const geometry_msgs::Pose2D pose)
{
    //ROS_INFO("leftHandPoseCB");
    left_hand_pose = pose;

#if DEBUG
    cout << "left_hand_pose: " << left_hand_pose.x << ", "
         << left_hand_pose.y << endl;
#endif
}

void FuturePrediction::rightHandPoseCB(const geometry_msgs::Pose2D pose)
{
    //ROS_INFO("rightHandPoseCB");
    right_hand_pose = pose;

#if DEBUG
    cout << "right_hand_pose: " << right_hand_pose.x << ", "
         << right_hand_pose.y << endl;
#endif
}


void FuturePrediction::predictFuture()
{
    cout << "predict Future!" << endl;


    const vector<shared_ptr<Layer<float> > >& layers = caffe_net->layers();

#if DEBUG
    for (int i = 0; i < layers.size(); ++i) {
        const caffe::string& layername = layers[i]->layer_param().name();
        cout << std::setfill(' ') << std::setw(10) << layername;
    }
#endif

    float loss = 0;
    unsigned int num_iterations = 3;
    for (int i = 0; i < num_iterations; ++i) {
        float iter_loss = 0;

        const vector<Blob<float>*>& result = caffe_net->Forward(&iter_loss);
        loss += iter_loss;
        //cout << "loss: " << loss << endl;
        const float* result_vec = result[0]->cpu_data();

        // number of windows in each batch_size
        //cout << result[0]->count() << endl;
        for (int j = 0; j < result[0]->count(); ++j) {
            const float score = result_vec[j];
            cout << " window num: [ " << j << "]" << " score: " << score << endl;
        }
    }
}