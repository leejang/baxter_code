/*
 * File: hand_detector.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <fstream>
#include "hand_detector.h"

using namespace std;

#define DEBUG 1
///////////////////////////////////
#define GPU_DEVICE_ID 0
//#define CPU_ONLY
#define MODEL_PATH "/home/leejang/data/caffe_models/hand_type_classifier.prototxt"
#define WEIGHTS_PATH "/home/leejang/data/caffe_models/hand_type_classifier.caffemodel"
// MATLAB Implemented Window Proposals
#define MATLAB_PATH ":/usr/local/MATLAB/R2015a/bin:/usr/local/MATLAB/R2015a/sys/os"
#define ADD_MATLAB_FUNCTION_DIR "addpath('/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/lib/window_proposals');"
#define WINDOW_INPUT_FILE "/home/leejang/data/hands/current_windows.txt"
///////////////////////////////////

std::fstream& GotoLine(std::fstream& file, unsigned int num)
{
    file.seekg(std::ios::beg);
    for(int i=0; i < num - 1; ++i){
        file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    return file;
}

HandDetector::HandDetector(ros::NodeHandle nh)
{
    this->nh = nh;

    // subscribers to get each joint position of user
    left_hand_pose_sub = nh.subscribe("skeleton/left_hand_joint_uv", 1, &HandDetector::leftHandPoseCB, this);
    right_hand_pose_sub = nh.subscribe("skeleton/right_hand_joint_uv", 1, &HandDetector::rightHandPoseCB, this);
 
    // Initialize Matlab Engine
    initMatlabEngine();

    // generate window proposals
    generateWindowProposals();

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

    if (matlab_ep != NULL)
        engClose(matlab_ep);
}

int HandDetector::initMatlabEngine()
{
    int retVal = 0;

    char *envPath;
    //char *newPath;
    envPath = getenv("PATH");
    
    string matlabPath = MATLAB_PATH; 

    unsigned int pathLength = strlen(envPath) + strlen(matlabPath.c_str());

    char *newPath = new char[pathLength];

    strcpy(newPath, envPath);
    strcat(newPath, matlabPath.c_str());

    setenv("PATH", newPath, 1);
    //system("echo $PATH");

    // For MATLAB
    if (!(matlab_ep = engOpen(""))) {
        cerr << "Can't start MATLAB engine" << endl;
        retVal = -1;
    }
    // Add Matlab function path
    string addMatlabFuncDir = ADD_MATLAB_FUNCTION_DIR;
    engEvalString(matlab_ep, addMatlabFuncDir.c_str());

    delete[] newPath;

    return retVal;
}

void HandDetector::generateWindowProposals()
{
    engEvalString(matlab_ep, "genWindowProposals(1000, 'test');");
}

int HandDetector::parseWindowInputFile()
{
    int retVal = 0;

    FILE * pFile;

    pFile = fopen(WINDOW_INPUT_FILE,"r");

    if (pFile != NULL) {

        fclose (pFile);
    } else {
        cerr << "Can't open widnow input file" << endl;
        retVal = -1;
    }

    cout << "parse done!" << endl;
    return retVal;
}

void HandDetector::leftHandPoseCB(const geometry_msgs::Pose2D pose)
{
    //ROS_INFO("leftHandPoseCB");
    left_hand_pose = pose;

#if DEBUG
    cout << "left_hand_pose: " << left_hand_pose.x << ", "
         << left_hand_pose.y << endl;
#endif
}

void HandDetector::rightHandPoseCB(const geometry_msgs::Pose2D pose)
{
    //ROS_INFO("rightHandPoseCB");
    right_hand_pose = pose;

#if DEBUG
    cout << "right_hand_pose: " << right_hand_pose.x << ", "
         << right_hand_pose.y << endl;
#endif
}

void HandDetector::doDetection()
{
    // generate window proposals
    generateWindowProposals();

    // parse generated window input file
    parseWindowInputFile();

    cout << "hand detection done!" << endl;

#if 0
    const vector<shared_ptr<Layer<float> > >& layers = caffe_net->layers();

#if 0
    for (int i = 0; i < layers.size(); ++i) {
        const caffe::string& layername = layers[i]->layer_param().name();
        cout << std::setfill(' ') << std::setw(10) << layername;
    }
#endif

    // Get number of windows
    fstream infile(layers[0]->layer_param().window_data_param().source().c_str());

    GotoLine(infile,6);
    unsigned int num_windows = 0;
    infile >> num_windows;
    //cout << "num_windows: " << num_windows << endl;
    infile.close();

    // Get batch_size
    const int batch_size = layers[0]->layer_param().window_data_param().batch_size();
    //cout << "batch_size: " << batch_size << endl;

    unsigned int num_iterations = (num_windows / batch_size) + 1;
    //cout << "num_iterations: " << num_iterations << endl;

    float loss = 0;

    unsigned int window_cnt = 0;
    for (int i = 0; i < num_iterations; ++i) {
        float iter_loss = 0;

        const vector<Blob<float>*>& result = caffe_net->Forward(&iter_loss);
        loss += iter_loss;

        const float* result_vec = result[1]->cpu_data();

        // number of windows in each batch_size
        //cout << result[1]->count() << endl;
        for (int j = 0; j < result[1]->count(); ++j) {
            const float score = result_vec[j];
            //cout << " window num: [" << window_cnt << "]" << " score: " << score << endl;
            // my left hand
            if (window_cnt % 5 == 1 && score > 0.9)
                cout << "[My left hand detected] window num: [" << window_cnt << "]" << " score: " << score << endl;
            // my right hand
            if (window_cnt % 5 == 2 && score > 0.9)
                cout << "[My right hand detected] window num: [" << window_cnt << "]" << " score: " << score << endl;
            // your left hand
            if (window_cnt % 5 == 3 && score > 0.9)
                cout << "[Your left hand detected] window num: [" << window_cnt << "]" << " score: " << score << endl;
            // your right hand
            if (window_cnt % 5 == 4 && score > 0.9)
                cout << "[Your right hand detected] window num: [" << window_cnt << "]" << " score: " << score << endl;

            window_cnt++;
        }
    }
#endif
}
