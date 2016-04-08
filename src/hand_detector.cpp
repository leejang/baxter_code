/*
 * File: hand_detector.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */

#include <fstream>
#include "hand_detector.h"

using namespace std;

///////////////////////////////////
#define GPU_DEVICE_ID 0
//#define CPU_ONLY
#define MODEL_PATH "/home/leejang/data/caffe_models/hand_type_classifier.prototxt"
#define WEIGHTS_PATH "/home/leejang/data/caffe_models/hand_type_classifier.caffemodel"
// MATLAB Implemented Window Proposals
#define MATLAB_PATH ":/usr/local/MATLAB/R2015a/bin:/usr/local/MATLAB/R2015a/sys/os"
#define ADD_MATLAB_FUNCTION_DIR "addpath('/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/lib/window_proposals');"
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

    // Initialize Matlab Engine
    initMatlabEngine();

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
    engEvalString(matlab_ep, "genWindowProposals(1000);");
}

void HandDetector::doDetection()
{
    // generate window proposals
    generateWindowProposals();

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

        for (int j = 0; j < result[1]->count(); ++j) {
            const float score = result_vec[j];
            //cout << " window num: [" << window_cnt << "]" << " score: " << score << endl;
            window_cnt++;
        }
    }
#endif
}
