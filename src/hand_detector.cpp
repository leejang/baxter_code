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
// MATLAB SCRIPT
#define WINDOW_PROPOSALS_MATLAB_SCRIPT "/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/lib/window_proposals/execute_matlab.sh"
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

#ifdef CPU_ONLY
    cout << "CPU_ONLY" << endl;
    Caffe::set_mode(Caffe::CPU);    
#else
    Caffe::set_mode(Caffe::GPU);
    Caffe::SetDevice(GPU_DEVICE_ID);
#endif
}

HandDetector::~HandDetector()
{
#if 0
    if (caffe_net != NULL)
        delete caffe_net;
#endif

}

void HandDetector::generateWindowProposals()
{
    // CAll MATLAB code
    system(WINDOW_PROPOSALS_MATLAB_SCRIPT);
}

void HandDetector::doDetection()
{

    // generate window proposals
    generateWindowProposals();

    string model_path = MODEL_PATH;
    string weights_path = WEIGHTS_PATH;

    // Caffe Initialize
    caffe_net = new Net<float>(model_path, caffe::TEST);
    caffe_net->CopyTrainedLayersFrom(weights_path);

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
            cout << " window num: [" << window_cnt << "]" << " score: " << score << endl;
            window_cnt++;
        }
    }
}
