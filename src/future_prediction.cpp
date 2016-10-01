/*
 * File: future_prediction.cpp
 * Author: Jangwon lee
 * Email: leejang@indiana.edu
 */


#include "future_prediction.h"

using namespace std;

#define DEBUG 0
//#define CPU_ONLY

// Kinect Camera
#define ROS_CAM_RGB_COLOR_PATH "/camera/rgb/image_color"

///////////////////////////////////
#define GPU_DEVICE_ID 1
//#define CPU_ONLY
#define MODEL_PATH "/home/leejang/data/caffe_models/robot_learning.prototxt"
//#define WEIGHTS_PATH "/home/leejang/data/caffe_models/robot_learning.caffemodel"
#define WEIGHTS_PATH "/home/leejang/data/caffe_models/tmp_iter_500000.caffemodel"
#define PREDICTION_INPUT "/home/leejang/data/recorded_videos_on_0920_2016/scenario2/0214/prediction_input.h5"
#define PREDICTION_OUTPUT "/home/leejang/data/recorded_videos_on_0920_2016/scenario2/0214/prediction.txt"
#define DATASET "data"
#define LABELSET "label"
#define OBJ_ID_TABLE 0
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
///////////////////////////////////

FuturePrediction::FuturePrediction(ros::NodeHandle nh)
{
    this->nh = nh;

    // subscribers to get each joint position of user
    head_pose_sub = nh.subscribe("skeleton/head_joint_uv", 1, &FuturePrediction::headPoseCB, this);
    left_hand_pose_sub = nh.subscribe("skeleton/left_hand_joint_uv", 1, &FuturePrediction::leftHandPoseCB, this);
    right_hand_pose_sub = nh.subscribe("skeleton/right_hand_joint_uv", 1, &FuturePrediction::rightHandPoseCB, this);
    // for detected objects
    obj_pose_sub = nh.subscribe("objects", 1, &FuturePrediction::objPoseCB, this);

    // publishcer to send detection completed message
    detected_pub = nh.advertise<std_msgs::UInt64>("future_prediction/done", 1);

    it = new image_transport::ImageTransport(nh);
    cam_sub = it->subscribeCamera(ROS_CAM_RGB_COLOR_PATH, 10, &FuturePrediction::onNewImageCB, this);
    robot_screen = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 1, true);


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

    file = H5Fcreate(PREDICTION_INPUT, H5F_ACC_TRUNC, H5P_DEFAULT, H5P_DEFAULT);
    // data set
    d_dims[0] = CHUNK_SZ;
    d_dims[1] = 1;
    d_dims[2] = DIM0;
    d_dims[3] = DIM1*DIM2;
    dspace = H5Screate_simple(DATA_DIM, d_dims, NULL);
    dcpl = H5Pcreate(H5P_DATASET_CREATE);
    status = H5Pset_chunk(dcpl, DATA_DIM, d_dims);
    dset = H5Dcreate(file, DATASET, H5T_IEEE_F32BE, dspace, H5P_DEFAULT, dcpl, H5P_DEFAULT);

    memset(dataset, 0, sizeof(dataset));

    // label set
    l_dims[0] = CHUNK_SZ;
    l_dims[1] = DIM0*DIM1;
    lspace = H5Screate_simple(LABEL_DIM, l_dims, NULL);
    lcpl = H5Pcreate(H5P_DATASET_CREATE);
    status = H5Pset_chunk(lcpl, LABEL_DIM, l_dims);
    lset = H5Dcreate(file, LABELSET, H5T_IEEE_F32BE, lspace, H5P_DEFAULT, lcpl, H5P_DEFAULT);

    save_hdf_file = true;

    full_screen.image = cv::Mat(600, 1024, CV_8UC3);

    // reference pose (table)
    ref_pose.x = 0;
    ref_pose.y = 0;
    got_ref_pose = false;

    cout << "Initialization done!" <<endl;
}

FuturePrediction::~FuturePrediction()
{
    if (caffe_net != NULL)
        delete caffe_net;

    // close all HDF5 hanles
    status = H5Pclose(dcpl);
    status = H5Pclose(lcpl);
    status = H5Dclose(dset);
    status = H5Dclose(lset);
    status = H5Sclose(dspace);
    status = H5Sclose(lspace);
    status = H5Fclose(file);
}


void FuturePrediction::headPoseCB(const geometry_msgs::Pose2D pose)
{
#if 1
    //cout << "FP: headPoseCB Count: " << head_pose_cnt << endl;

    // my_left
    // x
    dataset[0][0][0][0 + head_pose_cnt * DIM1] =
        (my_left_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2); 
    // y
    dataset[0][0][1][0 + head_pose_cnt * DIM1] =
        (my_left_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2); 

    // my_right
    // x
    dataset[0][0][0][1 + head_pose_cnt * DIM1] =
        (my_right_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2); 
    // y
    dataset[0][0][1][1 + head_pose_cnt * DIM1] =
        (my_right_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2); 

    // your_left
    // x
    dataset[0][0][0][2 + head_pose_cnt * DIM1] =
        (yr_left_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2); 
    // y
    dataset[0][0][1][2 + head_pose_cnt * DIM1] =
        (yr_left_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2); 

    // your_right
    // x
    dataset[0][0][0][3 + head_pose_cnt * DIM1] =
        (yr_right_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2); 
    // y
    dataset[0][0][1][3 + head_pose_cnt * DIM1] =
        (yr_right_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2); 

    if (head_pose_cnt == 9) {
        if (save_hdf_file) {
            status = H5Dwrite(dset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, dataset[0]);
            save_hdf_file = false;
            predictFuture();
        }
        head_pose_cnt = 0;
    } else { 
        head_pose_cnt++;
    }
#endif
    //doDetection();
}

void FuturePrediction::leftHandPoseCB(const geometry_msgs::Pose2D pose)
{
    //ROS_INFO("leftHandPoseCB");
    yr_left_hand_pose = pose;

#if DEBUG
    cout << "your left_hand_pose: " << yr_left_hand_pose.x << ", "
         << yr_left_hand_pose.y << endl;
#endif
}

void FuturePrediction::rightHandPoseCB(const geometry_msgs::Pose2D pose)
{
    //ROS_INFO("rightHandPoseCB");
    yr_right_hand_pose = pose;

#if DEBUG
    cout << "your right_hand_pose: " << yr_right_hand_pose.x << ", "
         << yr_right_hand_pose.y << endl;
#endif
}

void FuturePrediction::objPoseCB(const std_msgs::Float32MultiArray & msg)
{
    if (msg.data.size()) {
        for (unsigned int i=0; i < msg.data.size(); i+=12) {

            // get data
            int id = (int)msg.data[i];
            float objectWidth = msg.data[i+1];
            float objectHeight = msg.data[i+2];

            cout << "object detected!" << endl;

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

            if (id == OBJ_ID_TABLE) {

                if (!got_ref_pose) {
                    ref_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    ref_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
                    cout << "Table: " << ref_pose.x << ", " << ref_pose.y << endl;
                    got_ref_pose = true;
                }

            }
#if 0
            printf("Object %d detected, CV corners at (%d,%d) (%d,%d) (%d,%d) (%d,%d)\n",
                    id,
                    (int)round(outPts.at(0).x), (int)round(outPts.at(0).y),
                    (int)round(outPts.at(1).x), (int)round(outPts.at(1).y),
                    (int)round(outPts.at(2).x), (int)round(outPts.at(2).y),
                    (int)round(outPts.at(3).x), (int)round(outPts.at(3).y)
            );
#endif
        }
    } else {
        //printf("no objects\n");
    }
}

void FuturePrediction::onNewImageCB(const sensor_msgs::ImageConstPtr& image_msg,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cam_model.fromCameraInfo(info_msg);

    if (&image_msg == NULL) {
        ROS_WARN_THROTTLE(1,"NULL image received.");
        return;
    } else {

       try
       {
           cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
           ROS_ERROR("Image acquisition failed: %s", e.what());
           return;
       }

       try
       {
           // tfansform each body position to camera frame
           // get the transform from camera frame to the frame of each joint
           tfListener.lookupTransform(cam_model.tfFrame(), "/left_gripper_base", ros::Time(0), tf_my_left_hand);
           tfListener.lookupTransform(cam_model.tfFrame(), "/right_gripper_base", ros::Time(0), tf_my_right_hand);
       }
       catch (tf::TransformException & ex)
       {
            //ROS_WARN("%s",ex.what());
            return;
       }

       static const int RADIUS = 10;
       // Baxter's left hand
       my_left_hand_pt = tf_my_left_hand.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d my_left_hand_cv_3d_pt(my_left_hand_pt.x(), my_left_hand_pt.y(), my_left_hand_pt.z());
       cv::Point2d my_left_hand_cv_pt = cam_model.project3dToPixel(my_left_hand_cv_3d_pt);
       my_left_hand_pose.x = my_left_hand_cv_pt.x;
       my_left_hand_pose.y = my_left_hand_cv_pt.y;
       cv::circle(cv_ptr->image, my_left_hand_cv_pt, RADIUS, CV_RGB(255,255,0), -1);

       // Baxter's right hand
       my_right_hand_pt = tf_my_right_hand.getOrigin();
       // camera frame to open cv coordinate frame
       cv::Point3d my_right_hand_cv_3d_pt(my_right_hand_pt.x(), my_right_hand_pt.y(), my_right_hand_pt.z());
       cv::Point2d my_right_hand_cv_pt = cam_model.project3dToPixel(my_right_hand_cv_3d_pt);
       my_right_hand_pose.x = my_right_hand_cv_pt.x;
       my_right_hand_pose.y = my_right_hand_cv_pt.y;
       cv::circle(cv_ptr->image, my_right_hand_cv_pt, RADIUS, CV_RGB(0,255,255), -1);

       // your left hand
       cv::Point2d yr_left_hand_cv_pt(yr_left_hand_pose.x, yr_left_hand_pose.y);
       cv::circle(cv_ptr->image, yr_left_hand_cv_pt, RADIUS, CV_RGB(255,0,0), -1);
       // your right hand
       cv::Point2d yr_right_hand_cv_pt(yr_right_hand_pose.x, yr_right_hand_pose.y);
       cv::circle(cv_ptr->image, yr_right_hand_cv_pt, RADIUS, CV_RGB(0,0,255), -1);
       // ref object (table)
       cv::Point2d ref_pose_cv_pt(ref_pose.x, ref_pose.y);
       cv::circle(cv_ptr->image, ref_pose_cv_pt, RADIUS, CV_RGB(0,0,0), -1);

       // 20 is the size of offset for priniting out text

       CvPoint my_left_h_origin = cvPoint(my_left_hand_pose.x - 20, my_left_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "My Left Hand", my_left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,255,0));

       CvPoint my_right_h_origin = cvPoint(my_right_hand_pose.x - 20, my_right_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "My Right Hand", my_right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,255,255));

       CvPoint yr_left_h_origin = cvPoint(yr_left_hand_pose.x - 20, yr_left_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Your Left Hand", yr_left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0));

       CvPoint yr_right_h_origin = cvPoint(yr_right_hand_pose.x - 20, yr_right_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Your Right Hand", yr_right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,0,255));

       // display on Baxter's screen
       full_screen.header = cv_ptr->header;
       full_screen.encoding = cv_ptr->encoding;
       cv::resize(cv_ptr->image, full_screen.image, full_screen.image.size(), 0, 0, cv::INTER_NEAREST);
       robot_screen.publish(full_screen.toImageMsg());
    }
}

void FuturePrediction::predictFuture()
{
    cout << "predict Future!" << endl;

    //std::ofstream cnn_output;
    //cnn_output.open(PREDICTION_OUTPUT);

    const vector<shared_ptr<Layer<float> > >& layers = caffe_net->layers();

#if DEBUG
    for (int i = 0; i < layers.size(); ++i) {
        const caffe::string& layername = layers[i]->layer_param().name();
        cout << std::setfill(' ') << std::setw(10) << layername;
    }
#endif

    float loss = 0;
    unsigned int num_iterations = 114;
    for (int i = 0; i < num_iterations; ++i) {
        float iter_loss = 0;

        const vector<Blob<float>*>& result = caffe_net->Forward(&iter_loss);
        loss += iter_loss;
        //cout << "loss: " << loss << endl;
        const float* result_vec = result[0]->cpu_data();

        //cnn_output << i << "\t";
        // number of windows in each batch_size
        //cout << result[0]->count() << endl;
        for (int j = 0; j < result[0]->count(); ++j) {
            const float score = result_vec[j];
#if 0
            if (j == 19)
                cnn_output << score << endl;
            else
                cnn_output << score << " ";
#endif
            //cout << " window num: [ " << j << "]" << " score: " << score << endl;
        }
    }

    save_hdf_file = true;
    //cnn_output.close();
}
