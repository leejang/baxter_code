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
#define WEIGHTS_PATH "/home/leejang/data/caffe_models/robot_learning.caffemodel"
#define PREDICTION_INPUT "/home/leejang/data/caffe_models/prediction_input.h5"
#define PREDICTION_OUTPUT "/home/leejang/data/caffe_models/prediction.txt"
#define DATASET "data"
#define LABELSET "label"
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
///////////////////////////////////
// OBJECT ID DEFINITIONS
#define OBJ_ID_TABLE 0
#define OBJ_ID_PAN_1 1
#define OBJ_ID_PAN_2 2
#define OBJ_ID_PAN_3 3
#define OBJ_ID_PAN_4 4
#define OBJ_ID_PAN_5 5
#define OBJ_ID_PAN_6 6
#define OBJ_ID_PAN_7 7
#define OBJ_ID_PAN_8 8
#define OBJ_ID_PAN_9 9
#define OBJ_ID_PAN_10 10
#define OBJ_ID_TRIVET_1 11
#define OBJ_ID_TRIVET_2 12
#define OBJ_ID_TRIVET_3 13
#define OBJ_ID_TRIVET_4 14
#define OBJ_ID_TRIVET_5 15

///////////////////////////////////

template <typename T>
std::string to_string(T value)
{
    std::ostringstream os;
    os << value;
    return os.str() ;
}

FuturePrediction::FuturePrediction(ros::NodeHandle nh)
{
    this->nh = nh;

    // subscribers to get each joint position of user
    head_pose_sub = nh.subscribe("skeleton/head_joint_uv", 1, &FuturePrediction::headPoseCB, this);
    left_hand_pose_sub = nh.subscribe("skeleton/left_hand_joint_uv", 1, &FuturePrediction::leftHandPoseCB, this);
    right_hand_pose_sub = nh.subscribe("skeleton/right_hand_joint_uv", 1, &FuturePrediction::rightHandPoseCB, this);
    // for detected objects
    obj_pose_sub = nh.subscribe("objects", 1, &FuturePrediction::objPoseCB, this);
    obj_pose_w_depth_sub = nh.subscribe("objectsStamped", 1, &FuturePrediction::objPoseWDepthCB, this);

    // robot both arm's endpoints
    right_end_sub = nh.subscribe("/robot/limb/right/endpoint_state", 2, &FuturePrediction::rightEndCB, this);
    left_end_sub = nh.subscribe("/robot/limb/left/endpoint_state", 2, &FuturePrediction::leftEndCB, this);

    // publishcer to send detection completed message
    detected_pub = nh.advertise<std_msgs::UInt64>("future_prediction/done", 1);

    it = new image_transport::ImageTransport(nh);
    cam_sub = it->subscribeCamera(ROS_CAM_RGB_COLOR_PATH, 10, &FuturePrediction::onNewImageCB, this);
    robot_screen = nh.advertise<sensor_msgs::Image>("robot/xdisplay", 1, true);

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

    status = H5Dwrite(dset, H5T_NATIVE_FLOAT, H5S_ALL, H5S_ALL, H5P_DEFAULT, dataset[0]);

    save_hdf_file = true;

    full_screen.image = cv::Mat(600, 1024, CV_8UC3);

    // intialize all hands/objects poses
    my_left_hand_pose.x = 0, my_left_hand_pose.y = 0;
    my_right_hand_pose.x = 0, my_right_hand_pose.y = 0;
    yr_left_hand_pose.x = 0, yr_left_hand_pose.y = 0;
    yr_right_hand_pose.x = 0, yr_right_hand_pose.y = 0;
    obj_pan_pose.x = 0, obj_pan_pose.y = 0;
    obj_trivet_pose.x = 0, obj_trivet_pose.y = 0;
    obj_beer_box_pose.x = 0, obj_beer_box_pose.y = 0;
    obj_oatmeal_pose.x = 0, obj_oatmeal_pose.y = 0;
    obj_butter_pose.x = 0, obj_butter_pose.y = 0;
    obj_coffee_pose.x = 0, obj_coffee_pose.y = 0;

    // predicted positions
    // future my left hand pose
    f_my_left_hand_pose.x = 0, f_my_left_hand_pose.y = 0;
    f_my_right_hand_pose.x = 0, f_my_right_hand_pose.y = 0;
    f_yr_left_hand_pose.x = 0, f_yr_left_hand_pose.y = 0;
    f_yr_right_hand_pose.x = 0, f_yr_right_hand_pose.y = 0;
    f_obj_pan_pose.x = 0, f_obj_pan_pose.y = 0;
    f_obj_trivet_pose.x = 0, f_obj_trivet_pose.y = 0;

    // reference pose (table)
    ref_pose.x = 193, ref_pose.y = 237;
    got_ref_pose = true;
#if 0
    // Baxter controller without motion planning
    baxter_ctrl = new BaxterController(nh);

    // Baxter moveit controller
    baxter_moveit_ctrl = new BaxterMoveitController(nh);

    // move both arms to initial position
    double left_approach[] = {-0.42567966863820234, -0.8145437983671547,0.19213109368264808,
                               1.4012914497333255, -0.6496408636694727, 1.0653496571864198, 0.922305948716105};
    baxter_ctrl->moveLeftToJointPositions(left_approach);
    double right_approach[] = {0.22511168062218448, -0.7144515519576314, -0.13345632854603098,
                               1.2540292940963258,0.7343933022001419, 1.1428156869746333, -1.716524501643778};
    baxter_ctrl->moveRightToJointPositions(right_approach);

    loadMotionMappingTable();
#endif

#if 0
    for (int i = 0; i < 20; j++) {
         for (int j = 0; j < 9; i++) {
             //cout << motionMapping[i][j] << endl;
         }
     }
#endif

#if 1
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
#endif
    cout << "Initialization done!" <<endl;
}

FuturePrediction::~FuturePrediction()
{
#if 0
    if (caffe_net != NULL)
        delete caffe_net;
#endif
    // close all HDF5 hanles
    status = H5Pclose(dcpl);
    status = H5Pclose(lcpl);
    status = H5Dclose(dset);
    status = H5Dclose(lset);
    status = H5Sclose(dspace);
    status = H5Sclose(lspace);
    status = H5Fclose(file);

    delete baxter_ctrl;
    delete baxter_moveit_ctrl;
}


void FuturePrediction::headPoseCB(const geometry_msgs::Pose2D pose)
{
    //cout << "FP: headPoseCB Count: " << head_pose_cnt << endl;

    // my_left
    if ((my_left_hand_pose.x != 0) || (my_left_hand_pose.y != 0)) {
        // x
        dataset[0][0][0][0 + head_pose_cnt * DIM1] =
            (my_left_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][0 + head_pose_cnt * DIM1] =
            (my_left_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // my_right
    if ((my_right_hand_pose.x != 0) || (my_right_hand_pose.y != 0)) {
        // x
        dataset[0][0][0][1 + head_pose_cnt * DIM1] =
            (my_right_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][1 + head_pose_cnt * DIM1] =
            (my_right_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // your_left
    if ((yr_left_hand_pose.x != 0) || (yr_left_hand_pose.y != 0)) {
        // x
        dataset[0][0][0][2 + head_pose_cnt * DIM1] =
            (yr_left_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][2 + head_pose_cnt * DIM1] =
            (yr_left_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // your_right
    if ((yr_right_hand_pose.x != 0) || (yr_right_hand_pose.y != 0)) {
        // x
        dataset[0][0][0][3 + head_pose_cnt * DIM1] =
            (yr_right_hand_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][3 + head_pose_cnt * DIM1] =
            (yr_right_hand_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // object: pan
    if ((obj_pan_pose.x != 0) || (obj_pan_pose.y != 0)) {
        // x
        dataset[0][0][0][4 + head_pose_cnt * DIM1] =
            (obj_pan_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][4 + head_pose_cnt * DIM1] =
            (obj_pan_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // object: trivet
    if ((obj_trivet_pose.x != 0) || (obj_trivet_pose.y != 0)) {
        // x
        dataset[0][0][0][5 + head_pose_cnt * DIM1] =
            (obj_trivet_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][5 + head_pose_cnt * DIM1] =
            (obj_trivet_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // object: beer_box
    if ((obj_beer_box_pose.x != 0) || (obj_beer_box_pose.y != 0)) {
        // x
        dataset[0][0][0][6 + head_pose_cnt * DIM1] =
            (obj_beer_box_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][6 + head_pose_cnt * DIM1] =
            (obj_beer_box_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // object: oatmeal
    if ((obj_oatmeal_pose.x != 0) || (obj_oatmeal_pose.y != 0)) {
        // x
        dataset[0][0][0][7 + head_pose_cnt * DIM1] =
            (obj_oatmeal_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][7 + head_pose_cnt * DIM1] =
            (obj_oatmeal_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // object: butter
    if ((obj_butter_pose.x != 0) || (obj_butter_pose.y != 0)) {
        // x
        dataset[0][0][0][8 + head_pose_cnt * DIM1] =
            (obj_butter_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][8 + head_pose_cnt * DIM1] =
            (obj_butter_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

    // object: coffee
    if ((obj_coffee_pose.x != 0) || (obj_coffee_pose.y != 0)) {
        // x
        dataset[0][0][0][9 + head_pose_cnt * DIM1] =
            (obj_coffee_pose.x - ref_pose.x + IMG_WIDTH)/(IMG_WIDTH * 2);
        // y
        dataset[0][0][1][9 + head_pose_cnt * DIM1] =
            (obj_coffee_pose.y - ref_pose.y + IMG_HEIGHT)/(IMG_HEIGHT * 2);
    }

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
    if ((obj_pan_pose.x != 0) || (obj_trivet_pose.x != 0)) {
        yr_left_hand_pose.x = 0, yr_left_hand_pose.y = 0;
    }
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

            //cout << "object detected!" << endl;

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

            // 0
            if (id == OBJ_ID_TABLE) {

                if (!got_ref_pose) {
                    ref_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    ref_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
                    cout << "Table: " << ref_pose.x << ", " << ref_pose.y << endl;
                    got_ref_pose = true;
                }
            }
            // 1 ~ 10
            if (id == OBJ_ID_PAN_1) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_PAN_2) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_PAN_3) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_PAN_4) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_PAN_5) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            }  else if (id == OBJ_ID_PAN_6) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            }
            // 11 ~ 15
            if (id == OBJ_ID_TRIVET_1) {
                    obj_trivet_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_trivet_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_TRIVET_2) {
                    obj_trivet_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_trivet_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_TRIVET_3) {
                    obj_trivet_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_trivet_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
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

void FuturePrediction::objPoseWDepthCB(const find_object_2d::ObjectsStampedConstPtr& msg)
{
    if (msg->objects.data.size()) {
        for (unsigned int i = 0; i < msg->objects.data.size(); i += 12) {

            // get data
            int id = (int)msg->objects.data[i];
            string objectFrameId = "object_" + to_string(id);
            //cout << objectFrameId << endl;

            try
            {
                // Get transformation from "object_#" frame to target frame "torso"
                // The timestamp matches the one sent over TF
                if ((id > 3) && (id < 7)) {
                    tfListener.lookupTransform("/torso", objectFrameId, msg->header.stamp, obj_trivet_pose_w_d);
                }
            }
            catch (tf::TransformException & ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }
#if 0
            cout << "Test: " << msg->header.frame_id.c_str() << endl;

            // Here "pose" is the position of the object "id" in "/torso" frame.
            ROS_INFO("Object_%d [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
                        id, "/torso",
                        obj_trivet_pose_w_d.getOrigin().x(), obj_trivet_pose_w_d.getOrigin().y(), obj_trivet_pose_w_d.getOrigin().z(),
                        obj_trivet_pose_w_d.getRotation().x(), obj_trivet_pose_w_d.getRotation().y(),
                        obj_trivet_pose_w_d.getRotation().z(), obj_trivet_pose_w_d.getRotation().w());
#endif
        }
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
       //cout << "x: " << my_right_hand_pose.x << ", y: " << my_right_hand_pose.y <<endl;
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

       // object pan
       cv::Point2d obj_pan_pose_cv_pt(obj_pan_pose.x, obj_pan_pose.y);
       cv::circle(cv_ptr->image, obj_pan_pose_cv_pt, RADIUS, CV_RGB(0,0,0), -1);

       // object trivet
       cv::Point2d obj_trivet_pose_cv_pt(obj_trivet_pose.x, obj_trivet_pose.y);
       cv::circle(cv_ptr->image, obj_trivet_pose_cv_pt, RADIUS, CV_RGB(0,0,0), -1);

       // predicted my left hand
       cv::Point2d f_my_left_hand_pose_cv_pt(f_my_left_hand_pose.x, f_my_left_hand_pose.y);
       cv::circle(cv_ptr->image, f_my_left_hand_pose_cv_pt, RADIUS, CV_RGB(204,0,77), -1);

       // predicted my right hand
       cv::Point2d f_my_right_hand_pose_cv_pt(f_my_right_hand_pose.x, f_my_right_hand_pose.y);
       cv::circle(cv_ptr->image, f_my_right_hand_pose_cv_pt, RADIUS, CV_RGB(204,0,77), -1);

       // predicted your left hand
       cv::Point2d f_yr_left_hand_pose_cv_pt(f_yr_left_hand_pose.x, f_yr_left_hand_pose.y);
       cv::circle(cv_ptr->image, f_yr_left_hand_pose_cv_pt, RADIUS, CV_RGB(204,0,77), -1);

       // predicted your right hand
       cv::Point2d f_yr_right_hand_pose_cv_pt(f_yr_right_hand_pose.x, f_yr_right_hand_pose.y);
       cv::circle(cv_ptr->image, f_yr_right_hand_pose_cv_pt, RADIUS, CV_RGB(204,0,77), -1);

       // predicted obj pan
       cv::Point2d f_obj_pan_pose_cv_pt(f_obj_pan_pose.x, f_obj_pan_pose.y);
       cv::circle(cv_ptr->image, f_obj_pan_pose_cv_pt, RADIUS, CV_RGB(204,0,77), -1);

       // predicted my trivet
       cv::Point2d f_obj_trivet_pose_cv_pt(f_obj_trivet_pose.x, f_obj_trivet_pose.y);
       cv::circle(cv_ptr->image, f_obj_trivet_pose_cv_pt, RADIUS, CV_RGB(204,0,77), -1);
       // 20 is the size of offset for priniting out text

       CvPoint my_left_h_origin = cvPoint(my_left_hand_pose.x - 20, my_left_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "My Left Hand", my_left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,255,0));

       CvPoint my_right_h_origin = cvPoint(my_right_hand_pose.x - 20, my_right_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "My Right Hand", my_right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,255,255));

       CvPoint yr_left_h_origin = cvPoint(yr_left_hand_pose.x - 20, yr_left_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Your Left Hand", yr_left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255,0,0));

       CvPoint yr_right_h_origin = cvPoint(yr_right_hand_pose.x - 20, yr_right_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Your Right Hand", yr_right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(0,0,255));

       CvPoint f_my_left_h_origin = cvPoint(f_my_left_hand_pose.x - 20, f_my_left_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Predicted My Left Hand",f_my_left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(204,0,77));

       CvPoint f_my_right_h_origin = cvPoint(f_my_right_hand_pose.x - 20, f_my_right_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Predicted My Right Hand",f_my_right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(204,0,77));

       CvPoint f_yr_left_h_origin = cvPoint(f_yr_left_hand_pose.x - 20, f_yr_left_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Predicted Your Left Hand",f_yr_left_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(204,0,77));

       CvPoint f_yr_right_h_origin = cvPoint(f_yr_right_hand_pose.x - 20, f_yr_right_hand_pose.y - 20);
       cv::putText(cv_ptr->image, "Predicted Your Rgiht Hand",f_yr_right_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(204,0,77));

       CvPoint f_obj_pan_h_origin = cvPoint(f_obj_pan_pose.x - 20, f_obj_pan_pose.y - 20);
       cv::putText(cv_ptr->image, "Predicted Obj Pan",f_obj_pan_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(204,0,77));

       CvPoint f_obj_trivet_h_origin = cvPoint(f_obj_trivet_pose.x - 20, f_obj_trivet_pose.y - 20);
       cv::putText(cv_ptr->image, "Predicted Obj Trivet",f_obj_trivet_h_origin, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(204,0,77));

       // display on Baxter's screen
       full_screen.header = cv_ptr->header;
       full_screen.encoding = cv_ptr->encoding;
       cv::resize(cv_ptr->image, full_screen.image, full_screen.image.size(), 0, 0, cv::INTER_NEAREST);
       robot_screen.publish(full_screen.toImageMsg());
    }
}

void FuturePrediction::leftEndCB(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    cur_pose_left.position.x = msg->pose.position.x;
    cur_pose_left.position.y = msg->pose.position.y;
    cur_pose_left.position.z = msg->pose.position.z;

    cur_pose_left.orientation.x = msg->pose.orientation.x;
    cur_pose_left.orientation.y = msg->pose.orientation.y;
    cur_pose_left.orientation.z = msg->pose.orientation.z;
    cur_pose_left.orientation.w = msg->pose.orientation.w;
#if DEBUG
    cout << right << "left hand positions: " <<
        cur_pose_left.position.x << ","  << cur_pose_left.position.y << "," << cur_pose_left.position.z << endl;
    cout << right << "left hand orientations: " <<
        cur_pose_left.orientation.x << "," << cur_pose_left.orientation.y << "," <<
        cur_pose_left.orientation.z << "," << cur_pose_left.orientation.w << endl;
#endif
}

void FuturePrediction::rightEndCB(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    cur_pose_right.position.x = msg->pose.position.x;
    cur_pose_right.position.y = msg->pose.position.y;
    cur_pose_right.position.z = msg->pose.position.z;

    cur_pose_right.orientation.x = msg->pose.orientation.x;
    cur_pose_right.orientation.y = msg->pose.orientation.y;
    cur_pose_right.orientation.z = msg->pose.orientation.z;
    cur_pose_right.orientation.w = msg->pose.orientation.w;
#if DEBUG
    cout << right << "right hand positions: " <<
        cur_pose_right.position.x << ","  << cur_pose_right.position.y << "," << cur_pose_right.position.z << endl;
    cout << right << "right hand orientations: " <<
        cur_pose_right.orientation.x << "," << cur_pose_right.orientation.y << "," <<
        cur_pose_right.orientation.z << "," << cur_pose_right.orientation.w << endl;
#endif
}

void FuturePrediction::predictFuture()
{
    //cout << "predict Future!" << endl;

#if 0
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

#endif
    std::ofstream cnn_output;
    cnn_output.open(PREDICTION_OUTPUT);

    const vector<shared_ptr<Layer<float> > >& layers = caffe_net->layers();

#if DEBUG
    for (int i = 0; i < layers.size(); ++i) {
        const caffe::string& layername = layers[i]->layer_param().name();
        cout << std::setfill(' ') << std::setw(10) << layername;
    }
#endif

    float loss = 0;
    unsigned int num_iterations = 235;
    for (int i = 0; i < num_iterations; ++i) {
        float iter_loss = 0;

        const vector<Blob<float>*>& result = caffe_net->Forward(&iter_loss);
        loss += iter_loss;
        //cout << "loss: " << loss << endl;
        const float* result_vec = result[0]->cpu_data();

        cnn_output << i << "\t";
        // number of windows in each batch_size
        //cout << result[0]->count() << endl;
        for (int j = 0; j < result[0]->count(); ++j) {
            const float score = result_vec[j];

#if 0
            if (j == 0) {
                //cout << " predicted my hand: [ " << j << "]" << " score: " << score << endl;
                f_my_left_hand_pose.x = (int)round((score - 0.5)*IMG_WIDTH*2) + ref_pose.x;
            }
            if (j == 10) {
                //cout << " predicted my hand: [ " << j << "]" << " score: " << score << endl;
                f_my_left_hand_pose.y = (int)round((score - 0.5)*IMG_HEIGHT*2) + ref_pose.y;
            }
            if (j == 1) {
                //cout << " predicted my hand: [ " << j << "]" << " score: " << score << endl;
                f_my_right_hand_pose.x = (int)round((score - 0.5)*IMG_WIDTH*2) + ref_pose.x;
                //cout << "predicted my right x: " << f_my_right_hand_pose.x << "ref_pose.x: " << ref_pose.x << endl;
            }
            if (j == 11) {
                //cout << " predicted my hand: [ " << j << "]" << " score: " << score << endl;
                f_my_right_hand_pose.y = (int)round((score - 0.5)*IMG_HEIGHT*2) + ref_pose.y;
                //cout << "predicted my right y: " << f_my_right_hand_pose.y << endl;
            }
            if (j == 2) {
                f_yr_left_hand_pose.x = (int)round((score - 0.5)*IMG_WIDTH*2) + ref_pose.x;
            }
            if (j == 12) {
                f_yr_left_hand_pose.y = (int)round((score - 0.5)*IMG_HEIGHT*2) + ref_pose.y;
            }

            if (j == 3) {
                f_yr_right_hand_pose.x = (int)round((score - 0.5)*IMG_WIDTH*2) + ref_pose.x;
            }
            if (j == 13) {
                f_yr_right_hand_pose.y = (int)round((score - 0.5)*IMG_HEIGHT*2) + ref_pose.y;
            }

            if (j == 4) {
                f_obj_pan_pose.x = (int)round((score - 0.5)*IMG_WIDTH*2) + ref_pose.x;
            }
            if (j == 14) {
                f_obj_pan_pose.y = (int)round((score - 0.5)*IMG_HEIGHT*2) + ref_pose.y;
            }

            if (j == 5) {
                f_obj_trivet_pose.x = (int)round((score - 0.5)*IMG_WIDTH*2) + ref_pose.x;
            }
            if (j == 15) {
                f_obj_trivet_pose.y = (int)round((score - 0.5)*IMG_HEIGHT*2) + ref_pose.y;
            }
#endif
            if (j == 19)
                cnn_output << score << endl;
            else
                cnn_output << score << " ";

            //cout << " window num: [ " << j << "]" << " score: " << score << endl;
        }
    }

    //generate robot motion
    //generateRobotMotion();

    //delete caffe_net;

    save_hdf_file = true;
    cnn_output.close();
}

void FuturePrediction::generateRobotMotion()
{
    cout << "generate robot motion!" << endl;

    unsigned int old_dist = 1000;
    unsigned int new_dist = 0;
    unsigned int motion_id = 0;
#if 0
    // compute distance
    // between predicted my right hand pose and current target obj (trivet) pose
    int dist1 = getDistance(f_my_right_hand_pose.x, f_my_right_hand_pose.y,
                           obj_trivet_pose.x, obj_trivet_pose.y);
#endif
    //cout << "Distance b/w my hands and trivet: " << dist1 << endl;

    // searching in motion mapping table
    double move_to[NUM_OF_JOINTS] = {};

    for (int i = 0; i < NUM_OF_MOTIONS; i++) {

        new_dist = getDistance(f_my_right_hand_pose.x, f_my_right_hand_pose.y,
                           motionMapping[i][0], motionMapping[i][1]);

        if (new_dist < old_dist) {
            old_dist = new_dist;
            motion_id = i;

            for (int j = 0; j < NUM_OF_JOINTS; j++) {
                move_to[j] = motionMapping[i][j+2];
            }
        }
    }

       cout << "Motion ID to generate: " << motion_id << endl;
       baxter_ctrl->moveRightToJointPositions(move_to);
}

int FuturePrediction::getDistance(int x1, int y1, int x2, int y2)
{
     int x = x1 - x2;
     int y = y1 - y2;
     int dist;

     dist = pow(x, 2) + pow(y, 2);
     dist = sqrt(dist);

     return dist;
}

void FuturePrediction::loadMotionMappingTable()
{
     //cout << "load Motion Mapping table" << endl;
     // first position
     double map_1 [] = {622, 403,0.22511168062218448, -0.7144515519576314, -0.13345632854603098, 1.2540292940963258,0.7343933022001419, 1.1428156869746333, -1.716524501643778};
     for (int i = 0; i < 9; i++) {
         motionMapping[0][i] = map_1[i];
     }
     // second position
     double map_2 [] = {467, 417,0.749733110078996, -1.0108933392164876, -0.23508255574343967, 1.9462381246296188, 1.1504855909140603, 0.5656554155327463, -1.737233242280231};
     for (int i = 0; i < 9; i++) {
         motionMapping[1][i] = map_2[i];
     }
     // third position
     double map_3 [] = {342, 423, 0.9384127469889019, -0.9618059540041545, -0.13460681413694506, 1.982286673144926, 1.5286118551278147, 0.7040971816394049, -1.9247623935992229};
     for (int i = 0; i < 9; i++) {
         motionMapping[2][i] = map_3[i];
     }
     // 4th position
     double map_4 [] = {230, 392, 1.2340875438538152, -0.7163690279424882, -0.19750002644024703, 1.3859516418544713, 1.1873011298233103, 0.8739855538977145, -1.0638156763985345};
     for (int i = 0; i < 9; i++) {
         motionMapping[3][i] = map_4[i];
     }
     // 5th position
     double map_5 [] = {142, 407, 1.5121215616580466, -0.4536748180171111, -0.3482136388499889, 1.0388884885953964, 1.3487526077482501, 0.7673738891396782, -1.4438594165971457};
     for (int i = 0; i < 9; i++) {
         motionMapping[4][i] = map_5[i];
     }
     // 6th position
     double map_6 [] = {550, 334, 0.4916408425172751, -0.5395777421386942, -0.15339807878854136, 1.1385972398079482, 1.2110778320355342, 0.3681553890924993, -1.5335972926884425};
     for (int i = 0; i < 9; i++) {
         motionMapping[5][i] = map_6[i];
     }
     // 7th position
     double map_7 [] = {481, 355,0.698728248881806, -0.6945098017151211, -0.28455343615274425, 1.4568982532941717, 1.393238050596927, 0.6419709597300457, -1.8208351952199862};
     for (int i = 0; i < 9; i++) {
         motionMapping[6][i] = map_7[i];
     }
     // 8th position
     double map_8 [] = {416, 362, 0.9997719785043184, -0.8057234088368136, -0.387330148941067, 1.6582332317041322, 1.5013836961428486, 0.5135000687446423, -1.817383738447244};
     for (int i = 0; i < 9; i++) {
         motionMapping[7][i] = map_8[i];
     }
     // 9 position
     double map_9 [] = {355, 362, 1.1869176346263388, -0.7635389371699647, -0.324436936637765, 1.631005072719166, 1.7226604247953197, 0.228946632591898, -1.744903146219658};
     for (int i = 0; i < 9; i++) {
         motionMapping[8][i] = map_9[i];
     }
     // 10 position
     double map_10 [] = {281, 360,1.318456487187513, -0.6327670750027332, -0.30027673922856973, 1.4066603824909245, 1.6367575006737365, 0.23508255574343967, -1.4718545659760545};
     for (int i = 0; i < 9; i++) {
         motionMapping[9][i] = map_10[i];
     }
     // 11 position
     double map_11 [] = {224, 363,1.3947720313848124, -0.559902987578176, -0.18906313210687725, 1.2693691019751798, 1.7184419776286348, 0.08015049616701286, -1.4845099074761092};
     for (int i = 0; i < 9; i++) {
         motionMapping[10][i] = map_11[i];
     }
     // 12 position
     double map_12 [] = {165, 358, 1.4944807825973643, -0.4306651061988299, -0.20708740636453085, 1.0139613007922585, 1.6183497312191115, 0.08782040010643993, -1.4177817432030937};
     for (int i = 0; i < 9; i++) {
         motionMapping[11][i] = map_12[i];
     }
     // 13 position
     double map_13 [] = {480, 316, 0.7113835903818606, -0.5414952181235511, -0.22434469022824177, 1.1125195664138963, 1.1431991821716045, 0.35703402838033005, -1.3188399823844845};
     for (int i = 0; i < 9; i++) {
         motionMapping[12][i] = map_13[i];
     }
     // 14 position
     double map_14 [] = {418, 309, 0.7113835903818606, -0.5414952181235511, -0.22511168062218448, 1.1113690808229824, 1.143582677368576, 0.35626703798638737, -1.3188399823844845};
     for (int i = 0; i < 9; i++) {
         motionMapping[13][i] = map_14[i];
     }
     // 15 position
     double map_15 [] = {355, 316, 0.9832816850345502, -0.47284957786567877, -0.12808739578843203, 1.0891263593986438, 1.594956524203859, 0.1514806028036846, -1.597257495385687};
     for (int i = 0; i < 9; i++) {
         motionMapping[14][i] = map_15[i];
     }
     // 16 position
     double map_16 [] = {278, 318, 1.291228328202547, -0.3608689803500436, -0.253873820395036, 0.9280583766706754, 1.5650438988400934, -0.0847524385306691, -1.3357137710512241};
     for (int i = 0; i < 9; i++) {
         motionMapping[15][i] = map_16[i];
     }
     // 17 position
     double map_17 [] = {228, 320, 1.367160377202875, -0.05292233718204677, -0.31139809994073897, 0.4333495725776294, 1.9216944320234521, -0.17103885784922362, -1.6486458517798483};
     for (int i = 0; i < 9; i++) {
         motionMapping[16][i] = map_17[i];
     }
     // 18 position
     double map_18 [] = {160, 336, 1.4212331999758359, 0.1936650744705335, -0.2676796474860047, -0.01227184630308331, 2.0240876496148035, -0.13614079492483047, -1.8085633489169028};
     for (int i = 0; i < 9; i++) {
         motionMapping[17][i] = map_18[i];
     }
     // 19 position
     double map_19 [] = {81, 358, 1.5443351582036402, 0.15301458359157002, -0.6005534784571395, 0.04486893804564835, 2.206247868176196, 0.14150972768242942, -1.615665264840312};
     for (int i = 0; i < 9; i++) {
         motionMapping[18][i] = map_19[i];
     }
     // 20 position
     double map_20 [] = {550, 298, 0.48397093857784806, 0.171422353046195, -0.1257864246066039, 0.029529130166794215, 1.7011846937649238, 0.0007669903939427069, -1.7345487759014315};
     for (int i = 0; i < 9; i++) {
         motionMapping[19][i] = map_20[i];
     }
     // 21 position
     double map_21 [] = {482, 292, 0.6170437719269076, 0.146495165243057, -0.15493205957642678, 0.052155346788104066, 1.2785729867024924, 0.09357282806101024, -1.226034144717417};
     for (int i = 0; i < 9; i++) {
         motionMapping[20][i] = map_20[i];
     }
     // 22 position
     double map_22 [] = {418, 292, 0.8038059328519568, 0.07746602978821339, -0.18637866572807776, 0.19941750242510378, 1.4120293152485233, 0.0007669903939427069, -1.3380147422330522};
     for (int i = 0; i < 9; i++) {
         motionMapping[21][i] = map_20[i];
     }
     // 23 position
     double map_23 [] = {352, 302, 0.9898011033830633, -0.23431556534949696, -0.15339807878854136, 0.7079321336091184, 1.4526798061274868, 0.06864564025787226, -1.39975746894544};
     for (int i = 0; i < 9; i++) {
         motionMapping[22][i] = map_20[i];
     }
     // 24 position
     double map_24 [] = {292, 308, 1.15892248524743, -0.05062136600021865, -0.26384469551629114, 0.43795151494128565, 1.683927409901213, 0.004218447166684887, -1.5013836961428486};
     for (int i = 0; i < 9; i++) {
         motionMapping[23][i] = map_20[i];
     }
     // 25 position
     double map_25 [] = {227, 319, 1.2985147369450027, 0.0015339807878854137, -0.13115535736420286, 0.31676703269833795, 1.8960002538263714, -0.16106798272796843, -1.9113400617052256};
     for (int i = 0; i < 9; i++) {
         motionMapping[24][i] = map_20[i];
     }
     // 26 position
     double map_26 [] = {159, 346,1.4304370847031482, 0.19673303604630432, -0.27496605622846043, 0.02684466378799474, 1.8231361664018142, -0.10891263593986437, -1.6206507024009396};
     for (int i = 0; i < 9; i++) {
         motionMapping[25][i] = map_20[i];
     }
     // 27 position
     double map_27 [] = {143, 350, 1.4442429117941171, 0.19980099762207515, -0.28723790253154374, 0.023776702212223912, 3.057607205452601, -0.012655341500054663, -2.954446997467307};
     for (int i = 0; i < 9; i++) {
         motionMapping[26][i] = map_20[i];
     }
}
