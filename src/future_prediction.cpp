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
#define IMG_WIDTH 640
#define IMG_HEIGHT 480
///////////////////////////////////
// OBJECT ID DEFINITIONS
#define OBJ_ID_TABLE 0
#define OBJ_ID_PAN_1 1
#define OBJ_ID_PAN_2 2
#define OBJ_ID_PAN_3 3
#define OBJ_ID_TRIVET_1 4
#define OBJ_ID_TRIVET_2 5
#define OBJ_ID_TRIVET_3 6
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

#if 0
    for (int i = 0; i < 20; j++) {
         for (int j = 0; j < 9; i++) {
             //cout << motionMapping[i][j] << endl;
         }
     }
#endif
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
            // 1 ~ 3
            if (id == OBJ_ID_PAN_1) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_PAN_2) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            } else if (id == OBJ_ID_PAN_3) {
                    obj_pan_pose.x = (int)round((outPts.at(0).x + outPts.at(3).x)/2);
                    obj_pan_pose.y = (int)round((outPts.at(0).y + outPts.at(3).y)/2);
            }
            // 4 ~ 6
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
    unsigned int num_iterations = 1;
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
#if 0
            if (j == 19)
                cnn_output << score << endl;
            else
                cnn_output << score << " ";
#endif
            //cout << " window num: [ " << j << "]" << " score: " << score << endl;
        }
    }

    //generate robot motion
    generateRobotMotion();

    save_hdf_file = true;
    //cnn_output.close();
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
     double map_7 [] = {442, 325, 0.7608544707911652, -0.5273058958356109, -0.2044029399857314, 1.1562380188686305, 1.3594904732634479, 0.4371845245473429, -1.7042526553406947};
     for (int i = 0; i < 9; i++) {
         motionMapping[6][i] = map_7[i];
     }
     // 8th position
     double map_8 [] = {323, 331, 1.212995308020391, -0.5541505596236057, -0.27113110425874687, 1.2505778373235836, 1.4841264122791378, 0.05368932757598948, -1.4147137816273228};
     for (int i = 0; i < 9; i++) {
         motionMapping[7][i] = map_8[i];
     }
     // 9 position
     double map_9 [] = {192, 378, 1.613364293658484, -0.3857961681531816, -0.3593349995621582, 1.1182719943684667, 0.9050486648523941, -0.10469418877317949, -0.5925000793207411};
     for (int i = 0; i < 9; i++) {
         motionMapping[8][i] = map_9[i];
     }
     // 10 position
     double map_10 [] = {81, 358, 1.5443351582036402, 0.15301458359157002, -0.6005534784571395, 0.04486893804564835, 2.206247868176196, 0.14150972768242942, -1.615665264840312};
     for (int i = 0; i < 9; i++) {
         motionMapping[9][i] = map_10[i];
     }
     // 11 position
     double map_11 [] = {550, 298, 0.48397093857784806, 0.171422353046195, -0.1257864246066039, 0.029529130166794215, 1.7011846937649238, 0.0007669903939427069, -1.7345487759014315};
     for (int i = 0; i < 9; i++) {
         motionMapping[10][i] = map_11[i];
     }
     // 12 position
     double map_12 [] = {445, 294, 0.7244224270788866, 0.1277039005914607, -0.18484468494019235, 0.1277039005914607, 1.6662866308405306, 0.032213596545593685, -1.5554565189158096};
     for (int i = 0; i < 9; i++) {
         motionMapping[11][i] = map_12[i];
     }
     // 13 position
     double map_13 [] = {323, 297, 1.0730195611258468, 0.08513593372764046, -0.2822524649709161, 0.18906313210687725, 2.2561022437824723, -0.10431069357620813, -2.094267270660561};
     for (int i = 0; i < 9; i++) {
         motionMapping[12][i] = map_13[i];
     }
     // 14 position
     double map_14 [] = {191, 337, 1.3625584348392188, 0.21207284392515846, -0.2623107147284057, -0.021859226227367145,2.520330434495735, -0.18100973297047881, -2.3412381775101125};
     for (int i = 0; i < 9; i++) {
         motionMapping[13][i] = map_14[i];
     }
     // 15 position
     double map_15 [] = {143, 350, 1.4442429117941171, 0.19980099762207515, -0.28723790253154374, 0.023776702212223912, 3.057607205452601, -0.012655341500054663, -2.954446997467307};
     for (int i = 0; i < 9; i++) {
         motionMapping[14][i] = map_15[i];
     }
#if 0
     // 16 position
     double map_16 [] = {};
     for (int i = 0; i < 9; i++) {
         motionMapping[15][i] = map_16[i];
     }
     // 17 position
     double map_17 [] = {};
     for (int i = 0; i < 9; i++) {
         motionMapping[16][i] = map_17[i];
     }
     // 18 position
     double map_18 [] = {};
     for (int i = 0; i < 9; i++) {
         motionMapping[17][i] = map_18[i];
     }
     // 19 position
     double map_19 [] = {};
     for (int i = 0; i < 9; i++) {
         motionMapping[18][i] = map_19[i];
     }
     // 20 position
     double map_20 [] = {};
     for (int i = 0; i < 9; i++) {
         motionMapping[19][i] = map_20[i];
     }
#endif
}
