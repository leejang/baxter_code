#!/usr/bin/env python

import os
import sys
import threading
import rospy
import cv2
import numpy as np
import time

#sys.path.insert(0, '/home/leejang/lib/ssd_caffe/caffe/python')

caffe_root = '/home/leejang/lib/ssd_caffe/caffe'
os.chdir(caffe_root)
sys.path.insert(0, 'python')

import caffe
import re

# for Caffe
from google.protobuf import text_format
from caffe.proto import caffe_pb2

# for ROS
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from baxter_core_msgs.msg import EndpointState
import image_geometry
import tf

labelmap_file = 'data/egohands_future/labelmap_voc.prototxt'
file = open(labelmap_file, 'r')
labelmap = caffe_pb2.LabelMap()
text_format.Merge(str(file.read()), labelmap)

def get_labelname(labelmap, labels):
    num_labels = len(labelmap.item)
    labelnames = []
    if type(labels) is not list:
        labels = [labels]
    for label in labels:
        found = False
        for i in xrange(0, num_labels):
            if label == labelmap.item[i].label:
                found = True
                labelnames.append(labelmap.item[i].display_name)
                break
        assert found == True
    return labelnames

pat = re.compile("(\d+)\D*$")
def key_func(x):
        mat=pat.search(os.path.split(x)[-1]) # match last group of digits
        if mat is None:
            return x
        return "{:>10}".format(mat.group(1)) # right align to 10 digits.

# SSD 500 x 500 with auto encoder
model_def = '/home/leejang/lib/ssd_caffe/caffe/models/VGGNet/egohands/SSD_BEST_AUTOENC/deploy.prototxt'
model_weights = '/home/leejang/lib/ssd_caffe/caffe/models/VGGNet/egohands/SSD_BEST_AUTOENC/egohands_SSD_2_500x500_iter_50000.caffemodel'

# future regression model for hands
# ten con
reg_model_def = '/home/leejang/lib/ssd_caffe/caffe/models/robot_regression/robot_regression_7cv_2fc_con10_test.prototxt'
reg_model_weights = '/home/leejang/lib/ssd_caffe/caffe/models/robot_regression/7cv_2fc_con10_iter_100000.caffemodel'

reg_net = caffe.Net(reg_model_def,      # defines the structure of the model
                reg_model_weights,  # contains the trained weights
                caffe.TEST)     # use test mode (e.g., don't perform dropout)

class hands_forecasting:

    def __init__(self):
     self.bridge = CvBridge()
     #############################################################
     # subscribers
     self.image_sub = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.img_callback)
     self.left_end_sub = rospy.Subscriber("/robot/limb/left/endpoint_state", EndpointState, self.left_end_cb)
     self.right_end_sub = rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, self.right_end_cb)
     self.camera_info_sub = rospy.Subscriber("/zed/rgb/camera_info", CameraInfo, self.cam_info_cb)
     #self.camera_info_sub = rospy.Subscriber("/zed/depth/camera_info", CameraInfo, self.cam_info_cb)

     ##############################################################
     # pubishers
     self.screen_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True)

     # listner for TF
     self.listener = tf.TransformListener()

     #self.gesture_pub = rospy.Publisher("forecasting/gesture", String, queue_size=10)
     self.lock = threading.Lock()
     self.net = caffe.Net(model_def,      # defines the structure of the mode
                          model_weights,  # contains the trained weights
                          caffe.TEST)     # use test mode (e.g., don't perform dropout)

     # input preprocessing: 'data' is the name of the input blob == net.inputs[0]
     self.transformer = caffe.io.Transformer({'data': self.net.blobs['data'].data.shape})
     self.transformer.set_transpose('data', (2, 0, 1))
     self.transformer.set_mean('data', np.array([104,117,123])) # mean pixel
     self.transformer.set_raw_scale('data', 255)  # the reference model operates on images in [0,255] range instead of [0,1]
     self.transformer.set_channel_swap('data', (2,1,0))  # the reference model has channels in BGR order instead of RGB

     # for SSD 500
     self.image_resize = 500
     self.net.blobs['data'].reshape(1,3,self.image_resize,self.image_resize)

     self.image_cnt = 0;
     self.gesture_cnt = 1;

     # layers to extract features
     self.extract_layer = 'fc_e6'
     if self.extract_layer not in self.net.blobs:
       raise TypeError("Invalid layer name: " + self.extract_layer)

    def img_callback(self,data):
      # processing time check
      t = time.time()

      try:
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError, e:
        print e

      try:
        #self.listener.waitForTransform('/camera_link', '/left_gripper_base', rospy.Time.now(), rospy.Duration(3.0))        
        #(trans, rot) = self.listener.lookupTransform('/camera_link', '/left_gripper_base', rospy.Time(0))        
        #(self.trans, self.rot) = self.listener.lookupTransform('/world', '/left_gripper_base', rospy.Time(0))
        # parameters: target_frame, soource frame, time
        # returns, position as a translation (x,y,z) and orientations (x,y,z,w)
        (self.my_left_trans, self.my_left_rot) = self.listener.lookupTransform('/zed_depth_camera', '/left_gripper_base', rospy.Time(0))        
      except (tf.LookupException, tf.ConnectivityException), e:
        print e

      # write image
      target_image = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/cur_image/cur_image.jpg'
      #target_image = '/home/leejang/ros_ws/src/forecasting_gestures/script/'+str(self.image_cnt)+'.jpg'
      #print target_image
      #cv2.imwrite(target_image, self.cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

      # crop image
      #crop_img = self.cv_image[0:360, 0:640]
      #cv2.imwrite(target_image, crop_img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

      cv2.imwrite(target_image, self.cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 100])
      self.image_cnt += 1

      #print('img_calback')
      #print("image_cnt {:d} .".format(self.image_cnt))
      #print("Proesssed in {:.3f} seconds.".format(time.time() - t))

      self.lock.acquire()
      self.do_hands_forecasting()
      self.lock.release()

    def cam_info_cb(self,data):
      # camera model
      self.cam_model = image_geometry.PinholeCameraModel()
      self.cam_model.fromCameraInfo(data)
      #print('cam info callback!')

      # processing time check
    def left_end_cb(self, msg):
      tt = 1
      #print('left end state callback!')

    def right_end_cb(self, msg):
      tt = 1
      #print('right end state callback!')
    
    def do_hands_forecasting(self):

      caffe.set_device(0)
      caffe.set_mode_gpu()

      # processing time check
      t = time.time()

      print('start hands forecasting!')

      cur_image = '/home/leejang/ros_ws/src/baxter_learning_from_egocentric_video/cur_image/cur_image.jpg'

      # screen test
      cv_img = cv2.imread(cur_image)
      #cv2.putText(cv_img, 'Hana and Yuna\' Dad!', (50, 50), cv2.FONT_HERSHEY_DUPLEX, 1,(0,0,255), 5)

      #print (self.trans[0], self.trans[1], self.trans[2])
      self.my_left_2d = self.cam_model.project3dToPixel((self.my_left_trans[0], self.my_left_trans[1], self.my_left_trans[2]))

      print (int(self.my_left_2d[0]), int(self.my_left_2d[1]))

      # RED (9,0,255) BGR in CV::Mat
      cv2.circle(cv_img, (int(self.my_left_2d[0]), int(self.my_left_2d[1])), 10, (0,0,255), -1)
 
      # to publish image on Baxter's screen
      img_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
      self.screen_pub.publish(img_msg)

      # resize (batch,dim,height,width)
      #net.blobs['data'].reshape(1,3,360,640)
      """
      # load image
      cur_image = '/home/leejang/ros_ws/src/forecasting_gestures/cur_image/cur_image.jpg'
      image = caffe.io.load_image(cur_image)

      transformed_image = self.transformer.preprocess('data', image)
      self.net.blobs['data'].data[...] = transformed_image

      # Forward pass.
      detections = self.net.forward()['detection_out']

      # Extract feature vector
      extract_features = self.net.blobs[self.extract_layer].data

      if self.gesture_cnt == 1:
        self.con_extract_features = extract_features
      # concatenate ten extracted features
      elif self.gesture_cnt < 10:
        self.con_extract_features = np.concatenate((self.con_extract_features, extract_features), axis=1)
        #print con_extract_features.shape
      else:
        self.con_extract_features = np.concatenate((self.con_extract_features, extract_features), axis=1)

        # do regression
        # con
        reg_net.blobs['data'].data[...] = self.con_extract_features
        # single
        #reg_net.blobs['data'].data[...] = extract_features
        future_features = reg_net.forward()['fc2']
        #print type(future_features)

        # delete the oldest extracted features in concatanated feature maps
        self.con_extract_features = np.delete(self.con_extract_features,(range(0,256)),1)

        # do detection with future features
        self.net.blobs[self.extract_layer].data[...] = future_features
        #net.blobs[extract_layer].data[...] = extract_features
        detections = self.net.forward(start='relu_e6', end='detection_out')['detection_out']

        # Parse the outputs.
        det_label = detections[0,0,:,1]
        det_conf = detections[0,0,:,2]
        det_xmin = detections[0,0,:,3]
        det_ymin = detections[0,0,:,4]
        det_xmax = detections[0,0,:,5]
        det_ymax = detections[0,0,:,6]

        # Get detections with confidence higher than 0.65.
        top_indices = [i for i, conf in enumerate(det_conf) if conf >= 0.65]
        #top_indices = [i for i, conf in enumerate(det_conf) if conf >= 0.99]

        top_conf = det_conf[top_indices]
        top_label_indices = det_label[top_indices].tolist()
        top_labels = get_labelname(labelmap, top_label_indices)
        top_xmin = det_xmin[top_indices]
        top_ymin = det_ymin[top_indices]
        top_xmax = det_xmax[top_indices]
        top_ymax = det_ymax[top_indices]

        for i in xrange(top_conf.shape[0]):
          xmin = int(round(top_xmin[i] * image.shape[1]))
          ymin = int(round(top_ymin[i] * image.shape[0]))
          xmax = int(round(top_xmax[i] * image.shape[1]))
          ymax = int(round(top_ymax[i] * image.shape[0]))
          score = top_conf[i]
          label = int(top_label_indices[i])
          label_name = top_labels[i]
          print(" %s: %.2f" %(label_name, score))

          text = ("%s: %.2f" %(label_name, score))
          coords = xmin, ymin, xmax-xmin+1, ymax-ymin+1
          centers = (xmin + xmax)/2, (ymin + ymax)/2

          # publish gesture topic
          self.gesture_pub.publish(label_name)

      print("Proesssed in {:.3f} seconds.".format(time.time() - t))
      """
      self.gesture_cnt += 1

def main(args):
    print 'initialize hands forecasting node (python)'

    gesture_node = hands_forecasting()
    rospy.init_node('hands_forecasting', anonymous=True)
    
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print "Shutting down"

if __name__ == '__main__':
    main(sys.argv)
